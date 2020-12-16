//
// Created by ziv on 2020/10/26.
//

#include "helper.h"
#include "keyframe.h"
#include "utils.h"
#include "map_generator.h"
#include "factors.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/gicp.h>

static int correct_count = 1;
static int save_count = 1;
static int graph_count = 1;

class LaserOdometry {

public:
    using LoopFactor = gtsam::NonlinearFactor::shared_ptr;

public:

    void pointCloudFullHandler(const sensor_msgs::PointCloud2ConstPtr &pointCloudMsg) {
        std::lock_guard lg(pcd_msg_mtx);
        pointCloudFullBuf.push(pointCloudMsg);
    }

    void pointCloudSurfHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
        std::lock_guard lg(pcd_msg_mtx);
        pointCloudSurfBuf.push(laserCloudMsg);
    }

    void pointCloudEdgeHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
        std::lock_guard lg(pcd_msg_mtx);
        pointCloudEdgeBuf.push(laserCloudMsg);
    }

    void gpsHandler(const sensor_msgs::NavSatFixConstPtr& navsat_msg) {
        std::lock_guard lock(gps_msg_mtx);
        geographic_msgs::GeoPointStampedPtr gps_msg(new geographic_msgs::GeoPointStamped());
        gps_msg->header = navsat_msg->header;
        gps_msg->position.latitude = navsat_msg->latitude;
        gps_msg->position.longitude = navsat_msg->longitude;
        gps_msg->position.altitude = navsat_msg->altitude;
        gps_queue.push(gps_msg);
    }

    void pointAssociate(PointT const *const pi, PointT *const po, const gtsam::Pose3& odom) {
        Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
        Eigen::Vector3d point_w = odom.rotation() * point_curr + odom.translation();
        po->x = point_w.x();
        po->y = point_w.y();
        po->z = point_w.z();
        po->intensity = pi->intensity;
    }

    void downsampling(const pcl::PointCloud<PointT>::Ptr& input, pcl::PointCloud<PointT>& output, FeatureType featureType) {
        if (input == nullptr) {
            cout << "input is nullptr!" << endl;
            return;
        }
        if (featureType == FeatureType::Edge) {
            downSizeFilterEdge.setInputCloud(input);
            downSizeFilterEdge.filter(output);
        } else if (featureType == FeatureType::Surf) {
            downSizeFilterSurf.setInputCloud(input);
            downSizeFilterSurf.filter(output);
        } else {
            downSizeFilterNor.setInputCloud(input);
            downSizeFilterNor.filter(output);
        }
    }

    void updateSlideWindows() {
        if (current_keyframe->valid_frames == 1) {
            int size = keyframes.size();
            int start = size < SLIDE_KEYFRAME_LEN ? 0 : size - SLIDE_KEYFRAME_LEN;

            slideWindowEdge = generate_cloud(keyframes, start, size, FeatureType::Edge);
            slideWindowSurf = generate_cloud(keyframes, start, size, FeatureType::Surf);
            downsampling(slideWindowEdge, *slideWindowEdge, FeatureType::Edge);
            downsampling(slideWindowSurf, *slideWindowSurf, FeatureType::Edge);

            sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2());
            pcl::toROSMsg(*slideWindowEdge, *cloud_msg);
            cloud_msg->header.stamp = cloud_in_time;
            cloud_msg->header.frame_id = "map";
            pub_sw_edge.publish(cloud_msg);
            pcl::toROSMsg(*slideWindowSurf, *cloud_msg);
            cloud_msg->header.stamp = cloud_in_time;
            cloud_msg->header.frame_id = "map";
            pub_sw_surf.publish(cloud_msg);
        }
    }

    bool nextFrameToBeKeyframe() {
        Eigen::Isometry3d T_delta((current_keyframe->pose.inverse() * odom).matrix());
        Eigen::Quaterniond q_delta(T_delta.rotation().matrix());
        q_delta.normalize();
        Eigen::Vector3d eulerAngle = q_delta.toRotationMatrix().eulerAngles(2, 1, 0);

        bool isKeyframe = min(abs(eulerAngle.x()), abs(abs(eulerAngle.x()) - M_PI)) > keyframeAngleThreshold ||
                          min(abs(eulerAngle.y()), abs(abs(eulerAngle.y()) - M_PI)) > keyframeAngleThreshold ||
                          min(abs(eulerAngle.z()), abs(abs(eulerAngle.z()) - M_PI)) > keyframeAngleThreshold ||
                          T_delta.translation().norm() > keyframeDistThreshold;
        return isKeyframe;
    }

    void pubOdomAndPath() {

        auto odometry_odom = poseToNavOdometry(cloud_in_time, odom, "map", "base_link");
        auto odometry_opti = poseToNavOdometry(cloud_in_time, odom, "map", "base_link");

        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(odom.translation().x(), odom.translation().y(), odom.translation().z()) );
        Eigen::Quaterniond q_current(odom.rotation().matrix());
        tf::Quaternion q(q_current.x(),q_current.y(),q_current.z(),q_current.w());
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, cloud_in_time, "map", "base_link"));

        geometry_msgs::PoseStamped laserPose;
        laserPose.header = odometry_opti.header;
        laserPose.pose = odometry_opti.pose.pose;
        path_opti.header.stamp = odometry_opti.header.stamp;
        path_opti.poses.clear();
        for (size_t i = 0; i < keyframes.size(); i++) {
            laserPose.header = odometry_opti.header;
            Eigen::Quaterniond q(keyframes[i]->pose.rotation().matrix());
            laserPose.pose.orientation.x    = q.x();
            laserPose.pose.orientation.y    = q.y();
            laserPose.pose.orientation.z    = q.z();
            laserPose.pose.orientation.w    = q.w();
            laserPose.pose.position.x       = keyframes[i]->pose.translation().x();
            laserPose.pose.position.y       = keyframes[i]->pose.translation().y();
            laserPose.pose.position.z       = keyframes[i]->pose.translation().z();
            path_opti.poses.push_back(laserPose);
        }
        path_opti.header.frame_id = "map";
        pub_path_opti.publish(path_opti);

        laserPose.header = odometry_odom.header;
        laserPose.pose = odometry_odom.pose.pose;
        path_odom.header.stamp = odometry_odom.header.stamp;
        path_odom.poses.push_back(laserPose);
        path_odom.header.frame_id = "map";
        pub_path_odom.publish(path_odom);


        sensor_msgs::PointCloud2Ptr edge_msg(new sensor_msgs::PointCloud2());
        sensor_msgs::PointCloud2Ptr surf_msg(new sensor_msgs::PointCloud2());
//        sensor_msgs::PointCloud2Ptr full_msg(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(*cloud_in_edge, *edge_msg);
        pcl::toROSMsg(*cloud_in_surf, *surf_msg);
//        pcl::toROSMsg(*cloud_in_full, *surf_msg);
        edge_msg->header.stamp = cloud_in_time;
        edge_msg->header.frame_id = "base_link";
        surf_msg->header.stamp = cloud_in_time;
        surf_msg->header.frame_id = "base_link";
//        full_msg->header.stamp = cloud_in_time;
//        full_msg->header.frame_id = "base_link";
        pub_curr_edge.publish(edge_msg);
        pub_curr_surf.publish(surf_msg);
//        pub_curr_full.publish(full_msg);
        cout << "edge: " << cloud_in_edge->size() << endl;
        cout << "surf: " << cloud_in_surf->size() << endl;
//        cout << "full: " << cloud_in_full->size() << endl;

    }

    int addEdgeCostFactor(
        const pcl::PointCloud<PointT>::Ptr& pc_in,
        const pcl::PointCloud<PointT>::Ptr& map_in,
        const pcl::KdTreeFLANN<PointT>::Ptr& kdtreeEdge,
        const gtsam::Pose3& odom,
        gtsam::NonlinearFactorGraph& factors,
        int state_key) {

        int corner_num = 0;
        PointT point_temp;
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        for (int i = 0; i < (int) pc_in->points.size(); i++) {

            pointAssociate(&(pc_in->points[i]), &point_temp, odom);
            kdtreeEdge->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis);

            if (pointSearchSqDis[4] < 1.0) {
                std::vector<Eigen::Vector3d> nearCorners;
                Eigen::Vector3d center(0, 0, 0);
                for (int j = 0; j < 5; j++) {
                    Eigen::Vector3d tmp(map_in->points[pointSearchInd[j]].x,
                                        map_in->points[pointSearchInd[j]].y,
                                        map_in->points[pointSearchInd[j]].z);
                    center = center + tmp;
                    nearCorners.push_back(tmp);
                }
                center = center / 5.0;

                Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
                for (int j = 0; j < 5; j++) {
                    Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
                    covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
                }

                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

                Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
                Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);
                if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]) {
                    Eigen::Vector3d point_on_line = center;
                    Eigen::Vector3d point_a, point_b;
                    point_a = 0.1 * unit_direction + point_on_line;
                    point_b = -0.1 * unit_direction + point_on_line;

                    factors.emplace_shared<gtsam::PointToEdgeFactor>(
                            X(state_key), curr_point, point_a, point_b, edge_noise_model);
                    corner_num++;
                }
            }
        }

        if (corner_num < 20) {
            printf("not enough correct points\n");
        }
        return corner_num;

    }

    int addSurfCostFactor(
        const pcl::PointCloud<PointT>::Ptr& pc_in,
        const pcl::PointCloud<PointT>::Ptr& map_in,
        const pcl::KdTreeFLANN<PointT>::Ptr& kdtreeSurf,
        const gtsam::Pose3& odom,
        gtsam::NonlinearFactorGraph& factors,
        int state_key) {

        int surf_num=0;
        PointT point_temp;
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        for (int i = 0; i < (int)pc_in->points.size(); i++) {

            pointAssociate(&(pc_in->points[i]), &point_temp, odom);
            kdtreeSurf->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis);

            Eigen::Matrix<double, 5, 3> matA0;
            Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
            if (pointSearchSqDis[4] < 1.0) {

                for (int j = 0; j < 5; j++) {
                    matA0(j, 0) = map_in->points[pointSearchInd[j]].x;
                    matA0(j, 1) = map_in->points[pointSearchInd[j]].y;
                    matA0(j, 2) = map_in->points[pointSearchInd[j]].z;
                }
                // find the norm of plane
                Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
                double negative_OA_dot_norm = 1 / norm.norm();
                norm.normalize();

                /*
                 * ne equation: nT p + d = 0;
                 * nT p / d = -1; (matB0 is -1)
                 * nT is norm, matA0 is p [x,3]
                 * pT n / d = -1 => Ax = b
                 * x = n / d => ||n|| = 1, d = ||x / n|| = 1 / ||x.norm()||, n = n.normailzed
                */
                bool planeValid = true;
                for (int j = 0; j < 5; j++) {
                    // if OX * n > 0.2, then plane is not fit well
                    if (fabs(norm(0) * map_in->points[pointSearchInd[j]].x +
                             norm(1) * map_in->points[pointSearchInd[j]].y +
                             norm(2) * map_in->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2) {
                        planeValid = false;
                        break;
                    }
                }
                Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);
                if (planeValid) {
                    factors.emplace_shared<gtsam::PointToPlaneFactor>(
                            X(state_key), curr_point, norm, negative_OA_dot_norm, surf_noise_model);
                    surf_num++;
                }
            }

        }

        if (surf_num < 20) {
            printf("not enough correct points\n");
        }

        return surf_num;

    }


//    gtsam::Pose3 predict_pose() {
//
//        if (keyframes.size() == 1 && keyframes.front()->sub_frames.size() == 1) {
//            return gtsam::Pose3::identity();
//        }
//
//        auto& sub_frames = current_keyframe->sub_frames;
//        if (sub_frames.empty()) {
//            sub_frames = keyframes[keyframes.size() - 2]->sub_frames;
//            if (sub_frames.size() > 1) {
//                delta = sub_frames[sub_frames.size() - 2]->pose.inverse() * sub_frames.back()->pose;
//                return pose_normalize(sub_frames.back()->pose * delta);
//            } else {
//                throw "sub frames size less than 2!";
//            }
//        } else if (sub_frames.size() == 1) {
//            delta = keyframes[keyframes.size() - 2]->sub_frames.back()->pose.inverse() * sub_frames.back()->pose;
//        } else {
//            delta = sub_frames[sub_frames.size() - 2]->pose.inverse() * sub_frames.back()->pose;
//        }
//        return pose_normalize(sub_frames.back()->pose * delta);
//
//    }

    void getTransToSlideWindow(pcl::PointCloud<PointT>::Ptr currEdgeDS, pcl::PointCloud<PointT>::Ptr currSurfDS) {

        gtsam::Pose3 odom_prediction = pose_normalize(odom * delta);

        last_odom = odom;
        odom = odom_prediction;

        // 'pose_w_c' to be optimize
        pose_w_c = odom;

        const auto state_key = X(0);

        std::cout << "currEdgeDS      size: " << currEdgeDS->size() << std::endl;
        std::cout << "currSurfDS      size: " << currSurfDS->size() << std::endl;
        std::cout << "slideWindowEdge size: " << slideWindowEdge->size() << std::endl;
        std::cout << "slideWindowSurf size: " << slideWindowSurf->size() << std::endl;

        if (slideWindowEdge->size() > 10 && slideWindowSurf->size() > 50) {

            kdtreeEdgeSW->setInputCloud(slideWindowEdge);
            kdtreeSurfSW->setInputCloud(slideWindowSurf);

            for (int opti_counter = 0; opti_counter < 3; opti_counter++) {

                gtsam::NonlinearFactorGraph factors;
                gtsam::Values init_values;
                init_values.insert(state_key, pose_w_c);

                addEdgeCostFactor(currEdgeDS, slideWindowEdge, kdtreeEdgeSW, pose_w_c, factors, 0);
                addSurfCostFactor(currSurfDS, slideWindowSurf, kdtreeSurfSW, pose_w_c, factors, 0);

                // gtsam
                gtsam::LevenbergMarquardtParams params;
                params.setLinearSolverType("MULTIFRONTAL_CHOLESKY");
                params.setRelativeErrorTol(1e-4);
                params.maxIterations = 6;

                auto result = gtsam::LevenbergMarquardtOptimizer(factors, init_values, params).optimize();

                pose_w_c = result.at<gtsam::Pose3>(state_key);

            }

        } else {
            printf("not enough points in submap to associate, error\n");
        }

        odom = pose_w_c;
    }

    void initOdomFactor() {

        poseGraph.add(gtsam::PriorFactor<gtsam::Pose3>(X(0), odom, prior_noise_model));
        initEstimate.insert(X(0), odom);
        cout << "initGTSAM" << endl;

        copyGraph.add(gtsam::PriorFactor<gtsam::Pose3>(X(0), odom, prior_noise_model));
        copyEstimate.insert(X(0), odom);

    }

    void addOdomFactor(int lastPoseIdx) {

        int currIdx = current_keyframe->index;
        auto lastPose = keyframes[lastPoseIdx]->pose;

        gtsam::Pose3 pose_last_curr = lastPose.between(current_keyframe->pose);

        poseGraph.add(gtsam::BetweenFactor<gtsam::Pose3>(X(lastPoseIdx), X(currIdx), pose_last_curr, odometry_noise_model));
        initEstimate.insert(X(currIdx), current_keyframe->pose);
        printf("add odom factor between %d and %d\n", lastPoseIdx, currIdx);

        copyGraph.add(gtsam::BetweenFactor<gtsam::Pose3>(X(lastPoseIdx), X(currIdx), pose_last_curr, odometry_noise_model));
        copyEstimate.insert(X(currIdx), current_keyframe->pose);

    }

    void handleRegistration(pcl::PointCloud<PointT>::Ptr currEdgeDS, pcl::PointCloud<PointT>::Ptr currSurfDS) {

        if (!current_keyframe->is_init()) {
            int lastIndex = current_keyframe->index - 1;

            // push to loopDetectBuf
            loop_mtx.lock();
            loopDetectBuf.push(keyframes[lastIndex]);
            loop_mtx.unlock();
            current_keyframe->set_init(odom);
//            savePCDandPose();
            addOdomFactor(lastIndex);
            factorGraphUpdate();

        } else {
            is_keyframe_next = nextFrameToBeKeyframe();
            delta = pose_normalize(last_odom.inverse() * odom);
            current_keyframe->valid_frames++;
        }

    }

    void factorGraphUpdate() {

        isam->update(poseGraph, initEstimate);
        isam->update();

        if (loop_found) {
            isam->update();
            isam->update();
            isam->update();
        }

        poseGraph.resize(0);
        initEstimate.clear();

        isamOptimize = isam->calculateEstimate();

        if (loop_found) {
            correctPose();
            loop_found = false;
            regenerate_map = true;
        }

        auto latestEstimate = isamOptimize.at<gtsam::Pose3>(X(keyframes.size() - 1));
        delta = pose_normalize(last_odom.inverse() * odom);
        odom = latestEstimate;
        current_keyframe->pose = odom;

    }

    void correctPose() {
        for (size_t i = 0; i < isamOptimize.size(); i++) {
            auto poseOpti = isamOptimize.at<gtsam::Pose3>(X(i));
            keyframes[i]->pose = poseOpti;
        }
    }

    void saveOdomGraph() {
        std::ofstream if_graph("/home/ziv/mloam/mloam.dot");
        copyGraph.saveGraph(if_graph, copyEstimate);
        if_graph.close();
    }

    void savePCDandPose() {
        int index = current_keyframe->index;
        string filepath = "/home/ziv/mloam/featureExtraction/" + std::to_string(index) + "/";
        _mkdir(filepath + "1.txt");

        pcl::io::savePCDFileASCII(filepath + "edge.pcd", *cloud_in_edge);
        pcl::io::savePCDFileASCII(filepath + "surf.pcd", *cloud_in_surf);
//        pcl::io::savePCDFileASCII(filepath + "full.pcd", *cloud_in_full);
        std::ofstream f(filepath + "odom.txt");
        f << odom.matrix() << endl;
        f.close();
        cout << index << ":\n" << odom.matrix() << endl;
    }

    void update(pcl::PointCloud<PointT>::Ptr currEdge, pcl::PointCloud<PointT>::Ptr currSurf, pcl::PointCloud<PointT>::Ptr currFull) {

        pcl::PointCloud<PointT>::Ptr currEdgeDS(new pcl::PointCloud<PointT>());
        pcl::PointCloud<PointT>::Ptr currSurfDS(new pcl::PointCloud<PointT>());

//        downsampling(currEdge, *currEdgeDS, FeatureType::Edge);
        currEdgeDS = currEdge->makeShared();
        downsampling(currSurf, *currSurfDS, FeatureType::Surf);

        // if is keyframe, append to keyframes vector
        if (is_keyframe_next) {
            current_keyframe = boost::make_shared<Keyframe>(keyframes.size(), currEdgeDS, currSurfDS, currFull->makeShared());
            keyframes.push_back(current_keyframe);
            is_keyframe_next = false;
        }

        if (!is_init) {
            current_keyframe->set_init(odom);
            initOdomFactor();
            is_init = true;
            std::cerr << "Initialization finished " << std::endl;
            return;
        }

        updateSlideWindows();
        // update current odom to world
        getTransToSlideWindow(currEdgeDS, currSurfDS);
        // handle and opti odom
        handleRegistration(currEdgeDS, currSurfDS);
        // pub path
        pubOdomAndPath();
    }

    bool gicp_matching(pcl::PointCloud<PointT>::Ptr cloud_to, pcl::PointCloud<PointT>::Ptr cloud_from, const gtsam::Pose3& pose_guess, gtsam::Pose3& pose) {

        pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
        gicp.setTransformationEpsilon(1e-6);
        gicp.setEuclideanFitnessEpsilon(1e-6);
        gicp.setMaximumIterations(64);
        gicp.setUseReciprocalCorrespondences(false);
        gicp.setCorrespondenceRandomness(20);
        gicp.setMaximumOptimizerIterations(20);

        // Align clouds
        gicp.setInputSource(cloud_from);
        gicp.setInputTarget(cloud_to);

        pcl::PointCloud<PointT>::Ptr unused_result(new pcl::PointCloud<PointT>());
        gicp.align(*unused_result, pose_guess.matrix().cast<float>());

        if (!gicp.hasConverged()) {
            cout << "gicp fail, not converged" << endl;
            return false;
        }
        cout << "fitness score: " << gicp.getFitnessScore() << endl;

        if (gicp.getFitnessScore() > 0.8) {
            return false;
        }
        pose = pose_normalize(gtsam::Pose3(gicp.getFinalTransformation().cast<double>().matrix()));
        return true;
    }


    bool loopMacthing_ICP(pcl::PointCloud<PointT>::Ptr crop, pcl::PointCloud<PointT>::Ptr latest, const gtsam::Pose3& pose_guess, gtsam::Pose3& pose) {

        pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
        gicp.setMaxCorrespondenceDistance(5);
        gicp.setMaximumIterations(100);
        gicp.setTransformationEpsilon(1e-6);
        gicp.setEuclideanFitnessEpsilon(1e-6);

        string filepath = "/home/ziv/mloam/" +std::to_string(save_count++) + "/";
        _mkdir(filepath + "1.txt");
        std::ofstream f(filepath+"init.txt");
        f << pose_guess.matrix();
        f.close();
        pcl::io::savePCDFileASCII(filepath + "crop.pcd", *crop);
        pcl::io::savePCDFileASCII(filepath + "keyframe.pcd", *latest);

        // Align clouds
        gicp.setInputSource(latest);
        gicp.setInputTarget(crop);

        pcl::PointCloud<PointT>::Ptr unused_result(new pcl::PointCloud<PointT>());
        gicp.align(*unused_result, pose_guess.matrix().cast<float>());

        if (!gicp.hasConverged()) {
            cout << "gicp fail, not converged" << endl;
            return false;
        }
        cout << "fitness score: " << gicp.getFitnessScore() << endl;

        if (gicp.getFitnessScore() > 0.8) {
            cout << "gicp fail, fitness score too large" << endl;
            std::ofstream f(filepath+"fail.txt");
            f << gicp.getFitnessScore() << endl;
            f << gicp.getFinalTransformation().cast<double>().matrix();
            f.close();
            return false;
        }

        cout << "icp success" << endl;
        std::ofstream f1(filepath+"success.txt");
        f1 << gicp.getFitnessScore() << endl;
        f1 << gicp.getFinalTransformation().cast<double>().matrix();
        f1.close();
        pose = pose_normalize(gtsam::Pose3(gicp.getFinalTransformation().cast<double>().matrix()));
        return true;
    }

    void loop_detector(const Keyframe::Ptr& latestKeyframe, std::vector<LoopFactor>& loopFactors) {

        int latest_index = latestKeyframe->index;

        if (latest_index < LOOP_LATEST_KEYFRAME_SKIP + 1)
            return;

        if (last_loop_found_index > 0 && latest_index <= last_loop_found_index + LOOP_COOLDOWN_KEYFRAME_COUNT)
            return;

        // <frameCount, distance>
        std::vector<pair<int, int>> candidates;
        candidates.reserve(latest_index - LOOP_LATEST_KEYFRAME_SKIP);
        for (int i = 0; i < latest_index - LOOP_LATEST_KEYFRAME_SKIP; i++) {
            auto pose_between = latestKeyframe->pose.between(keyframes[i]->pose);
            auto distance = pose_between.translation().norm();
            // too far
            if (distance > LOOP_CLOSE_DISTANCE)
                continue;
            candidates.emplace_back(i, distance);
        }

        if (candidates.empty())
            return;

        std::sort(candidates.begin(), candidates.end(), [](const auto& p1, const auto& p2) {
            return p1.second < p2.second;
        });

        auto closestKeyIdx = candidates[0].first;
        int start_crop = max(0, closestKeyIdx - LOOP_KEYFRAME_CROP_LEN);
        int end_crop = min(latest_index - LOOP_LATEST_KEYFRAME_SKIP, closestKeyIdx + LOOP_KEYFRAME_CROP_LEN);

        // crop submap to closestKeyIdx's pose frame
        auto crop = generate_cloud(keyframes, start_crop, end_crop, FeatureType::Full);
        pcl::transformPointCloud(*crop, *crop, keyframes[closestKeyIdx]->pose.inverse().matrix());
        auto latest = generate_cloud(keyframes, latest_index - 1, latest_index + 1, FeatureType::Full);
        pcl::transformPointCloud(*latest, *latest, latestKeyframe->pose.inverse().matrix());

        gtsam::Pose3 pose_crop_latest_opti;

        auto pose_crop_latest_coarse = keyframes[closestKeyIdx]->pose.between(latestKeyframe->pose);
        bool can_match = gicp_matching(crop, latest, pose_crop_latest_coarse, pose_crop_latest_opti);

        if (!can_match)
            return;

        loopFactors.emplace_back(new gtsam::BetweenFactor<gtsam::Pose3>(X(closestKeyIdx), X(latest_index), pose_crop_latest_opti, odometry_gaussian_model));
        cout << "find loop: [" << latest_index << "] and [" << closestKeyIdx << "]\n";
        cout << "pose coarse: \n" << pose_crop_latest_coarse.matrix() << endl;
        cout << "pose after gicp "<< save_count - 1 << ": \n" << pose_crop_latest_opti.matrix() << endl;
        pcl::io::savePCDFileASCII("/home/ziv/mloam/loop/crop.pcd", *crop);
        pcl::io::savePCDFileASCII("/home/ziv/mloam/loop/latest.pcd", *latest);
        last_loop_found_index = latest_index;

    }

    void laser_odometry() {

        cloud_in_edge.reset(new pcl::PointCloud<PointT>());
        cloud_in_surf.reset(new pcl::PointCloud<PointT>());
        cloud_in_full.reset(new pcl::PointCloud<PointT>());

        while (ros::ok()) {

            if (!pointCloudFullBuf.empty() && !pointCloudEdgeBuf.empty() && !pointCloudSurfBuf.empty()) {

                Timer t_laser_odometry("laser_odometry");

                pcd_msg_mtx.lock();

                if (pointCloudFullBuf.front()->header.stamp.toSec() != pointCloudEdgeBuf.front()->header.stamp.toSec() ||
                    pointCloudFullBuf.front()->header.stamp.toSec() != pointCloudSurfBuf.front()->header.stamp.toSec()) {
                    printf("unsync messeage!");
                    ROS_BREAK();
                }

                pcl::fromROSMsg(*pointCloudEdgeBuf.front(), *cloud_in_edge);
                pcl::fromROSMsg(*pointCloudSurfBuf.front(), *cloud_in_surf);
                pcl::fromROSMsg(*pointCloudFullBuf.front(), *cloud_in_full);

                cloud_in_time = pointCloudFullBuf.front()->header.stamp;

                pointCloudEdgeBuf.pop();
                pointCloudSurfBuf.pop();
                pointCloudFullBuf.pop();

                pcd_msg_mtx.unlock();

                update(cloud_in_edge, cloud_in_surf, cloud_in_full);

                t_laser_odometry.count();
            }

            //sleep 2 ms every time
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    void perform_loop_closure() {

        while (ros::ok()) {

            if (!loopDetectBuf.empty()) {

                Timer t_loop("loop detector");

                loop_mtx.lock();
                std::vector<LoopFactor> loopFactors;
                auto latestFrame = loopDetectBuf.front();
                loopDetectBuf.pop();
                loop_mtx.unlock();

                loop_detector(latestFrame, loopFactors);

                for (const auto& factor: loopFactors) {
                    poseGraph.add(factor);
                    copyGraph.add(factor);
                    cout << "add loop factor" << endl;
                }

                if (loopFactors.size() > 0) {
                    loop_found = true;
                    saveOdomGraph();
                }

                loopFactors.clear();
                t_loop.count();
            }

            //sleep 10 ms every time
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }


    void pub_global_map() {

        while (ros::ok()) {

            if (!current_keyframe || !current_keyframe->is_init()) {
                continue;
            }

            int size = keyframes.size();

            if (regenerate_map) {
                mapGenerator.clear();
                mapGenerator.insert(keyframes, 0, size);
                regenerate_map = false;
            } else {
                mapGenerator.insert(keyframes, last_map_generate_index, size);
            }

            auto globalMap = mapGenerator.get();
            if (!globalMap) {
                std::cout << "empty global map!" << std::endl;
                continue;
            }
            sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2());
            pcl::toROSMsg(*globalMap, *cloud_msg);
            cloud_msg->header.stamp = cloud_in_time;
            cloud_msg->header.frame_id = "map";

            pub_map.publish(cloud_msg);

            if (last_map_generate_index == size) {
                save_latency++;
            } else {
                save_latency = 0;
            }
            if (save_latency == 20) {
                pcl::io::savePCDFileASCII("/home/ziv/mloam/global_map_opti.pcd", *globalMap);
                cout << "saved map!" << endl;
            }

            last_map_generate_index = size;
            //sleep 200 ms every time
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }

    }

    void allocateMem() {
        slideWindowEdge    = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
        slideWindowSurf    = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
        kdtreeEdgeSW       = pcl::KdTreeFLANN<PointT>::Ptr(new pcl::KdTreeFLANN<PointT>());
        kdtreeSurfSW       = pcl::KdTreeFLANN<PointT>::Ptr(new pcl::KdTreeFLANN<PointT>());
    }

    void initROSHandler() {
        sub_laser_cloud      = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points_2", 100, &LaserOdometry::pointCloudFullHandler, this);
        sub_laser_cloud_edge = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_edge", 100, &LaserOdometry::pointCloudEdgeHandler, this);
        sub_laser_cloud_surf = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf", 100, &LaserOdometry::pointCloudSurfHandler, this);
        pub_path_odom = nh.advertise<nav_msgs::Path>("/path_odom", 100);
        pub_path_opti = nh.advertise<nav_msgs::Path>("/path_opti", 100);
        pub_map       = nh.advertise<sensor_msgs::PointCloud2>("/global_map", 5);
        pub_curr_edge = nh.advertise<sensor_msgs::PointCloud2>("/curr_edge", 10);
        pub_curr_surf = nh.advertise<sensor_msgs::PointCloud2>("/curr_surf", 10);
        pub_curr_full = nh.advertise<sensor_msgs::PointCloud2>("/curr_full", 10);
        pub_sw_edge   = nh.advertise<sensor_msgs::PointCloud2>("/sw_edge", 10);
        pub_sw_surf   = nh.advertise<sensor_msgs::PointCloud2>("/sw_surf", 10);
    }

    ~LaserOdometry() {

    }

    void initParam() {

        downSizeFilterEdge.setLeafSize(map_resolution, map_resolution, map_resolution);
        downSizeFilterSurf.setLeafSize(map_resolution * 2, map_resolution * 2, map_resolution * 2);
        downSizeFilterNor.setLeafSize(0.1, 0.1, 0.1);

        pose_w_c  = gtsam::Pose3::identity();
        odom      = gtsam::Pose3::identity();
        delta     = gtsam::Pose3::identity();

        edge_gaussian_model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(3) << 0.2, 0.2, 0.2).finished());
        surf_gaussian_model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(1) << 0.2).finished());

        edge_noise_model = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(0.58), edge_gaussian_model);
        surf_noise_model = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(0.02), surf_gaussian_model);

        prior_gaussian_model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished());
        odometry_gaussian_model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-2, 1e-2, 1e-1).finished());

//        prior_noise_model = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.345), prior_gaussian_model);
//        odometry_noise_model = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.345), odometry_gaussian_model);
        prior_noise_model = prior_gaussian_model;
        odometry_noise_model = odometry_gaussian_model;


        gtsam::ISAM2Params isam2Params;
        isam2Params.relinearizeThreshold = 0.1;
        isam2Params.relinearizeSkip = 1;
        isam = std::make_shared<gtsam::ISAM2>(isam2Params);


    }

    LaserOdometry()
    {

        allocateMem();

        initROSHandler();

        initParam();
    }



private:

    ros::NodeHandle nh;
    ros::Time cloud_in_time;

    MapGenerator mapGenerator;

    std::vector<Keyframe::Ptr> keyframes;
    Keyframe::Ptr current_keyframe;

    gtsam::Pose3 odom;      // world to current frame
    gtsam::Pose3 last_odom; // world to last frame
    gtsam::Pose3 delta;     // last frame to current frame
    gtsam::Pose3 pose_w_c;  // to be optimized

    // queue of ros pointcloud msg
    std::mutex pcd_msg_mtx;
    std::mutex gps_msg_mtx;
    std::mutex loop_mtx;
    std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudEdgeBuf;
    std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudSurfBuf;
    std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudFullBuf;
    std::queue<geographic_msgs::GeoPointStampedConstPtr> gps_queue;
    std::queue<Keyframe::Ptr> loopDetectBuf;

    pcl::PointCloud<PointT>::Ptr cloud_in_edge;
    pcl::PointCloud<PointT>::Ptr cloud_in_surf;
    pcl::PointCloud<PointT>::Ptr cloud_in_full;

    // frame-to-slidewindow
    pcl::PointCloud<PointT>::Ptr slideWindowEdge;
    pcl::PointCloud<PointT>::Ptr slideWindowSurf;

    pcl::KdTreeFLANN<PointT>::Ptr kdtreeEdgeSW;
    pcl::KdTreeFLANN<PointT>::Ptr kdtreeSurfSW;

    // downsample filter
    pcl::VoxelGrid<PointT> downSizeFilterEdge;
    pcl::VoxelGrid<PointT> downSizeFilterSurf;
    pcl::VoxelGrid<PointT> downSizeFilterNor;

    ros::Subscriber sub_laser_cloud, sub_laser_cloud_edge, sub_laser_cloud_surf;
    ros::Publisher pub_path_odom, pub_path_opti, pub_map;
    ros::Publisher pub_curr_edge, pub_curr_surf, pub_curr_full;
    ros::Publisher pub_sw_edge, pub_sw_surf;
    nav_msgs::Path path_odom, path_opti;


    const int SLIDE_KEYFRAME_LEN = 6;
    const int LOOP_KEYFRAME_CROP_LEN = 10;
    const int LOOP_LATEST_KEYFRAME_SKIP = 50;
    const int LOOP_COOLDOWN_KEYFRAME_COUNT = 20;
    const int LOOP_CLOSE_DISTANCE = 15;

    const double keyframeDistThreshold = 0.6;
    const double keyframeAngleThreshold = 0.1;

    bool is_init = false;
    bool is_keyframe_next = true;
    double map_resolution = 0.2;

    int last_loop_found_index = 0;
    bool loop_found = false;
    bool status_change = false;
    bool regenerate_map = false;
    int save_latency = 0;
    int last_map_generate_index = 0;

    // gtsam
    gtsam::NonlinearFactorGraph poseGraph;
    gtsam::NonlinearFactorGraph copyGraph;
    gtsam::Values initEstimate;
    gtsam::Values copyEstimate;
    std::shared_ptr<gtsam::ISAM2> isam;
    gtsam::Values isamOptimize;

    // gaussian model
    gtsam::SharedNoiseModel edge_gaussian_model, surf_gaussian_model, prior_gaussian_model, odometry_gaussian_model;
    // noise model
    gtsam::SharedNoiseModel edge_noise_model, surf_noise_model, prior_noise_model, odometry_noise_model;


};

int main(int argc, char **argv) {

    ros::init(argc, argv, "laserOdometry");

    LaserOdometry laserOdometry;

    std::thread laser_odometry_thread{&LaserOdometry::laser_odometry, &laserOdometry};

//    std::thread gps_ground_truth_thread{&LaserOdometry::gps_ground_truth, &laserOdometry};

    std::thread loop_detector_thread{&LaserOdometry::perform_loop_closure, &laserOdometry};

    std::thread global_map_publisher{&LaserOdometry::pub_global_map, &laserOdometry};

    ros::spin();

}