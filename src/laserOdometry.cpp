//
// Created by ziv on 2020/10/26.
//

#include "helper.h"
#include "keyframe.h"
#include "utils.h"
#include "map_generator.h"
#include "factors.h"
#include "loop_detector.h"

#include <pcl/segmentation/region_growing.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>

static int plane_id = 0;

class LaserOdometry {
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
        // update only when new keyframe goes in with correct pose
        if (current_keyframe->valid_frames == 1) {
            size_t size = keyframeVec->keyframes.size();
            size_t start = size < SLIDE_KEYFRAME_LEN ? 0 : size - SLIDE_KEYFRAME_LEN;
            size_t start2 = size < SLIDE_PLANE_LEN ? 0 : size - SLIDE_PLANE_LEN;

            slideWindowEdge = MapGenerator::generate_cloud(keyframeVec, start, size, FeatureType::Edge);
            slideWindowSurf = MapGenerator::generate_cloud(keyframeVec, start, size, FeatureType::Surf);
            slideWindowPlane = MapGenerator::generate_cloud(keyframeVec, start2, size, FeatureType::Plane);
            downsampling(slideWindowEdge, *slideWindowEdge, FeatureType::Edge);
            downsampling(slideWindowSurf, *slideWindowSurf, FeatureType::Surf);
            downsampling(slideWindowPlane, *slideWindowPlane, FeatureType::Plane);
        }
    }

    bool nextFrameToBeKeyframe() {
        Eigen::Isometry3d T_delta((current_keyframe->pose_world_curr.inverse() * odom).matrix());
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

        // read lock
        std::vector<gtsam::Pose3> poseVec = keyframeVec->read_poses(0, keyframeVec->keyframes.size());

        for (size_t i = 0; i < poseVec.size(); i++) {
            laserPose.header = odometry_opti.header;
            Eigen::Quaterniond q(poseVec[i].rotation().matrix());
            laserPose.pose.orientation.x    = q.x();
            laserPose.pose.orientation.y    = q.y();
            laserPose.pose.orientation.z    = q.z();
            laserPose.pose.orientation.w    = q.w();
            laserPose.pose.position.x       = poseVec[i].translation().x();
            laserPose.pose.position.y       = poseVec[i].translation().y();
            laserPose.pose.position.z       = poseVec[i].translation().z();
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
        pcl::toROSMsg(*cloud_in_edge, *edge_msg);
        pcl::toROSMsg(*cloud_in_surf, *surf_msg);
        edge_msg->header.stamp = cloud_in_time;
        edge_msg->header.frame_id = "base_link";
        surf_msg->header.stamp = cloud_in_time;
        surf_msg->header.frame_id = "base_link";
        pub_curr_edge.publish(edge_msg);
        pub_curr_surf.publish(surf_msg);

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
                             norm(2) * map_in->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.12) {
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

    int addSurfCostFactor2(
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
                            X(state_key), curr_point, norm, negative_OA_dot_norm, surf_noise_model2);
                    surf_num++;
                }
            }

        }

        if (surf_num < 20) {
            printf("not enough correct points\n");
        }

        return surf_num;

    }

//    void addPlaneToPlaneFactor(Keyframe::Ptr fromFrame, Keyframe::Ptr toFrame) {
//
//        if (!fromFrame->is_init() || !toFrame->is_init()) {
//            std::cerr << "from or to frame haven't init!" << std::endl;
//            return;
//        }
//
//        // read lock
//        gtsam::Pose3 from_pose, to_pose;
//        {
//            std::shared_lock<std::shared_mutex> sl(keyframeVec->pose_mtx);
//            from_pose = fromFrame->pose_world_curr;
//            to_pose = toFrame->pose_world_curr;
//        }
//        int correspondence = 0;
//
//        std::vector<gtsam::PlaneToPlaneFactor::shared_ptr> planePlaneFactors;
//
//        for (size_t i = 0; i < fromFrame->planes.size(); i++) {
//
//            int best_j = -1;
//            double distanceThreshold = map_resolution * 0.75;
//            double angleThreshold = 2.0;
//            auto this_plane = fromFrame->planes[i].transform(from_pose);
//            Eigen::Vector3d n1(this_plane.normal().unitVector());
//
//            for (size_t j = 0; j < toFrame->planes.size(); j++) {
//                auto other_plane = toFrame->planes[j].transform(to_pose);
//                Eigen::Vector3d n2(other_plane.normal().unitVector());
//                double angle = std::atan2(n1.cross(n2).norm(), n1.transpose() * n2) * 180 / M_PI;
//                double distance = std::abs(this_plane.distance() - other_plane.distance());
//
//                if (distance < distanceThreshold && (angle > -angleThreshold && angle < angleThreshold ||
//                                                     angle > 180 - angleThreshold &&
//                                                     angle < 180 + angleThreshold)) {
//                    distanceThreshold = distance;
//                    angleThreshold = angle;
//                    best_j = j;
//                }
//            }
//
//            if (best_j != -1) {
//                std::cout << "find closest plane!" << std::endl;
//                std::cout << "current: " << n1.transpose() << " " << this_plane.distance() << std::endl;
//                auto other_plane = toFrame->planes[best_j].transform(to_pose);
//                Eigen::Vector3d n2(other_plane.normal().unitVector());
//                std::cout << "closest: " << n2.transpose() << " " << other_plane.distance() << std::endl;
//                planePlaneFactors.emplace_back(boost::make_shared<gtsam::PlaneToPlaneFactor>(X(fromFrame->index), fromFrame->planes[i],
//                                                                                             X(toFrame->index), toFrame->planes[best_j],
//                                                                                             plane_plane_noise_model));
//                correspondence++;
//
//            }
//
//        }
//
//        if (correspondence >= 8) {
//            for (const auto& factor : planePlaneFactors) {
//                BAGraph.add(factor);
//            }
//            std::cerr << "add plane plane factor!" << std::endl;
//        } else {
//            std::cerr << "not enough plane plane factor!" << std::endl;
//        }
//
//    }

    void getTransToSlideWindow(pcl::PointCloud<PointT>::Ptr currEdgeDS, pcl::PointCloud<PointT>::Ptr currSurfDS, pcl::PointCloud<PointT>::Ptr currPlane) {

        gtsam::Pose3 odom_prediction = pose_normalize(odom * delta);

        last_odom = odom;
        odom = odom_prediction;

        // 'pose_w_c' to be optimize
        pose_w_c = odom;

        const auto state_key = X(0);

        if (slideWindowEdge->size() > 10 && slideWindowSurf->size() > 50) {

            kdtreeEdgeSW->setInputCloud(slideWindowEdge);
            kdtreeSurfSW->setInputCloud(slideWindowSurf);
            kdtreePlaneSW->setInputCloud(slideWindowPlane);

            for (int opti_counter = 0; opti_counter < 3; opti_counter++) {

                gtsam::NonlinearFactorGraph factors;
                gtsam::Values init_values;
                init_values.insert(state_key, pose_w_c);

                addEdgeCostFactor(currEdgeDS, slideWindowEdge, kdtreeEdgeSW, pose_w_c, factors, 0);
                addSurfCostFactor(currSurfDS, slideWindowSurf, kdtreeSurfSW, pose_w_c, factors, 0);
//                addSurfCostFactor2(currPlane, slideWindowSurf, kdtreeSurfSW, pose_w_c, factors, 0);

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
        delta = pose_normalize(last_odom.inverse() * odom);
    }

    void initOdomFactor() {
        BAGraph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0), gtsam::Pose3(), prior_noise_model);
        BAEstimate.insert(X(0), gtsam::Pose3());
    }

    void addOdomFactor() {
        int index = current_keyframe->index;
        // read lock
        gtsam::Pose3 last_pose = keyframeVec->read_poses(index- 1, index).front();

        BAGraph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(index - 1), X(index),
                                                                   last_pose.between(current_keyframe->pose_world_curr),
                                                                   odometry_noise_model);
        BAEstimate.insert(X(index), current_keyframe->pose_world_curr);
    }

    void handleRegistration() {

        if (!current_keyframe->is_init()) {
            Keyframe::Ptr last_keyframe = keyframeVec->keyframes[current_keyframe->index - 1];
            // push to loopDetectBuf
            BA_mtx.lock();
            BAKeyframeBuf.push(last_keyframe);
            BA_mtx.unlock();

            /*** all  pose: [0, current]
             *** opti pose: [0, k-1]
             *** not  opti: [k, current] */

            // update raw odom of KF_current and relative pose to KF_last
            current_keyframe->set_init(last_keyframe, odom);

            // update pose from 0 to current
            updatePoses();

            addOdomFactor();

            odom = current_keyframe->pose_world_curr;

        } else {

            is_keyframe_next = nextFrameToBeKeyframe();

            current_keyframe->add_frame();

        }
    }

    void updatePoses() {
        // write lock
        std::unique_lock<std::shared_mutex> ul(keyframeVec->pose_mtx);

        graph_mtx.lock();
        isam->update(BAGraph, BAEstimate);
        BAGraph.resize(0);
        BAEstimate.clear();
        isam->update();
        graph_mtx.unlock();

        isamOptimize = isam->calculateEstimate();

        for (size_t i = 0; i < isamOptimize.size(); i++) {
            keyframeVec->keyframes[i]->pose_world_curr = pose_normalize(isamOptimize.at<gtsam::Pose3>(X(i)));
        }
    }

    void init_extract_plane_params() {
        searchKdtree = pcl::search::KdTree<PointT>::Ptr(new pcl::search::KdTree<PointT>());
        ne.setSearchMethod(searchKdtree);
        ne.setKSearch(20);
        regionGrowing.setMinClusterSize(20);
        regionGrowing.setMaxClusterSize(50000);
        regionGrowing.setSearchMethod(searchKdtree);
        regionGrowing.setResidualThreshold(map_resolution);
        regionGrowing.setSmoothnessThreshold(3 / 180.0 * M_PI);
        regionGrowing.setCurvatureThreshold(0.1);
    }

    pcl::PointCloud<PointT>::Ptr extract_planes(pcl::PointCloud<PointT>::Ptr currSurf) {
        TimeCounter t1;
        pcl::PointCloud<NormalT>::Ptr cloud_normals(new pcl::PointCloud<NormalT>());
        std::vector<pcl::PointIndices> clusters;
        ne.setInputCloud(currSurf);
        ne.compute(*cloud_normals);
        regionGrowing.setInputCloud(currSurf);
        regionGrowing.setInputNormals(cloud_normals);
        regionGrowing.extract(clusters);

        pcl::PointCloud<PointT>::Ptr planes(new pcl::PointCloud<PointT>());

        for (const auto& cluster : clusters) {
            size_t size = cluster.indices.size();
            Eigen::MatrixXd matA0(size, 3), matB0(size, 1);
            matB0.setOnes();
            matB0 = -1 * matB0;

            for (size_t j = 0; j < size; j++) {
                int idx = cluster.indices[j];
                matA0(j, 0) = currSurf->points[idx].x;
                matA0(j, 1) = currSurf->points[idx].y;
                matA0(j, 2) = currSurf->points[idx].z;
            }
            Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
            double d = 1 / norm.norm();
            norm.normalize();

            bool planeValid = true;
            for (size_t j = 0; j < size; j++) {
                int idx = cluster.indices[j];
                if (fabs(norm(0) * currSurf->points[idx].x +
                         norm(1) * currSurf->points[idx].y +
                         norm(2) * currSurf->points[idx].z + d) > 0.25) {
                    planeValid = false;
                    break;
                }
            }

            if (planeValid) {
                for (size_t j = 0; j < size; j++) {
//                    current_keyframe->planes->push_back(current_keyframe->surfFeatures->points[cluster.indices[j]]);
                    planes->push_back(currSurf->points[cluster.indices[j]]);
                }
            }
        }
        return planes;
//        std::cout << "extract plane: " << current_keyframe->planes.size() << " within " << t1.count() << "ms" << std::endl;
    }

    void update_keyframe(pcl::PointCloud<PointT>::Ptr currEdge, pcl::PointCloud<PointT>::Ptr currSurf, pcl::PointCloud<PointT>::Ptr currFull) {
        current_keyframe = boost::make_shared<Keyframe>(keyframeVec->keyframes.size(), currEdge, currSurf, currFull->makeShared());
        keyframeVec->keyframes.emplace_back(current_keyframe);
        is_keyframe_next = false;
    }

    void update(pcl::PointCloud<PointT>::Ptr currEdge, pcl::PointCloud<PointT>::Ptr currSurf, pcl::PointCloud<PointT>::Ptr currFull) {

        pcl::PointCloud<PointT>::Ptr currEdgeDS(new pcl::PointCloud<PointT>());
        pcl::PointCloud<PointT>::Ptr currSurfDS(new pcl::PointCloud<PointT>());

        currEdgeDS = currEdge->makeShared();
        downsampling(currSurf, *currSurfDS, FeatureType::Surf);

        auto currPlanes = extract_planes(currSurfDS);

        // if is keyframe, append to keyframes vector
        if (is_keyframe_next) {
            update_keyframe(currEdgeDS, currSurfDS, currFull);
            current_keyframe->planes = currPlanes;
        }

        if (!is_init) {
            current_keyframe->set_init(current_keyframe, gtsam::Pose3());
            initOdomFactor();
            is_init = true;
            return;
        }

        updateSlideWindows();
        // update current odom to world
        getTransToSlideWindow(currEdgeDS, currSurfDS, currPlanes);
        // handle and opti odom
        handleRegistration();
        // pub path
        pubOdomAndPath();

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

//    pcl::PointCloud<PointT>::Ptr planeCollector(Keyframe::Ptr current_keyframe) {
//
//        pcl::PointCloud<PointT>::Ptr plane_sliding_windows(new pcl::PointCloud<PointT>());
//        const int plane_track_sliding_window_len = 6;
//
//        int curr_idx = current_keyframe->index;
//        int start = max(0, curr_idx - plane_track_sliding_window_len);
//
//        std::vector<gtsam::Pose3> poseVec = keyframeVec->read_poses(start, curr_idx);
//
//        std::cout << "start: " << start << " curr_idx: " << curr_idx << std::endl;
//
//        for (int i = start; i < curr_idx; i++) {
//            for (const auto& plane : keyframeVec->keyframes[i]->planes) {
//                auto temp_plane = plane->points->makeShared();
//                pcl::transformPointCloud(*temp_plane, *temp_plane, poseVec[i - start].matrix());
//                *plane_sliding_windows += *temp_plane;
//            }
//        }
//
//        std::cout << "plane_sliding_windows size" << plane_sliding_windows->size() << std::endl;
//        return plane_sliding_windows;
//    }
//
//    void initPlaneCollector(Keyframe::Ptr current_keyframe) {
//        for (const auto& plane : current_keyframe->planes) {
//            for (auto& point : plane->points->points) {
//                point.intensity = plane_id;
//            }
////            plane2HostMap[plane_id] = current_keyframe->index;
//            plane_id++;
//        }
//    }

//    void addPointToPlaneFactor(Keyframe::Ptr current_keyframe) {
//
//        initPlaneCollector(current_keyframe);
//        if (current_keyframe->index == 0)
//            return;
//        pcl::PointCloud<PointT>::Ptr plane_sliding_windows = planeCollector(current_keyframe);
//
//        std::cout << "plane_sliding_windows empty ? " << plane_sliding_windows->empty() << std::endl;
//
//        pcl::KdTreeFLANN<PointT> kdtree;
//        kdtree.setInputCloud(plane_sliding_windows);
//
//        PointT point_temp;
//        std::vector<int> pointSearchInd;
//        std::vector<float> pointSearchSqDis;
//
//        gtsam::Pose3 pose_w_c;
//        {
//            keyframeVec->pose_mtx.lock_shared();
//            pose_w_c = current_keyframe->pose_world_curr;
//            keyframeVec->pose_mtx.unlock_shared();
//        }
//
//        gtsam::NonlinearFactorGraph factors;
//        gtsam::Values init_values;
//        init_values.insert(X(current_keyframe->index), pose_w_c);
//
//        for (const auto& plane : current_keyframe->planes) {
//
//            size_t total_points = plane->points->size();
//            unordered_map<int, int> m;
//
//            for (const auto& point : plane->points->points) {
//                pointAssociate(&point, &point_temp, pose_w_c);
//                kdtree.nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis);
//
//                Eigen::Matrix<double, 5, 3> matA0;
//                Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
//
//                if (pointSearchSqDis[4] < 0.4) {
//
//                    bool samePlane = true;
//                    for (int j = 1; j < 5; j++) {
//                        if (plane_sliding_windows->points[pointSearchInd[j]].intensity != plane_sliding_windows->points[pointSearchInd[j - 1]].intensity) {
//                            samePlane = false;
//                            break;
//                        }
//                    }
//                    if (!samePlane) {
//                        continue;
//                    }
//                    for (int j = 0; j < 5; j++) {
//                        matA0(j, 0) = plane_sliding_windows->points[pointSearchInd[j]].x;
//                        matA0(j, 1) = plane_sliding_windows->points[pointSearchInd[j]].y;
//                        matA0(j, 2) = plane_sliding_windows->points[pointSearchInd[j]].z;
//                    }
//                    // find the norm of plane
//                    Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
//                    double d = 1 / norm.norm();
//                    norm.normalize();
//
//                    bool planeValid = true;
//                    for (int j = 0; j < 5; j++) {
//                        // if OX * n > 0.2, then plane is not fit well
//                        if (fabs(norm(0) * plane_sliding_windows->points[pointSearchInd[j]].x +
//                                 norm(1) * plane_sliding_windows->points[pointSearchInd[j]].y +
//                                 norm(2) * plane_sliding_windows->points[pointSearchInd[j]].z + d) > 0.2) {
//                            planeValid = false;
//                            break;
//                        }
//                    }
//                    Eigen::Vector3d curr_point(point.x, point.y, point.z);
//                    if (planeValid) {
//                        int plane_id = int(plane_sliding_windows->points[pointSearchInd[0]].intensity);
//                        factors.emplace_shared<gtsam::PointToPlaneFactor>(
//                                X(current_keyframe->index), curr_point, norm, d, surf_noise_model);
//                        m[plane_id]++;
//                    }
//
//                }
//            }
//            for (const auto& p : m) {
//                if (p.second >= total_points / 2) {
//                    std::cout << "same plane!" << std::endl;
//                    for (auto& point : plane->points->points) {
//                        point.intensity = p.first;
//                    }
//                }
//            }
//        }
//
//        // gtsam
//        gtsam::LevenbergMarquardtParams params;
//        params.setLinearSolverType("MULTIFRONTAL_CHOLESKY");
//        params.setRelativeErrorTol(1e-4);
//        params.maxIterations = 6;
//
//        auto result = gtsam::LevenbergMarquardtOptimizer(factors, init_values, params).optimize();
//
//        pose_w_c = result.at<gtsam::Pose3>(X(current_keyframe->index));
//
//        BAGraph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(0), X(current_keyframe->index), pose_w_c, odometry_noise_model);
//    }

//    void updateFactorGraph() {
//
//        std::lock_guard<std::Mmutex> lockGuard(graph_mtx);
//
//
//
//    }

    void BA_optimization() {

        while (ros::ok()) {

            std::vector<Keyframe::Ptr> keyframes_buf;
            BA_mtx.lock();
            keyframes_buf.reserve(BAKeyframeBuf.size());
            while (!BAKeyframeBuf.empty()) {
                auto keyframe = BAKeyframeBuf.front();
                BAKeyframeBuf.pop();
                keyframes_buf.push_back(keyframe);
            }
            BA_mtx.unlock();

            if (keyframes_buf.size() > 0) {
                // todo use openmp
//                for (const auto& this_keyframe : keyframes_buf) {
//                    addPointToPlaneFactor(this_keyframe);
//                }
                std::vector<FactorPtr> factors;
                for (const auto& keyframe : keyframes_buf) {
//                    loopDetector.submap_finetune(keyframeVec, keyframe, factors);
                    loopDetector.loop_detector(keyframeVec, keyframe, factors);
                }

                // todo use openmp
                for (const auto& factor: factors) {
                    BAGraph.add(factor);
                }

                status_change = (factors.size() > 0);
            }

            // isam update
            graph_mtx.lock();
            isam->update(BAGraph, BAEstimate);
            BAGraph.resize(0);
            BAEstimate.clear();
            isam->update();

            if (status_change) {
                isam->update();
                isam->update();
                isam->update();
                status_change = false;
            }
            graph_mtx.unlock();

            regenerate_map = true;

            //sleep 10 ms every time
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

    }

    void pub_global_map() {

        while (ros::ok()) {

            if (!current_keyframe || !current_keyframe->is_init()) {
                continue;
            }

            size_t size = keyframeVec->keyframes.size();

            if (regenerate_map) {
                mapGenerator.clear();
                mapGenerator.insert(keyframeVec, 0, size);
                regenerate_map = false;
            } else {
                mapGenerator.insert(keyframeVec, last_map_generate_index, size);
            }

            auto globalMap = mapGenerator.get(0.0f);
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
        slideWindowPlane    = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
        kdtreeEdgeSW       = pcl::KdTreeFLANN<PointT>::Ptr(new pcl::KdTreeFLANN<PointT>());
        kdtreeSurfSW       = pcl::KdTreeFLANN<PointT>::Ptr(new pcl::KdTreeFLANN<PointT>());
        kdtreePlaneSW       = pcl::KdTreeFLANN<PointT>::Ptr(new pcl::KdTreeFLANN<PointT>());
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
        pub_sw_edge   = nh.advertise<sensor_msgs::PointCloud2>("/sw_edge", 10);
        pub_sw_surf   = nh.advertise<sensor_msgs::PointCloud2>("/sw_surf", 10);
    }

    ~LaserOdometry() {

    }

    void initParam() {

        ne.setNumberOfThreads(std::thread::hardware_concurrency());

        keyframeVec = boost::make_shared<KeyframeVec>();
        keyframeVec->keyframes.reserve(200);

        downSizeFilterEdge.setLeafSize(map_resolution / 2, map_resolution / 2, map_resolution /2 );
        downSizeFilterSurf.setLeafSize(map_resolution, map_resolution, map_resolution);
        downSizeFilterNor.setLeafSize(0.1, 0.1, 0.1);

        pose_w_c  = gtsam::Pose3::identity();
        odom      = gtsam::Pose3::identity();
        delta     = gtsam::Pose3::identity();

        edge_gaussian_model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(3) << 0.2, 0.2, 0.2).finished());
        surf_gaussian_model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(1) << 0.2).finished());

        edge_noise_model = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(0.58), edge_gaussian_model);
        surf_noise_model = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(0.02), surf_gaussian_model);
        surf_noise_model2 = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(0.001), surf_gaussian_model);

        prior_gaussian_model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished());
        odometry_gaussian_model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-2, 1e-2, 1e-1).finished());

        prior_noise_model = prior_gaussian_model;
        odometry_noise_model = odometry_gaussian_model;

        gtsam::ISAM2Params isam2Params;
        isam2Params.relinearizeThreshold = 0.1;
        isam2Params.relinearizeSkip = 1;
        isam = std::make_shared<gtsam::ISAM2>(isam2Params);

        init_extract_plane_params();

    }

    LaserOdometry()
    {

        allocateMem();

        initROSHandler();

        initParam();

    }



private:

    /*********************************************************************
   ** Global Variables
   *********************************************************************/
    ros::NodeHandle nh;
    ros::Time cloud_in_time;
    KeyframeVec::Ptr keyframeVec;

    MapGenerator mapGenerator;
    LoopDetector loopDetector;

    // ros msg
    ros::Subscriber sub_laser_cloud, sub_laser_cloud_edge, sub_laser_cloud_surf;
    ros::Publisher pub_path_odom, pub_path_opti, pub_map;
    ros::Publisher pub_curr_edge, pub_curr_surf;
    ros::Publisher pub_sw_edge, pub_sw_surf;
    nav_msgs::Path path_odom, path_opti;

    const double keyframeDistThreshold = 0.6;
    const double keyframeAngleThreshold = 0.1;
    bool is_keyframe_next = true;

    double map_resolution = 0.4;

    // status
    bool regenerate_map = false;

    // gtsam
    std::mutex graph_mtx;
    gtsam::NonlinearFactorGraph BAGraph;
    gtsam::Values BAEstimate;
    std::shared_ptr<gtsam::ISAM2> isam;
    gtsam::Values isamOptimize;

    std::mutex poses_opti_mtx;
    std::vector<gtsam::Pose3> poses_optimized;

    // gaussian model
    gtsam::SharedNoiseModel edge_gaussian_model, surf_gaussian_model, prior_gaussian_model, odometry_gaussian_model;
    // noise model
    gtsam::SharedNoiseModel edge_noise_model, surf_noise_model, prior_noise_model, odometry_noise_model, plane_plane_noise_model, surf_noise_model2;

    /*********************************************************************
   ** Laser Odometry
   *********************************************************************/
    Keyframe::Ptr current_keyframe;
//    std::vector<Keyframe::Ptr> keyframes;
    gtsam::Pose3 odom;      // world to current frame
    gtsam::Pose3 last_odom; // world to last frame
    gtsam::Pose3 delta;     // last frame to current frame
    gtsam::Pose3 pose_w_c;  // to be optimized

    bool is_init = false;

    // frame-to-slidewindow
    const int SLIDE_KEYFRAME_LEN = 6;
    const int SLIDE_PLANE_LEN = 10;
    pcl::PointCloud<PointT>::Ptr slideWindowEdge;
    pcl::PointCloud<PointT>::Ptr slideWindowSurf;
    pcl::PointCloud<PointT>::Ptr slideWindowPlane;
    pcl::KdTreeFLANN<PointT>::Ptr kdtreeEdgeSW;
    pcl::KdTreeFLANN<PointT>::Ptr kdtreeSurfSW;
    pcl::KdTreeFLANN<PointT>::Ptr kdtreePlaneSW;

    // downsample filter
    pcl::VoxelGrid<PointT> downSizeFilterEdge;
    pcl::VoxelGrid<PointT> downSizeFilterSurf;
    pcl::VoxelGrid<PointT> downSizeFilterNor;

    // point cloud from laserProcessing
    pcl::PointCloud<PointT>::Ptr cloud_in_edge;
    pcl::PointCloud<PointT>::Ptr cloud_in_surf;
    pcl::PointCloud<PointT>::Ptr cloud_in_full;

    // queue of ros pointcloud msg
    std::mutex pcd_msg_mtx;
    std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudEdgeBuf;
    std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudSurfBuf;
    std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudFullBuf;

    // plane segment
    pcl::NormalEstimationOMP<PointT, NormalT> ne;
    pcl::search::KdTree<PointT>::Ptr searchKdtree;
    pcl::RegionGrowing<PointT, NormalT> regionGrowing;

    /*********************************************************************
   ** backend BA
   *********************************************************************/
    std::mutex BA_mtx;
    std::queue<Keyframe::Ptr> BAKeyframeBuf;
    bool status_change = false;

    /*********************************************************************
   ** Map Generator
   *********************************************************************/
    int save_latency = 0;
    int last_map_generate_index = 0;

};

int main(int argc, char **argv) {

    ros::init(argc, argv, "laserOdometry");

    LaserOdometry laserOdometry;

    std::thread laser_odometry_thread{&LaserOdometry::laser_odometry, &laserOdometry};

    std::thread backend_ba_optimization{&LaserOdometry::BA_optimization, &laserOdometry};

    std::thread global_map_publisher{&LaserOdometry::pub_global_map, &laserOdometry};

    ros::spin();

}