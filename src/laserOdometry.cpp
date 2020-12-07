//
// Created by ziv on 2020/10/26.
//

//#include "helper.h"
//#include "frame.h"
#include "map_generator.h"
#include "factors.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/crop_box.h>
#include <pcl/registration/gicp.h>

static int correct_count = 1;
static int save_count = 1;
static int graph_count = 1;

class LaserOdometry {

public:
    using LoopFactor = gtsam::NonlinearFactor::shared_ptr;

    enum class FeatureType {
        Edge = 0,
        Surf = 1
    };

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

    void pointAssociateToMap(PointT const *const pi, PointT *const po) {
        Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
        Eigen::Vector3d point_w = odom_submap.rotation() * point_curr + odom_submap.translation();
        po->x = point_w.x();
        po->y = point_w.y();
        po->z = point_w.z();
        po->intensity = pi->intensity;
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
            cout << "invalid downsample type" << endl;
            return;
        }
    }

    void addToCurrSlice(pcl::PointCloud<PointT>::Ptr currEdge, pcl::PointCloud<PointT>::Ptr currSurf, const gtsam::Pose3& odom) {

        if (isCurrKeyframeSliceEmpty()) {
            *current_keyframe->edgeSlice += *currEdge;
            *current_keyframe->surfSlice += *currSurf;
            current_keyframe->frameCount++;
            return;
        }

        pcl::PointCloud<PointT>::Ptr transformedEdge(new pcl::PointCloud<PointT>());
        pcl::PointCloud<PointT>::Ptr transformedSurf(new pcl::PointCloud<PointT>());
        // all frame in same slice transform to keyframe's pose
        Eigen::Isometry3d pose2currKey((current_keyframe->pose.inverse() * odom).matrix());

        pcl::transformPointCloud(*currEdge, *transformedEdge, pose2currKey.matrix());
        pcl::transformPointCloud(*currSurf, *transformedSurf, pose2currKey.matrix());
        *current_keyframe->edgeSlice += *transformedEdge;
        *current_keyframe->surfSlice += *transformedSurf;
        current_keyframe->frameCount++;

        downsampling(current_keyframe->edgeSlice, *current_keyframe->edgeSlice, FeatureType::Edge);
        downsampling(current_keyframe->surfSlice, *current_keyframe->surfSlice, FeatureType::Surf);

    }

    void addToSubMap(pcl::PointCloud<PointT>::Ptr currEdge, pcl::PointCloud<PointT>::Ptr currSurf) {

        PointT point_temp;

        for (size_t i = 0; i < currEdge->size(); i++) {
            pointAssociateToMap(&currEdge->points[i], &point_temp);
            pointCloudEdgeMap->push_back(point_temp);
        }

        for (size_t i = 0; i < currSurf->size(); i++) {
            pointAssociateToMap(&currSurf->points[i], &point_temp);
            pointCloudSurfMap->push_back(point_temp);
        }

        double x_min = +odom_submap.translation().x()-50;
        double y_min = +odom_submap.translation().y()-50;
        double z_min = +odom_submap.translation().z()-50;
        double x_max = +odom_submap.translation().x()+50;
        double y_max = +odom_submap.translation().y()+50;
        double z_max = +odom_submap.translation().z()+50;

        //ROS_INFO("size : %f,%f,%f,%f,%f,%f", x_min, y_min, z_min,x_max, y_max, z_max);
        cropBoxFilter.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0));
        cropBoxFilter.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0));
        cropBoxFilter.setNegative(false);

        pcl::PointCloud<PointT>::Ptr tmpEdge(new pcl::PointCloud<PointT>());
        pcl::PointCloud<PointT>::Ptr tmpSurf(new pcl::PointCloud<PointT>());
        cropBoxFilter.setInputCloud(pointCloudSurfMap);
        cropBoxFilter.filter(*tmpSurf);
        cropBoxFilter.setInputCloud(pointCloudEdgeMap);
        cropBoxFilter.filter(*tmpEdge);

        downSizeFilterEdge.setInputCloud(tmpSurf);
        downSizeFilterEdge.filter(*pointCloudSurfMap);
        downSizeFilterSurf.setInputCloud(tmpEdge);
        downSizeFilterSurf.filter(*pointCloudEdgeMap);
    }

    void addToGlobalMapOdom(pcl::PointCloud<PointT>::Ptr currEdge, pcl::PointCloud<PointT>::Ptr currSurf) {

        PointT point_temp;

        for (size_t i = 0; i < currEdge->size(); i++) {
            pointAssociate(&currEdge->points[i], &point_temp, odom);
            globalMap_odom->push_back(point_temp);
        }

        for (size_t i = 0; i < currSurf->size(); i++) {
            pointAssociate(&currSurf->points[i], &point_temp, odom);
            globalMap_odom->push_back(point_temp);
        }

//        downSizeFilterEdge.setInputCloud(globalMap_odom);
//        downSizeFilterEdge.filter(*globalMap_odom);
    }

    void updateSlideWindows() {

        slideWindowEdge->clear();
        slideWindowSurf->clear();

        pcl::PointCloud<PointT>::Ptr tempEdge(new pcl::PointCloud<PointT>());
        pcl::PointCloud<PointT>::Ptr tempSurf(new pcl::PointCloud<PointT>());

        int size = keyframes.size();
        // not add latest keyframe, cuz its slice is empty, haven't registered
        if (isCurrKeyframeSliceEmpty()) {
            size--;
        }

        int start = size < SLIDE_KEYFRAME_SUBMAP_LEN ? 0 : size - SLIDE_KEYFRAME_SUBMAP_LEN;

        for (int count = start; count < size; count++) {
            pcl::transformPointCloud(*keyframes[count]->edgeSlice, *tempEdge, keyframes[count]->pose.matrix());
            pcl::transformPointCloud(*keyframes[count]->surfSlice, *tempSurf, keyframes[count]->pose.matrix());
            *slideWindowEdge += *tempEdge;
            *slideWindowSurf += *tempSurf;
        }

    }

    bool isCurrKeyframeSliceEmpty() {
        return current_keyframe->frameCount == 0;
    }

    void initWithFirstFrame(pcl::PointCloud<PointT>::Ptr currEdge, pcl::PointCloud<PointT>::Ptr currSurf) {

        // init local submap
        *pointCloudEdgeMap += *currEdge;
        *pointCloudSurfMap += *currSurf;

        addToCurrSlice(currEdge, currSurf, odom);
        // init odom factor
        initOdomFactor();
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

    void updateKeyframes(const pcl::PointCloud<PointT>::Ptr& cloud_in_edge, const pcl::PointCloud<PointT>::Ptr& cloud_in_surf) {
        if (is_keyframe_next) {
            current_keyframe = boost::make_shared<Keyframe>(keyframes.size(), cloud_in_edge, cloud_in_surf);
            printf("This is keyframe!!!! %d\n", int(keyframes.size()));
            keyframes.push_back(current_keyframe);
            is_keyframe_next = false;
        }
    }

    void pubOdomAndPath() {

        odometry_submap = poseToNavOdometry(cloud_in_time, odom, "map", "base_link");

        odometry_slide_window = poseToNavOdometry(cloud_in_time, odom, "map", "base_link");

        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(odom.translation().x(), odom.translation().y(), odom.translation().z()) );
        Eigen::Quaterniond q_current(odom.rotation().matrix());
        tf::Quaternion q(q_current.x(),q_current.y(),q_current.z(),q_current.w());
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, cloud_in_time, "map", "base_link"));


        geometry_msgs::PoseStamped laserPose;
        laserPose.header = odometry_submap.header;
        laserPose.pose = odometry_submap.pose.pose;
        path_submap.header.stamp = odometry_submap.header.stamp;
        path_submap.poses.clear();
        for (size_t i = 0; i < keyframes.size(); i++) {
            laserPose.header = odometry_submap.header;
            Eigen::Quaterniond q(keyframes[i]->pose.rotation().matrix());
            laserPose.pose.orientation.x    = q.x();
            laserPose.pose.orientation.y    = q.y();
            laserPose.pose.orientation.z    = q.z();
            laserPose.pose.orientation.w    = q.w();
            laserPose.pose.position.x       = keyframes[i]->pose.translation().x();
            laserPose.pose.position.y       = keyframes[i]->pose.translation().y();
            laserPose.pose.position.z       = keyframes[i]->pose.translation().z();
            path_submap.poses.push_back(laserPose);
        }
        path_submap.poses.push_back(laserPose);
        path_submap.header.frame_id = "map";
        pub_path_submap.publish(path_submap);

        laserPose.header = odometry_slide_window.header;
        laserPose.pose = odometry_slide_window.pose.pose;
        path_slide_window.header.stamp = odometry_slide_window.header.stamp;
        path_slide_window.poses.push_back(laserPose);
        path_slide_window.header.frame_id = "map";
        pub_path_slide_window.publish(path_slide_window);


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

    void getTransToSubMap(pcl::PointCloud<PointT>::Ptr currEdge, pcl::PointCloud<PointT>::Ptr currSurf) {

        gtsam::Pose3 odom_prediction = odom_submap * (last_odom_submap.inverse() * odom_submap);
        odom_prediction = pose_normalize(odom_prediction);

        last_odom_submap = odom_submap;
        odom_submap = odom_prediction;

        // 'pose_w_c' to be optimize
        pose_w_c_submap = odom_submap;

        const auto state_key = X(0);

        if (pointCloudEdgeMap->size() > 10 && pointCloudSurfMap->size() > 50) {

            kdtreeEdgeMap->setInputCloud(pointCloudEdgeMap);
            kdtreeSurfMap->setInputCloud(pointCloudSurfMap);

            for (int opti_counter = 0; opti_counter < 3; opti_counter++) {

                gtsam::NonlinearFactorGraph factors;
                gtsam::Values init_values;
                init_values.insert(state_key, pose_w_c_submap);

                addEdgeCostFactor(currEdge, pointCloudEdgeMap, kdtreeEdgeMap, pose_w_c_submap, factors, 0);
                addSurfCostFactor(currSurf, pointCloudSurfMap, kdtreeSurfMap, pose_w_c_submap, factors, 0);

                // gtsam
                gtsam::LevenbergMarquardtParams params;
                params.setLinearSolverType("MULTIFRONTAL_CHOLESKY");
                params.setRelativeErrorTol(1e-4);
                params.maxIterations = 6;

                auto result = gtsam::LevenbergMarquardtOptimizer(factors, init_values, params).optimize();
                pose_w_c_submap = result.at<gtsam::Pose3>(state_key);

            }

        } else {
            printf("not enough points in submap to associate, error\n");
        }
        odom_submap = pose_w_c_submap;

        addToSubMap(currEdge, currSurf);
    }

    void getTransToSlideWindow(pcl::PointCloud<PointT>::Ptr currEdge, pcl::PointCloud<PointT>::Ptr currSurf) {

        // if empty, means this is start of a new keyframe slice
        if (isCurrKeyframeSliceEmpty()) {
            currEdge = current_keyframe->edgeFeatures;
            currSurf = current_keyframe->surfFeatures;
        }

        updateSlideWindows();

        gtsam::Pose3 odom_prediction = pose_normalize(odom * delta);

        last_odom = odom;
        odom = odom_prediction;

        // 'pose_w_c' to be optimize
        pose_w_c = odom;

        const auto state_key = X(0);

        if (slideWindowEdge->size() > 10 && slideWindowSurf->size() > 50) {

            kdtreeEdgeSW->setInputCloud(slideWindowEdge);
            kdtreeSurfSW->setInputCloud(slideWindowSurf);

            for (int opti_counter = 0; opti_counter < 3; opti_counter++) {

                gtsam::NonlinearFactorGraph factors;
                gtsam::Values init_values;
                init_values.insert(state_key, pose_w_c);

                addEdgeCostFactor(currEdge, slideWindowEdge, kdtreeEdgeSW, pose_w_c, factors, 0);
                addSurfCostFactor(currSurf, slideWindowSurf, kdtreeSurfSW, pose_w_c, factors, 0);

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

    void handleRegistration(pcl::PointCloud<PointT>::Ptr currEdge, pcl::PointCloud<PointT>::Ptr currSurf) {

        if (isCurrKeyframeSliceEmpty()) {
            // if current slice is empty, means last index of keyframes have complete slice
            int lastCompleteIndex = current_keyframe->index - 1;

            // push to loopDetectBuf
            loop_mtx.lock();
            loopDetectBuf.push(keyframes[lastCompleteIndex]);
            loop_mtx.unlock();

            current_keyframe->pose = odom;
            addToCurrSlice(current_keyframe->edgeSlice, current_keyframe->surfSlice, odom);
            addOdomFactor(lastCompleteIndex);
            factorGraphUpdate();

        } else {
            is_keyframe_next = nextFrameToBeKeyframe();
//            addToCurrSlice(currEdge, currSurf, odom);
            delta = pose_normalize(last_odom.inverse() * odom);
        }

    }

    void factorGraphUpdate() {
//        string filepath = "/home/ziv/mloam/";
        // before loop
//        if (loop_found)
//        {
//            // save
//            graph_count++;
//
//            std::ofstream of_save_graph(filepath + "isam_before" + to_string(graph_count) + ".dot");
//            isam->getFactorsUnsafe().saveGraph(of_save_graph, isam->calculateEstimate());
//        }

        isam->update(poseGraph, initEstimate);
        isam->update();

        if (loop_found) {
            isam->update();
            isam->update();
        }

        // after loop
//        if (loop_found)
//        {
//            // save
//            std::ofstream of_save_graph(filepath + "isam_after" + to_string(graph_count) + ".dot");
//            isam->getFactorsUnsafe().saveGraph(of_save_graph, isam->calculateEstimate());
//        }


        poseGraph.resize(0);
        initEstimate.clear();

        isamOptimize = isam->calculateEstimate();

        if (loop_found) {
            correctPose();
            loop_found = false;
        }

        auto latestEstimate = isamOptimize.at<gtsam::Pose3>(X(keyframes.size() - 1));

        cout << "origin odom: \n" << odom.matrix() << endl;

        delta = pose_normalize(last_odom.inverse() * odom);

        odom = latestEstimate;

        cout << "latest odom: \n" << latestEstimate.matrix() << endl;

    }

    void correctPose() {

        int numPoses = isamOptimize.size();

        string filename = "/home/ziv/mloam/loop_correct" + std::to_string(correct_count++) + "/";
        _mkdir(filename + "1.txt");
        std::ofstream f1(filename+"before.txt");
        std::ofstream f2(filename+"after.txt");

        f1 << "keyframeVec size " << keyframes.size() << endl;
        f1 << "isam size " << numPoses << endl;
        for (int i = 0; i < numPoses; i++) {
            f1 << pose_to_str(keyframes[i]->pose);
            auto poseOpti = isamOptimize.at<gtsam::Pose3>(X(i));
            keyframes[i]->pose = poseOpti;
            f2 << pose_to_str(keyframes[i]->pose);
        }
        f1.close();
        f2.close();

        flush_map = true;

    }

    void saveOdomGraph() {
        std::ofstream if_graph("/home/ziv/mloam/mloam.dot");
        copyGraph.saveGraph(if_graph, copyEstimate);
        if_graph.close();
    }

    void updateOdomWithFrame(pcl::PointCloud<PointT>::Ptr currEdge, pcl::PointCloud<PointT>::Ptr currSurf) {

        if (!is_init) {
            initWithFirstFrame(currEdge, currSurf);
            is_init = true;
            std::cout << "Initialization finished \n";
            return;
        }

        pcl::PointCloud<PointT>::Ptr currEdgeDS(new pcl::PointCloud<PointT>());
        pcl::PointCloud<PointT>::Ptr currSurfDS(new pcl::PointCloud<PointT>());

        downsampling(currEdge, *currEdgeDS, FeatureType::Edge);
        downsampling(currSurf, *currSurfDS, FeatureType::Surf);

//        getTransToSubMap(currEdgeDS, currSurfDS);
        getTransToSlideWindow(currEdgeDS, currSurfDS);

        handleRegistration(currEdgeDS, currSurfDS);

        pubOdomAndPath();
//        addToGlobalMapOdom(currEdgeDS, currSurfDS);
//        saveOdomGraph();
    }

    bool loopMatching(
        const pcl::PointCloud<PointT>::Ptr& cropEdge,
        const pcl::PointCloud<PointT>::Ptr& cropSurf,
        const pcl::PointCloud<PointT>::Ptr& keyframeEdge,
        const pcl::PointCloud<PointT>::Ptr& keyframeSurf,
        const gtsam::Pose3& pose_guess,
        gtsam::Pose3& pose_out) {

        const auto state_key = X(0);

        gtsam::Pose3 pose = pose_guess;

        if (cropEdge->size() > 100 && cropSurf->size() > 300) {

            kdtreeEdgeLoop->setInputCloud(cropEdge);
            kdtreeSurfLoop->setInputCloud(cropSurf);

            for (int opti_counter = 0; opti_counter < 3; opti_counter++) {

                gtsam::NonlinearFactorGraph factors;
                gtsam::Values init_values;
                init_values.insert(state_key, pose);

                int edge_factor_count = addEdgeCostFactor(keyframeEdge, cropEdge, kdtreeEdgeLoop, pose, factors, 0);
                int surf_factor_count = addSurfCostFactor(keyframeSurf, cropSurf, kdtreeSurfLoop, pose, factors, 0);

                if (edge_factor_count < 50 || surf_factor_count < 200) {
                    printf("edge surf features not enough %d %d\n", edge_factor_count, surf_factor_count);
                    return false;
                }

                // gtsam
                gtsam::LevenbergMarquardtParams params;
                params.setLinearSolverType("MULTIFRONTAL_CHOLESKY");
                params.setRelativeErrorTol(1e-4);
                params.maxIterations = 6;
//
//                // solve
                auto result = gtsam::LevenbergMarquardtOptimizer(factors, init_values, params).optimize();
//
//                // write result
                pose = result.at<gtsam::Pose3>(state_key);

            }

        } else {
            printf("not enough points in submap to loopMatching, error\n");
            return false;
        }
        pose_out = pose;
        return true;
    }

    bool loopMacthing_ICP(
            const pcl::PointCloud<PointT>::Ptr& cropEdge,
            const pcl::PointCloud<PointT>::Ptr& cropSurf,
            const pcl::PointCloud<PointT>::Ptr& keyframeEdge,
            const pcl::PointCloud<PointT>::Ptr& keyframeSurf,
            const gtsam::Pose3& pose_guess,
            gtsam::Pose3& pose) {

        pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
        gicp.setMaxCorrespondenceDistance(5);
        gicp.setMaximumIterations(100);
        gicp.setTransformationEpsilon(1e-6);
        gicp.setEuclideanFitnessEpsilon(1e-6);

        pcl::PointCloud<PointT>::Ptr crop(new pcl::PointCloud<PointT>());
        pcl::PointCloud<PointT>::Ptr keyframe(new pcl::PointCloud<PointT>());

        *crop += *cropEdge;
        *crop += *cropSurf;

        *keyframe += *keyframeEdge;
        *keyframe += *keyframeSurf;

        string filepath = "/home/ziv/mloam/" +std::to_string(save_count++) + "/";
        _mkdir(filepath + "1.txt");
        std::ofstream f(filepath+"init.txt");
        f << pose_guess.matrix();
        f.close();
        pcl::io::savePCDFileASCII(filepath + "crop.pcd", *crop);
        pcl::io::savePCDFileASCII(filepath + "keyframe.pcd", *keyframe);

        // Align clouds
        gicp.setInputSource(keyframe);
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
            if (distance > 15)
                continue;
            candidates.emplace_back(i, distance);
        }

        if (candidates.empty())
            return;

        std::sort(candidates.begin(), candidates.end(), [](const auto& p1, const auto& p2) {
            return p1.second < p2.second;
        });

        pcl::PointCloud<PointT>::Ptr cropEdge(new pcl::PointCloud<PointT>());
        pcl::PointCloud<PointT>::Ptr cropSurf(new pcl::PointCloud<PointT>());

        pcl::PointCloud<PointT>::Ptr tempEdge(new pcl::PointCloud<PointT>());
        pcl::PointCloud<PointT>::Ptr tempSurf(new pcl::PointCloud<PointT>());

        auto closestKeyIdx = candidates[0].first;
        int start_crop = max(0, closestKeyIdx - LOOP_KEYFRAME_CROP_LEN / 2);
        int end_crop = min(latest_index - LOOP_LATEST_KEYFRAME_SKIP, closestKeyIdx + LOOP_KEYFRAME_CROP_LEN / 2);

        // crop submap to closestKeyIdx's pose frame
        auto closestPoseInverse = keyframes[closestKeyIdx]->pose.inverse();
        for (int k = start_crop; k < end_crop; k++) {
            Eigen::Isometry3d correct_pose((closestPoseInverse * keyframes[k]->pose).matrix());
            pcl::transformPointCloud(*keyframes[k]->edgeSlice, *tempEdge, correct_pose.matrix());
            pcl::transformPointCloud(*keyframes[k]->surfSlice, *tempSurf, correct_pose.matrix());
            *cropEdge += *tempEdge;
            *cropSurf += *tempSurf;
        }

        gtsam::Pose3 pose_crop_latest_opti;

        auto pose_crop_latest_coarse = keyframes[closestKeyIdx]->pose.between(latestKeyframe->pose);

//        bool can_match = loopMatching(cropEdge, cropSurf, latestKeyframe->edgeSlice, latestKeyframe->surfSlice,
//                                      pose_latest_crop_coarse, pose_latest_crop_opti);
        cout << "pose coarse: \n" << pose_crop_latest_coarse.matrix() << endl;
//        cout << "pose gtsam featrue: \n" << pose_latest_crop_opti.matrix() << endl;
        bool can_match = loopMacthing_ICP(cropEdge, cropSurf, latestKeyframe->edgeSlice, latestKeyframe->surfSlice, pose_crop_latest_coarse, pose_crop_latest_opti);
        cout << "pose after icp "<< save_count - 1 << ": \n" << pose_crop_latest_opti.matrix() << endl;
        if (!can_match)
            return;

        loopFactors.emplace_back(new gtsam::BetweenFactor<gtsam::Pose3>( X(closestKeyIdx), X(latest_index), pose_crop_latest_opti, odometry_noise_model) );
        cout << "find loop: [" << latest_index << "] and [" << closestKeyIdx << "]\n";

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

                timePointCloudFull = pointCloudFullBuf.front()->header.stamp.toSec();
                timePointCloudEdge = pointCloudEdgeBuf.front()->header.stamp.toSec();
                timePointCloudSurf = pointCloudSurfBuf.front()->header.stamp.toSec();

                if (timePointCloudFull != timePointCloudEdge || timePointCloudFull != timePointCloudSurf) {
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

                // if is keyframe, append to keyframes vector
                updateKeyframes(cloud_in_edge, cloud_in_surf);

                updateOdomWithFrame(cloud_in_edge, cloud_in_surf);

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

    void gps_ground_truth() {
        boost::optional<Eigen::Vector3d> zero_utm;
        while (ros::ok()) {
            if (!gps_queue.empty()) {

                gps_msg_mtx.lock();
                auto curr_gps = gps_queue.front();
                gps_queue.pop();
                gps_msg_mtx.unlock();

                // convert (latitude, longitude, altitude) -> (easting, northing, altitude) in UTM coordinate
                geodesy::UTMPoint utm;
                geodesy::fromMsg((*curr_gps).position, utm);
                Eigen::Vector3d xyz(-utm.easting, utm.northing, utm.altitude);

                // the first gps data position will be the origin of the map
                if(!zero_utm) {
                    zero_utm = xyz;
                }
                xyz -= (*zero_utm);

                odometry_gt.header.frame_id = "map";
                odometry_gt.child_frame_id = "base_link";
                odometry_gt.header.stamp = curr_gps->header.stamp;
                Eigen::Quaterniond q2(odom.rotation().matrix());
                odometry_gt.pose.pose.orientation.x   = 0;
                odometry_gt.pose.pose.orientation.y   = 0;
                odometry_gt.pose.pose.orientation.z   = 0;
                odometry_gt.pose.pose.orientation.w   = 1;
                odometry_gt.pose.pose.position.x      = xyz.x();
                odometry_gt.pose.pose.position.y      = xyz.y();
                odometry_gt.pose.pose.position.z      = xyz.z();

                geometry_msgs::PoseStamped laserPose;
                laserPose.header = odometry_gt.header;
                laserPose.pose = odometry_gt.pose.pose;
                path_gt.header.stamp = odometry_gt.header.stamp;
                path_gt.poses.push_back(laserPose);
                path_gt.header.frame_id = "map";
                pub_path_gt.publish(path_gt);

            }

            //sleep 2 ms every time
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    void pub_global_map() {

        while (ros::ok()) {

            int size = keyframes.size();
            if (!is_init) {
                continue;
            }
            if (isCurrKeyframeSliceEmpty()) {
                size--;
            }

            pcl::PointCloud<PointT>::Ptr tempEdge(new pcl::PointCloud<PointT>());
            pcl::PointCloud<PointT>::Ptr tempSurf(new pcl::PointCloud<PointT>());
            pcl::PointCloud<PointT>::Ptr globalMap(new pcl::PointCloud<PointT>());

            globalMap->clear();

//            for (int i = 0; i < size; i++) {
//                pcl::transformPointCloud(*keyframes[i]->edgeSlice, *tempEdge, keyframes[i]->pose.matrix());
//                pcl::transformPointCloud(*keyframes[i]->surfSlice, *tempSurf, keyframes[i]->pose.matrix());
//                *globalMapEdge += *tempEdge;
//                *globalMapSurf += *tempSurf;
//            }
//            downsampleMap2.setInputCloud(globalMapEdge);
//            downsampleMap2.filter(*globalMapEdge);
//            downsampleMap2.setInputCloud(globalMapSurf);
//            downsampleMap2.filter(*globalMapSurf);

//            *globalMap2 += *globalMapEdge;
//            *globalMap2 += *globalMapSurf;
            globalMap = mapGenerator.generate(keyframes, 0.2);

//            last_keyframe_index = size;
//            flush_map = false;

            sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2());
            pcl::toROSMsg(*globalMap, *cloud_msg);
            cloud_msg->header.stamp = cloud_in_time;
            cloud_msg->header.frame_id = "map";

            pub_map_slide_window.publish(cloud_msg);

            if (size == 298) {
                saveMap();
            }

            //sleep 100 ms every time
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

    }

    void allocateMem() {
        pointCloudEdgeMap  = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
        pointCloudSurfMap  = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
        slideWindowEdge    = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
        slideWindowSurf    = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
//        globalMapEdge      = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
//        globalMapSurf      = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
        globalMap_odom     = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());

        kdtreeEdgeMap      = pcl::KdTreeFLANN<PointT>::Ptr(new pcl::KdTreeFLANN<PointT>());
        kdtreeSurfMap      = pcl::KdTreeFLANN<PointT>::Ptr(new pcl::KdTreeFLANN<PointT>());
        kdtreeEdgeSW       = pcl::KdTreeFLANN<PointT>::Ptr(new pcl::KdTreeFLANN<PointT>());
        kdtreeSurfSW       = pcl::KdTreeFLANN<PointT>::Ptr(new pcl::KdTreeFLANN<PointT>());
        kdtreeEdgeLoop     = pcl::KdTreeFLANN<PointT>::Ptr(new pcl::KdTreeFLANN<PointT>());
        kdtreeSurfLoop     = pcl::KdTreeFLANN<PointT>::Ptr(new pcl::KdTreeFLANN<PointT>());
    }

    void initROSHandler() {

        sub_laser_cloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points_2", 100, &LaserOdometry::pointCloudFullHandler, this);

        sub_laser_cloud_edge = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_edge", 100, &LaserOdometry::pointCloudEdgeHandler, this);

        sub_laser_cloud_surf = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf", 100, &LaserOdometry::pointCloudSurfHandler, this);

        sub_gps = nh.subscribe<sensor_msgs::NavSatFix>("/kitti/oxts/gps/fix", 1000, &LaserOdometry::gpsHandler, this);

        pub_path_submap = nh.advertise<nav_msgs::Path>("/path_submap", 100);

        pub_path_slide_window = nh.advertise<nav_msgs::Path>("/path_slide_window", 100);

        pub_path_gt = nh.advertise<nav_msgs::Path>("/path_gt", 100);

        pub_map_slide_window = nh.advertise<sensor_msgs::PointCloud2>("/map_slide_window", 5);

        pub_map_submap = nh.advertise<sensor_msgs::PointCloud2>("/map_submap", 5);

        pub_curr_edge = nh.advertise<sensor_msgs::PointCloud2>("/curr_edge", 10);

        pub_curr_surf = nh.advertise<sensor_msgs::PointCloud2>("/curr_surf", 10);

    }

    void saveMap() {
//        pcl::io::savePCDFileASCII("/home/ziv/mloam/global_map_odom.pcd", *globalMap_odom);
        pcl::PointCloud<PointT>::Ptr gmap(new pcl::PointCloud<PointT>());

        gmap = mapGenerator.generate(keyframes, 0.2);
        pcl::io::savePCDFileASCII("/home/ziv/mloam/global_map_opti.pcd", *gmap);

        cout << "saved map!" << endl;
    }

    ~LaserOdometry() {

        cout << "exit!" << endl;


    }

    void initParam() {

        map_resolution = nh.param<double>("map_resolution", 0.2);

        downSizeFilterEdge.setLeafSize(map_resolution, map_resolution, map_resolution);
        downSizeFilterSurf.setLeafSize(map_resolution * 2, map_resolution * 2, map_resolution * 2);
        downsampleMap.setLeafSize(0.2, 0.2, 0.2);
        downsampleMap2.setLeafSize(0.2, 0.2, 0.2);

        pose_w_c = gtsam::Pose3::identity();
        odom    = gtsam::Pose3::identity();
        last_odom = gtsam::Pose3::identity();
        delta = gtsam::Pose3::identity();

        pose_w_c_submap = gtsam::Pose3::identity();
        odom_submap    = gtsam::Pose3::identity();
        last_odom_submap = gtsam::Pose3::identity();

        edge_gaussian_model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(3) << 1.0, 1.0, 1.0).finished());
        surf_gaussian_model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(1) << 1.0).finished());

        edge_noise_model = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(0.1), edge_gaussian_model);
        surf_noise_model = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(0.1), surf_gaussian_model);

        prior_gaussian_model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-6).finished()); // rad*rad, meter*meter
        odometry_gaussian_model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-6).finished());

        prior_noise_model = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(0.1), prior_gaussian_model);
        odometry_noise_model = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(0.1), odometry_gaussian_model);

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

    int frameCount = 0;

//    PoseWriter pW1;
//    PoseWriter pW2;

    ros::NodeHandle nh;
    ros::Time cloud_in_time;

    MapGenerator mapGenerator;

    std::vector<Keyframe::Ptr> keyframes;
    Keyframe::Ptr current_keyframe;

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

    // frame-to-submap
    pcl::PointCloud<PointT>::Ptr pointCloudEdgeMap;
    pcl::PointCloud<PointT>::Ptr pointCloudSurfMap;
    // frame-to-slidewindow
    pcl::PointCloud<PointT>::Ptr slideWindowEdge;
    pcl::PointCloud<PointT>::Ptr slideWindowSurf;
    // global-map of frame-to-slidewindow
    pcl::PointCloud<PointT>::Ptr globalMapEdge;
    pcl::PointCloud<PointT>::Ptr globalMapSurf;
    pcl::PointCloud<PointT>::Ptr globalMap_odom;

    pcl::KdTreeFLANN<PointT>::Ptr kdtreeEdgeMap;
    pcl::KdTreeFLANN<PointT>::Ptr kdtreeSurfMap;
    pcl::KdTreeFLANN<PointT>::Ptr kdtreeEdgeSW;
    pcl::KdTreeFLANN<PointT>::Ptr kdtreeSurfSW;
    pcl::KdTreeFLANN<PointT>::Ptr kdtreeEdgeLoop;
    pcl::KdTreeFLANN<PointT>::Ptr kdtreeSurfLoop;

    // downsample filter
    pcl::VoxelGrid<PointT> downSizeFilterEdge;
    pcl::VoxelGrid<PointT> downSizeFilterSurf;
    pcl::VoxelGrid<PointT> downsampleMap;
    pcl::VoxelGrid<PointT> downsampleMap2;
    //local map
    pcl::CropBox<PointT> cropBoxFilter;
    pcl::CropBox<PointT> cropLoopFilter;

    ros::Subscriber sub_laser_cloud, sub_laser_cloud_edge, sub_laser_cloud_surf, sub_gps;
    ros::Publisher pub_path_submap, pub_path_slide_window, pub_path_gt, pub_map_slide_window, pub_map_submap;
    ros::Publisher pub_curr_edge, pub_curr_surf;

    nav_msgs::Path path_submap, path_slide_window, path_gt;
    nav_msgs::Odometry odometry_submap, odometry_slide_window, odometry_gt;

    double map_resolution;

    double timePointCloudFull = 0, timePointCloudEdge = 0, timePointCloudSurf = 0;
    bool is_init = false;
    bool is_keyframe_next = true;

    const int SLIDE_KEYFRAME_SUBMAP_LEN = 6;
    const int LOOP_KEYFRAME_CROP_LEN = 6;
    const int LOOP_LATEST_KEYFRAME_SKIP = 30;
    const int LOOP_COOLDOWN_KEYFRAME_COUNT = 5;


    double keyframeDistThreshold = 1;
    double keyframeAngleThreshold = 0.15;
    int last_loop_found_index = 0;
    bool loop_found = false;
    bool flush_map = false;
    int last_keyframe_index = 0;

    // gtsam
    gtsam::NonlinearFactorGraph poseGraph;
    gtsam::NonlinearFactorGraph copyGraph;
    gtsam::Values initEstimate;
    gtsam::Values copyEstimate;
    std::shared_ptr<gtsam::ISAM2> isam;
    gtsam::Values isamOptimize;

    gtsam::Pose3 pose_w_c_submap;  // world to current
    gtsam::Pose3 last_odom_submap;
    gtsam::Pose3 odom_submap;

    gtsam::Pose3 pose_w_c;  // world to current
    gtsam::Pose3 last_odom;
    gtsam::Pose3 odom;
    gtsam::Pose3 delta;

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