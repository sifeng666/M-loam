//
// Created by ziv on 2020/10/26.
//

#include "helper.h"
#include "frame.h"
#include "factors.h"
#include "lidarFactor.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/crop_box.h>



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

    void flushMap() {
        pointCloudEdgeMap->clear();
        pointCloudSurfMap->clear();
    }

    void pointAssociateToMap(PointT const *const pi, PointT *const po) {
        Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
        Eigen::Vector3d point_w = odom_submap.rotation() * point_curr + odom_submap.translation();
        po->x = point_w.x();
        po->y = point_w.y();
        po->z = point_w.z();
        po->intensity = pi->intensity;
    }

    void pointAssociateToSlideWindow(PointT const *const pi, PointT *const po) {
        Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
        Eigen::Vector3d point_w = odom.rotation() * point_curr + odom.translation();
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

    void addToLastKeyframe(pcl::PointCloud<PointT>::Ptr currEdge = nullptr, pcl::PointCloud<PointT>::Ptr currSurf = nullptr) {

        if (!currEdge) currEdge = currFrame->edgeFeatures;
        if (!currSurf) currSurf = currFrame->surfFeatures;
        auto lastKeyframeEdge = lastKeyframe->getEdgeSubMap();
        auto lastKeyframeSurf = lastKeyframe->getSurfSubMap();

        pcl::PointCloud<PointT>::Ptr transformedEdge(new pcl::PointCloud<PointT>());
        pcl::PointCloud<PointT>::Ptr transformedSurf(new pcl::PointCloud<PointT>());

        // all frame in same slice transform to keyframe's pose
        Eigen::Isometry3d T_curr2lastKeyframe((last_keyframe_odom.inverse() * odom).matrix());

        pcl::transformPointCloud(*currEdge, *transformedEdge, T_curr2lastKeyframe.matrix());
        pcl::transformPointCloud(*currSurf, *transformedSurf, T_curr2lastKeyframe.matrix());

        *lastKeyframeEdge += *transformedEdge;
        *lastKeyframeSurf += *transformedSurf;

        downSizeFilterEdge.setInputCloud(lastKeyframeEdge);
        downSizeFilterEdge.filter(*lastKeyframeEdge);
        downSizeFilterSurf.setInputCloud(lastKeyframeSurf);
        downSizeFilterSurf.filter(*lastKeyframeSurf);

        lastKeyframe->setEdgeSubMap(lastKeyframeEdge);
        lastKeyframe->setSurfSubMap(lastKeyframeSurf);

    }

    void addToSubMap(pcl::PointCloud<PointT>::Ptr currEdge = nullptr, pcl::PointCloud<PointT>::Ptr currSurf = nullptr) {

        if (!currEdge) currEdge = currFrame->edgeFeatures;
        if (!currSurf) currSurf = currFrame->surfFeatures;
        PointT point_temp;

        for (int i = 0; i < currEdge->size(); i++) {
            pointAssociateToMap(&currEdge->points[i], &point_temp);
            pointCloudEdgeMap->push_back(point_temp);
        }

        for (int i = 0; i < currSurf->size(); i++) {
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

    void addToGlobalMap(pcl::PointCloud<PointT>::Ptr currEdge = nullptr, pcl::PointCloud<PointT>::Ptr currSurf = nullptr) {

        if (!currEdge) currEdge = currFrame->edgeFeatures;
        if (!currSurf) currSurf = currFrame->surfFeatures;

        pcl::PointCloud<PointT>::Ptr transformedEdge(new pcl::PointCloud<PointT>());
        pcl::PointCloud<PointT>::Ptr transformedSurf(new pcl::PointCloud<PointT>());

        pcl::transformPointCloud(*currEdge, *transformedEdge, odom.matrix());
        pcl::transformPointCloud(*currSurf, *transformedSurf, odom.matrix());

        *globalMapEdge += *transformedEdge;
        *globalMapSurf += *transformedSurf;

        downSizeFilterEdge.setInputCloud(globalMapEdge);
        downSizeFilterEdge.filter(*globalMapEdge);
        downSizeFilterSurf.setInputCloud(globalMapSurf);
        downSizeFilterSurf.filter(*globalMapSurf);
    }

    void updateSlideWindows() {

        slideWindowEdge->clear();
        slideWindowSurf->clear();

        pcl::PointCloud<PointT>::Ptr tempEdge(new pcl::PointCloud<PointT>());
        pcl::PointCloud<PointT>::Ptr tempSurf(new pcl::PointCloud<PointT>());

        if (keyframeVec.size() < SLIDE_KEYFRAME_SUBMAP_LEN) {
            for (int count = 0; count < keyframeVec.size(); count++) {
                pcl::transformPointCloud(*frameMap[keyframeVec[count]]->getEdgeSubMap(), *tempEdge, frameMap[keyframeVec[count]]->pose.matrix());
                pcl::transformPointCloud(*frameMap[keyframeVec[count]]->getSurfSubMap(), *tempSurf, frameMap[keyframeVec[count]]->pose.matrix());
                *slideWindowEdge += *tempEdge;
                *slideWindowSurf += *tempSurf;
            }
        } else {
            for (int count = keyframeVec.size() - SLIDE_KEYFRAME_SUBMAP_LEN; count < keyframeVec.size(); count++) {
                pcl::transformPointCloud(*frameMap[keyframeVec[count]]->getEdgeSubMap(), *tempEdge, frameMap[keyframeVec[count]]->pose.matrix());
                pcl::transformPointCloud(*frameMap[keyframeVec[count]]->getSurfSubMap(), *tempSurf, frameMap[keyframeVec[count]]->pose.matrix());
                *slideWindowEdge += *tempEdge;
                *slideWindowSurf += *tempSurf;
            }
        }

    }

    bool nextFrameToBeKeyframe() {

//        if (toBeKeyframeInterval < MIN_KEYFRAME_INTERVAL) {
//            toBeKeyframeInterval++;
//            return false;
//        }
//
//        if (toBeKeyframeInterval >= MAX_KEYFRAME_INTERVAL) {
//            toBeKeyframeInterval = 0;
//            return true;
//        }
        Eigen::Isometry3d T_delta((last_keyframe_odom.inverse() * odom).matrix());
        Eigen::Quaterniond q_delta(T_delta.rotation().matrix());
        q_delta.normalize();
        Eigen::Vector3d eulerAngle = q_delta.toRotationMatrix().eulerAngles(2, 1, 0);

        bool isKeyframe = min(abs(eulerAngle.x()), abs(abs(eulerAngle.x()) - M_PI)) > keyframeAngleThreshold ||
                          min(abs(eulerAngle.y()), abs(abs(eulerAngle.y()) - M_PI)) > keyframeAngleThreshold ||
                          min(abs(eulerAngle.z()), abs(abs(eulerAngle.z()) - M_PI)) > keyframeAngleThreshold ||
                          T_delta.translation().norm() > keyframeDistThreshold;
        return isKeyframe;

//        if (isKeyframe) {
//            toBeKeyframeInterval = 0;
//            return true;
//        } else {
//            toBeKeyframeInterval++;
//            return false;
//        }
    }

    void initWithFirstFrame() {

        // init local submap
        *pointCloudEdgeMap += *currFrame->edgeFeatures;
        *pointCloudSurfMap += *currFrame->surfFeatures;

        *slideWindowEdge += *currFrame->edgeFeatures;
        *slideWindowSurf += *currFrame->surfFeatures;

        *globalMapEdge += *currFrame->edgeFeatures;
        *globalMapSurf += *currFrame->surfFeatures;

        // for keyframe, set first frame as keyframe
        lastKeyframe = currFrame;
        lastKeyframe->alloc();
        addToLastKeyframe();
        keyframeVec.push_back(frameCount);

        // init odom factor
        initOdomFactor();
    }

    void updateOdomWithFrame() {

        if (!is_init) {
            initWithFirstFrame();
            is_init = true;
            std::cout << "Initialization finished \n";
            return;
        }

        pcl::PointCloud<PointT>::Ptr edgeFeaturesDS(new pcl::PointCloud<PointT>());
        pcl::PointCloud<PointT>::Ptr surfFeaturesDS(new pcl::PointCloud<PointT>());

        downSizeFilterEdge.setInputCloud(currFrame->edgeFeatures);
        downSizeFilterEdge.filter(*edgeFeaturesDS);
        downSizeFilterSurf.setInputCloud(currFrame->surfFeatures);
        downSizeFilterSurf.filter(*surfFeaturesDS);

//        currFrame->edgeFeatures = edgeFeaturesDS;
//        currFrame->surfFeatures = surfFeaturesDS;

//        cout << "pointCloudEdgeMap size: " << pointCloudEdgeMap->size() << " pointCloudSurfMap size: " << pointCloudSurfMap->size() << endl
//             << "slideWindowEdge size: " << slideWindowEdge->size() << " slideWindowSurf size: " << slideWindowSurf->size() << endl
//             << "edgeFeaturesDS size: " << edgeFeaturesDS->size() << " surfFeaturesDS size: " << surfFeaturesDS->size() << endl;

//        if (frameCount <= 500 && frameCount >= 1) {
//            string local_path = "/home/ziv/debuging/" + std::to_string(frameCount) + "/";
//            pcl::io::savePCDFileASCII(local_path + "edge_submap.pcd", *pointCloudEdgeMap);
//            pcl::io::savePCDFileASCII(local_path + "plane_submap.pcd", *pointCloudSurfMap);
//            pcl::io::savePCDFileASCII(local_path + "edge_feature.pcd", *edgeFeaturesDS);
//            pcl::io::savePCDFileASCII(local_path + "plane_feature.pcd", *surfFeaturesDS);
//
//            std::ofstream f(local_path + "result.txt");
//            f << "ceres:\n" << T_curr2world.matrix() << endl;
//            f << "gtsam:\n" << odom.matrix() << endl;
//            f.close();
//        }

//        if (Eigen::Isometry3d(T_curr2world.matrix()).translation().norm() < 10) {
//            pW1.write(T_curr2world, false);
//            pW2.write(odom, false);
//        } else {
//            pW1.close();
//            pW2.close();
//        }


        getTransToSubMap(edgeFeaturesDS, surfFeaturesDS);
        getTransToSlideWindow(edgeFeaturesDS, surfFeaturesDS);

        cout << "submap:\n" << odom_submap.matrix() << endl;
        cout << "slide window:\n" << odom.matrix() << endl;

        addToGlobalMap();
        pubOdomAndPath();
        pubMapSubMap();
        pubMapGlobalMap();
    }

    void pubMapGlobalMap() {

        pcl::PointCloud<PointT>::Ptr globalMap(new pcl::PointCloud<PointT>());
        sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2());

        *globalMap += *globalMapEdge;
        *globalMap += *globalMapSurf;

        pcl::toROSMsg(*globalMap, *cloud_msg);
        cloud_msg->header.stamp = cloud_in_time;
        cloud_msg->header.frame_id = "map";

        pub_map_slide_window.publish(cloud_msg);

    }

    void pubMapSubMap() {

        pcl::PointCloud<PointT>::Ptr submap(new pcl::PointCloud<PointT>());
        sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2());

        *submap += *pointCloudEdgeMap;
        *submap += *pointCloudSurfMap;

        pcl::toROSMsg(*submap, *cloud_msg);
        cloud_msg->header.stamp = cloud_in_time;
        cloud_msg->header.frame_id = "map";

        pub_map_submap.publish(cloud_msg);

    }

    void pubOdomAndPath() {

        odometry_submap = poseToNavOdometry(cloud_in_time, odom_submap, "map", "base_link");

        odometry_slide_window = poseToNavOdometry(cloud_in_time, odom, "map", "base_link");

        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(odom_submap.translation().x(), odom_submap.translation().y(), odom_submap.translation().z()) );
        Eigen::Quaterniond q_current(odom_submap.rotation().matrix());
        tf::Quaternion q(q_current.x(),q_current.y(),q_current.z(),q_current.w());
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, cloud_in_time, "map", "base_link"));


        geometry_msgs::PoseStamped laserPose;
        laserPose.header = odometry_submap.header;
        laserPose.pose = odometry_submap.pose.pose;
        path_submap.header.stamp = odometry_submap.header.stamp;
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

    void initOdomFactor() {
        poseGraph.add(gtsam::PriorFactor<gtsam::Pose3>(X(0), trans2gtsamPose3(odom.matrix()), prior_noise_model));
        initVal.insert(X(frameCount), trans2gtsamPose3(odom.matrix()));
        cout << "initGTSAM" << endl;
    }

    void addOdomFactor(int lastFrameIdx) {
        gtsam::Pose3 lastPose(frameMap[lastFrameIdx]->pose.matrix());
        poseGraph.add(gtsam::BetweenFactor<gtsam::Pose3>(X(lastFrameIdx), X(frameCount), lastPose.between(odom), odometry_noise_model));
        initVal.insert(X(frameCount), trans2gtsamPose3(odom.matrix()));
        printf("add odom factor between %d and %d\n", lastFrameIdx, frameCount);
    }

    void getTransToSubMap(pcl::PointCloud<PointT>::Ptr currEdge = nullptr, pcl::PointCloud<PointT>::Ptr currSurf = nullptr) {

        if (!currEdge) currEdge = currFrame->edgeFeatures;
        if (!currSurf) currSurf = currFrame->surfFeatures;

        gtsam::Pose3 odom_prediction = odom_submap * (last_odom_submap.inverse() * odom_submap);
        odom_prediction = pose_normalize(odom_prediction);

        last_odom_submap = odom_submap;
        odom_submap = odom_prediction;

        // 'pose_w_c' to be optimize
        pose_w_c_submap = odom_submap;

        const auto state_key = X(frameCount);

        if (pointCloudEdgeMap->size() > 10 && pointCloudSurfMap->size() > 50) {

            kdtreeEdgeMap->setInputCloud(pointCloudEdgeMap);
            kdtreeSurfMap->setInputCloud(pointCloudSurfMap);

            for (int opti_counter = 0; opti_counter < 3; opti_counter++) {

                gtsam::NonlinearFactorGraph factors;
                gtsam::Values init_values;
                init_values.insert(state_key, pose_w_c_submap);
//                if (opti_counter == 0) {
//                    initValuesSubmap.insert(state_key, pose_w_c_submap);
//                } else {
//                    initValuesSubmap.update(state_key, pose_w_c_submap);
//                }


//            string file_path("/home/ziv/debuging/" + std::to_string(frameCount) + "/" + std::to_string(opti_counter) + "/");
//            string tmp_result_filename = file_path + "tmp_result.txt";
//            _mkdir(tmp_result_filename);

                addEdgeCostFactor(currEdge, pointCloudEdgeMap, kdtreeEdgeMap, pose_w_c_submap, factors, frameCount);
                addSurfCostFactor(currSurf, pointCloudSurfMap, kdtreeSurfMap, pose_w_c_submap, factors, frameCount);

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
                pose_w_c_submap = result.at<gtsam::Pose3>(state_key);

                // gtsam

//                std::ofstream f(tmp_result_filename);
//                Eigen::Isometry3d temp = Eigen::Isometry3d::Identity();
//                temp.linear() = q_curr2world.toRotationMatrix();
//                temp.translation() = t_curr2world;
//                f << "ceres:\n" << temp.matrix() << endl;
//                f << "gtsam:\n" << pose_w_c.matrix() << endl;
//                f.close();
            }

        } else {
            printf("not enough points in submap to associate, error\n");
        }
        odom_submap = pose_w_c_submap;

        addToSubMap(currEdge, currSurf);
    }

    void getTransToSlideWindow(pcl::PointCloud<PointT>::Ptr currEdge = nullptr, pcl::PointCloud<PointT>::Ptr currSurf = nullptr) {

        if (!currEdge) currEdge = currFrame->edgeFeatures;
        if (!currSurf) currSurf = currFrame->surfFeatures;

        if (currFrame->is_keyframe()) {
            currEdge = currFrame->edgeFeatures;
            currSurf = currFrame->surfFeatures;
        }

        updateSlideWindows();

        gtsam::Pose3 odom_prediction = odom * (last_odom.inverse() * odom);
        odom_prediction = pose_normalize(odom_prediction);

        last_odom = odom;
        odom = odom_prediction;

        // 'pose_w_c' to be optimize
        pose_w_c = odom;

        const auto state_key = X(frameCount);

        if (slideWindowEdge->size() > 10 && slideWindowSurf->size() > 50) {

            kdtreeEdgeSW->setInputCloud(slideWindowEdge);
            kdtreeSurfSW->setInputCloud(slideWindowSurf);

            for (int opti_counter = 0; opti_counter < 3; opti_counter++) {

                gtsam::NonlinearFactorGraph factors;
                gtsam::Values init_values;
                init_values.insert(state_key, pose_w_c);

                addEdgeCostFactor(currEdge, slideWindowEdge, kdtreeEdgeSW, pose_w_c, factors, frameCount);
                addSurfCostFactor(currSurf, slideWindowSurf, kdtreeSurfSW, pose_w_c, factors, frameCount);

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
                pose_w_c = result.at<gtsam::Pose3>(state_key);

            }

        } else {
            printf("not enough points in submap to associate, error\n");
        }


        odom = pose_w_c;

        if (currFrame->is_keyframe()) {
            // push to loopDetectBuf
            loop_mtx.lock();
            loopDetectBuf.push(lastKeyframe);
            loop_mtx.unlock();

            int lastKeyframeIndex= lastKeyframe->frameCount;
            lastKeyframe = currFrame;
            lastKeyframe->alloc();
            last_keyframe_odom = odom;
            addToLastKeyframe(currEdge, currSurf);

            // add odom factor to last keyframe
            addOdomFactor(lastKeyframeIndex);

            keyframeVec.push_back(frameCount);

        } else {
            is_keyframe_next = nextFrameToBeKeyframe();
            addToLastKeyframe(currEdge, currSurf);
        }

    }

    void updateCurrentFrame(const pcl::PointCloud<PointT>::Ptr& cloud_in_edge, const pcl::PointCloud<PointT>::Ptr& cloud_in_surf) {
        if (is_keyframe_next) {
            currFrame = boost::make_shared<Keyframe>(frameCount, keyframeVec.size(), cloud_in_edge, cloud_in_surf);
            printf("This is keyframe!!!!\n");
            is_keyframe_next = false;
        } else {
            currFrame = boost::make_shared<Frame>(frameCount, cloud_in_edge, cloud_in_surf);
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

    void perform_loop_closure() {
        while (ros::ok()) {

            if (!loopDetectBuf.empty()) {

                Timer t_loop;

                loop_mtx.lock();
                std::vector<LoopFactor> loopFactors;
                auto latestFrame = loopDetectBuf.front();
                loopDetectBuf.pop();
                loop_mtx.unlock();

                loop_detector(latestFrame, loopFactors);
                for (const auto& factor: loopFactors) {
                    poseGraph.add(factor);
                    cout << "add loop factor" << endl;
                }
                loopFactors.clear();
                t_loop.count("loop detector");
                saveOdomGraph();
            }

            //sleep 10 ms every time
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }


    void loop_detector(const Frame::Ptr latestKeyframe, std::vector<LoopFactor>& loopFactors) {

        int keyframe_count = latestKeyframe->keyframe_count();

        if (keyframe_count < LOOP_LATEST_KEYFRAME_SKIP + 1)
            return;

        if (last_loop_found_index > 0 && keyframe_count <= last_loop_found_index + LOOP_COOLDOWN_KEYFRAME_COUNT)
            return;

        last_loop_found_index = keyframe_count;

        // <frameCount, distance>
        std::vector<pair<int, int>> candidates;
        candidates.reserve(keyframe_count - LOOP_LATEST_KEYFRAME_SKIP);
        for (int i = 0; i < keyframe_count - LOOP_LATEST_KEYFRAME_SKIP; i++) {
            auto pose_between = poseBetween(latestKeyframe->pose, frameMap[keyframeVec[i]]->pose);
            auto distance = pose_between.translation().norm();
            // too far
            if (distance > 5)
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
        int end_crop = min(keyframe_count - LOOP_LATEST_KEYFRAME_SKIP, closestKeyIdx + LOOP_KEYFRAME_CROP_LEN / 2);

        // crop submap to closestKeyIdx's pose frame
        auto closestPoseInverse = frameMap[keyframeVec[closestKeyIdx]]->pose.inverse();
        for (int k = start_crop; k < end_crop; k++) {
            Eigen::Isometry3d correct_pose((closestPoseInverse * frameMap[keyframeVec[k]]->pose).matrix());
            pcl::transformPointCloud(*frameMap[keyframeVec[k]]->getEdgeSubMap(), *tempEdge, correct_pose.matrix());
            pcl::transformPointCloud(*frameMap[keyframeVec[k]]->getSurfSubMap(), *tempSurf, correct_pose.matrix());
            *cropEdge += *tempEdge;
            *cropSurf += *tempSurf;
        }

//        auto pose_inverse = latestKeyframe->pose.inverse().matrix();
//        pcl::transformPointCloud(*latestKeyframe->getEdgeSubMap(), *copy_keyframeEdge, pose_inverse);
//        pcl::transformPointCloud(*latestKeyframe->getSurfSubMap(), *copy_keyframeSurf, pose_inverse);
//
//        pose_inverse = frameMap[keyframeVec[closestKeyIdx]]->pose.inverse().matrix();
//        pcl::transformPointCloud(*cropEdge, *cropEdge, pose_inverse);
//        pcl::transformPointCloud(*cropSurf, *cropSurf, pose_inverse);

        gtsam::Pose3 pose_finetune;
        auto pose_coarse = poseBetween(latestKeyframe->pose, frameMap[keyframeVec[closestKeyIdx]]->pose);
        bool can_match = loopMatching(cropEdge, cropSurf, latestKeyframe->getEdgeSubMap(), latestKeyframe->getSurfSubMap(),
                                      gtsam::Pose3(pose_coarse.matrix()), pose_finetune);
        if (!can_match)
            return;

        loopFactors.emplace_back(new gtsam::BetweenFactor<gtsam::Pose3>(X(lastKeyframe->frameCount), X(keyframeVec[closestKeyIdx]), pose_finetune, odometry_noise_model));
        cout << "find loop: [" << lastKeyframe->frameCount << "] and [" << keyframeVec[closestKeyIdx] << "]\n";
        cout << "pose before: \n" << pose_coarse.matrix() << endl;
        cout << "pose after: \n" << pose_finetune.matrix() << endl;

    }

    void laser_odometry() {

        cloud_in_edge.reset(new pcl::PointCloud<PointT>());
        cloud_in_surf.reset(new pcl::PointCloud<PointT>());
        cloud_in_full.reset(new pcl::PointCloud<PointT>());

        while (ros::ok()) {

            if (!pointCloudFullBuf.empty() && !pointCloudEdgeBuf.empty() && !pointCloudSurfBuf.empty()) {

                Timer t_laser_odometry;

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

                updateCurrentFrame(cloud_in_edge, cloud_in_surf);

                updateOdomWithFrame();

                currFrame->pose = odom.matrix();
                frameMap[frameCount++] = currFrame;
                t_laser_odometry.count("laser_odometry");
            }

            //sleep 2 ms every time
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    void allocateMem() {
        pointCloudEdgeMap  = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
        pointCloudSurfMap  = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
        slideWindowEdge    = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
        slideWindowSurf    = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
        globalMapEdge      = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
        globalMapSurf      = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
        copy_keyframeEdge  = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
        copy_keyframeSurf  = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());

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

    void initParam() {

//        T_curr2world = T_lastKeyframe = T_curr2worldBySW = T_last = Eigen::Isometry3d::Identity();

        map_resolution = nh.param<double>("map_resolution", 0.2);

        downSizeFilterEdge.setLeafSize(map_resolution, map_resolution, map_resolution);
        downSizeFilterSurf.setLeafSize(map_resolution * 2, map_resolution * 2, map_resolution * 2);

        pose_w_c = gtsam::Pose3::identity();
        odom    = gtsam::Pose3::identity();
        last_odom = gtsam::Pose3::identity();
        last_keyframe_odom = gtsam::Pose3::identity();

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

    }


    LaserOdometry()
    {

        allocateMem();

        initROSHandler();

        initParam();
    }

    void saveOdomGraph() {
//        std::cout << "saveOdomFactor" << std::endl;
        std::ofstream if_graph("/home/ziv/mloam.dot");
        poseGraph.saveGraph(if_graph, initVal);
    }

private:

    int frameCount = 0;

//    PoseWriter pW1;
//    PoseWriter pW2;

    ros::NodeHandle nh;
    ros::Time cloud_in_time;

    Frame::Ptr currFrame;
    Frame::Ptr lastKeyframe;

    // map of each frame or keyframe
    std::unordered_map<int, Frame::Ptr> frameMap;
    std::vector<int> keyframeVec;

    // queue of ros pointcloud msg
    std::mutex pcd_msg_mtx;
    std::mutex gps_msg_mtx;
    std::mutex loop_mtx;
    std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudEdgeBuf;
    std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudSurfBuf;
    std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudFullBuf;
    std::queue<geographic_msgs::GeoPointStampedConstPtr> gps_queue;
    std::queue<Frame::Ptr> loopDetectBuf;

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
    std::mutex global_mtx;
    pcl::PointCloud<PointT>::Ptr globalMapEdge;
    pcl::PointCloud<PointT>::Ptr globalMapSurf;
    pcl::PointCloud<PointT>::Ptr copy_keyframeEdge;
    pcl::PointCloud<PointT>::Ptr copy_keyframeSurf;

    pcl::KdTreeFLANN<PointT>::Ptr kdtreeEdgeMap;
    pcl::KdTreeFLANN<PointT>::Ptr kdtreeSurfMap;
    pcl::KdTreeFLANN<PointT>::Ptr kdtreeEdgeSW;
    pcl::KdTreeFLANN<PointT>::Ptr kdtreeSurfSW;
    pcl::KdTreeFLANN<PointT>::Ptr kdtreeEdgeLoop;
    pcl::KdTreeFLANN<PointT>::Ptr kdtreeSurfLoop;

    // downsample filter
    pcl::VoxelGrid<PointT> downSizeFilterEdge;
    pcl::VoxelGrid<PointT> downSizeFilterSurf;
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

    const int MAX_KEYFRAME_INTERVAL = 20;
    const int MIN_KEYFRAME_INTERVAL = 3;
    int toBeKeyframeInterval = 0;

    const int SLIDE_KEYFRAME_SUBMAP_LEN = 6;
    const int LOOP_KEYFRAME_CROP_LEN = 6;
    const int LOOP_LATEST_KEYFRAME_SKIP = 30;
    const int LOOP_COOLDOWN_KEYFRAME_COUNT = 5;


    double keyframeDistThreshold = 1;
    double keyframeAngleThreshold = 0.15;
    int last_loop_found_index = 0;

    // gtsam
    gtsam::NonlinearFactorGraph poseGraph;
    gtsam::Values initVal;

    gtsam::Pose3 pose_w_c_submap;  // world to current
    gtsam::Pose3 last_odom_submap;
    gtsam::Pose3 odom_submap;

    gtsam::Pose3 pose_w_c;  // world to current
    gtsam::Pose3 last_odom;
    gtsam::Pose3 odom;
    gtsam::Pose3 last_keyframe_odom;

    // gaussian model
    gtsam::SharedNoiseModel edge_gaussian_model, surf_gaussian_model, prior_gaussian_model, odometry_gaussian_model;
    // noise model
    gtsam::SharedNoiseModel edge_noise_model, surf_noise_model, prior_noise_model, odometry_noise_model;


};

int main(int argc, char **argv) {

    ros::init(argc, argv, "laserOdometry");

    LaserOdometry laserOdometry;

    std::thread laser_odometry_thread{&LaserOdometry::laser_odometry, &laserOdometry};

    std::thread gps_ground_truth_thread{&LaserOdometry::gps_ground_truth, &laserOdometry};

    std::thread loop_detector_thread{&LaserOdometry::perform_loop_closure, &laserOdometry};

    ros::spin();

}