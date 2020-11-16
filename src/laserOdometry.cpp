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

    void pointCloudFullHandler(const sensor_msgs::PointCloud2ConstPtr &pointCloudMsg) {
        std::lock_guard lg(mBuf);
        pointCloudFullBuf.push(pointCloudMsg);
    }

    void pointCloudSurfHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
        std::lock_guard lg(mBuf);
        pointCloudSurfBuf.push(laserCloudMsg);
    }

    void pointCloudEdgeHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
        std::lock_guard lg(mBuf);
        pointCloudEdgeBuf.push(laserCloudMsg);
    }

    void gpsHandler(const sensor_msgs::NavSatFixConstPtr& navsat_msg) {
        std::lock_guard lock(gps_mBuf);
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
        Eigen::Vector3d point_w = T_curr2world.rotation() * point_curr + T_curr2world.translation();
        po->x = point_w.x();
        po->y = point_w.y();
        po->z = point_w.z();
        po->intensity = pi->intensity;
    }

    void pointAssociateToSlideWindow(PointT const *const pi, PointT *const po) {
        Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
        Eigen::Vector3d point_w = T_curr2worldBySW.rotation() * point_curr + T_curr2worldBySW.translation();
        po->x = point_w.x();
        po->y = point_w.y();
        po->z = point_w.z();
        po->intensity = pi->intensity;
    }

//    void pointAssociateToKeyframeGTSAM(PointT const *const pi, PointT *const po) {
//        Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
//        Eigen::Vector3d point_w = T_curr2worldByKeyframe.rotation() * point_curr + T_curr2worldByKeyframe.translation();
//        po->x = point_w.x();
//        po->y = point_w.y();
//        po->z = point_w.z();
//        po->intensity = pi->intensity;
//    }

    void pointAssociateToTrans(PointT const *const pi, PointT *const po, const Eigen::Isometry3d& T) {
        Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
        Eigen::Vector3d point_w = T.rotation() * point_curr + T.translation();
        po->x = point_w.x();
        po->y = point_w.y();
        po->z = point_w.z();
        po->intensity = pi->intensity;
    }

    void addToLastKeyframe() {

        auto currEdge = currFrame->edgeFeatures;
        auto currSurf = currFrame->surfFeatures;
        PointT point_temp;

        for (int i = 0; i < currEdge->size(); i++) {
            pointAssociateToSlideWindow(&currEdge->points[i], &point_temp);
            lastKeyframe->addEdgeFeaturesToSubMap(point_temp);
        }

        for (int i = 0; i < currSurf->size(); i++) {
            pointAssociateToSlideWindow(&currSurf->points[i], &point_temp);
            lastKeyframe->addSurfFeaturesToSubMap(point_temp);
        }

        pcl::PointCloud<PointT>::Ptr keyframeEdgeSubMapDS(new pcl::PointCloud<PointT>());
        pcl::PointCloud<PointT>::Ptr keyframeSurfSubMapDS(new pcl::PointCloud<PointT>());

        downSizeFilterEdge.setInputCloud(lastKeyframe->getEdgeSubMap());
        downSizeFilterEdge.filter(*keyframeEdgeSubMapDS);
        downSizeFilterSurf.setInputCloud(lastKeyframe->getSurfSubMap());
        downSizeFilterSurf.filter(*keyframeSurfSubMapDS);

        lastKeyframe->setEdgeSubMap(keyframeEdgeSubMapDS);
        lastKeyframe->setSurfSubMap(keyframeSurfSubMapDS);
    }

    void addToSubMap() {

        auto currEdge = currFrame->edgeFeatures;
        auto currSurf = currFrame->surfFeatures;
        PointT point_temp;

        for (int i = 0; i < currEdge->size(); i++) {
            pointAssociateToMap(&currEdge->points[i], &point_temp);
            pointCloudEdgeMap->push_back(point_temp);
        }

        for (int i = 0; i < currSurf->size(); i++) {
            pointAssociateToMap(&currSurf->points[i], &point_temp);
            pointCloudSurfMap->push_back(point_temp);
        }

        double x_min = +T_curr2world.translation().x()-100;
        double y_min = +T_curr2world.translation().y()-100;
        double z_min = +T_curr2world.translation().z()-100;
        double x_max = +T_curr2world.translation().x()+100;
        double y_max = +T_curr2world.translation().y()+100;
        double z_max = +T_curr2world.translation().z()+100;

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

    void updateSlideWindows() {

        PointT point_temp;

        if (keyframeVec.size() >= SLIDE_KEYFRAME_SUBMAP_LEN) {

            slideWindowEdge->clear();
            slideWindowSurf->clear();

            for (int count = keyframeVec.size() - SLIDE_WINDOW_LEN; count < keyframeVec.size(); count++) {

                auto currEdge = frameMap[keyframeVec[count]]->getEdgeSubMap();
                auto currSurf = frameMap[keyframeVec[count]]->getSurfSubMap();
                auto currPose = frameMap[keyframeVec[count]]->pose;

                for (int i = 0; i < currEdge->size(); i++) {
//                    pointAssociateToTrans(&currEdge->points[i], &point_temp, currPose);
                    slideWindowEdge->push_back(currEdge->points[i]);
                }

                for (int i = 0; i < currSurf->size(); i++) {
//                    pointAssociateToTrans(&currSurf->points[i], &point_temp, currPose);
                    slideWindowSurf->push_back(currSurf->points[i]);
                }

            }

        } else if (frameCount >= SLIDE_WINDOW_LEN) { // frameCount start from 0

            slideWindowEdge->clear();
            slideWindowSurf->clear();

            for (int count = frameCount - SLIDE_WINDOW_LEN; count < frameCount; count++) {
                auto currEdge = frameMap[count]->edgeFeatures;
                auto currSurf = frameMap[count]->surfFeatures;
                auto currPose = frameMap[count]->pose;

                for (int i = 0; i < currEdge->size(); i++) {
                    pointAssociateToTrans(&currEdge->points[i], &point_temp, currPose);
                    slideWindowEdge->push_back(point_temp);
                }

                for (int i = 0; i < currSurf->size(); i++) {
                    pointAssociateToTrans(&currSurf->points[i], &point_temp, currPose);
                    slideWindowSurf->push_back(point_temp);
                }
            }

        } else { // frameCount < SLIDE_WINDOW_LEN

            // no need to flush slidewindow
            auto currEdge = frameMap[frameCount - 1]->edgeFeatures;
            auto currSurf = frameMap[frameCount - 1]->surfFeatures;
            auto currPose = frameMap[frameCount - 1]->pose;

            for (int i = 0; i < currEdge->size(); i++) {
                pointAssociateToTrans(&currEdge->points[i], &point_temp, currPose);
                slideWindowEdge->push_back(point_temp);
            }

            for (int i = 0; i < currSurf->size(); i++) {
                pointAssociateToTrans(&currSurf->points[i], &point_temp, currPose);
                slideWindowSurf->push_back(point_temp);
            }


        }

        downSizeFilterEdge.setInputCloud(slideWindowEdge);
        downSizeFilterEdge.filter(*slideWindowEdge);
        downSizeFilterSurf.setInputCloud(slideWindowSurf);
        downSizeFilterSurf.filter(*slideWindowSurf);

    }

    bool nextFrameToBeKeyframe() {

        if (toBeKeyframeInterval < MIN_KEYFRAME_INTERVAL) {
            toBeKeyframeInterval++;
            return false;
        }

        if (toBeKeyframeInterval >= MAX_KEYFRAME_INTERVAL) {
            toBeKeyframeInterval = 0;
            return true;
        }

        Eigen::Isometry3d T_delta = T_lastKeyframe.inverse() * T_curr2worldBySW;
        Eigen::Vector3d eulerAngle = T_delta.rotation().eulerAngles(2, 1, 0);

        bool isKeyframe = min(abs(eulerAngle.x()), abs(abs(eulerAngle.x()) - M_PI)) > keyframeAngleThreshold ||
                          min(abs(eulerAngle.y()), abs(abs(eulerAngle.y()) - M_PI)) > keyframeAngleThreshold ||
                          min(abs(eulerAngle.z()), abs(abs(eulerAngle.z()) - M_PI)) > keyframeAngleThreshold ||
                          T_delta.translation().norm() > keyframeDistThreshold;
//        cout << min(abs(eulerAngle.x()), abs(abs(eulerAngle.x()) - M_PI)) << " " << min(abs(eulerAngle.y()), abs(abs(eulerAngle.y()) - M_PI)) << " "
//                << min(abs(eulerAngle.z()), abs(abs(eulerAngle.z()) - M_PI)) << " " << T_delta.translation().norm() << endl;
        if (isKeyframe) {
            toBeKeyframeInterval = 0;
            return true;
        } else {
            toBeKeyframeInterval++;
            return false;
        }
    }

    void setCurrentFrameToLastKeyframe() {
        lastKeyframe = currFrame;
        lastKeyframe->alloc();

        addToLastKeyframe();

        T_lastKeyframe = T_curr2worldBySW;
        keyframeVec.push_back(frameCount);
    }

    void initWithFirstFrame() {

        // init local submap
        *pointCloudEdgeMap += *currFrame->edgeFeatures;
        *pointCloudSurfMap += *currFrame->surfFeatures;

        *slideWindowEdge += *currFrame->edgeFeatures;
        *slideWindowSurf += *currFrame->surfFeatures;

        // for keyframe, set first frame as keyframe
        setCurrentFrameToLastKeyframe();

        initGTSAM();
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

        cout << "pointCloudEdgeMap size: " << pointCloudEdgeMap->size() << " pointCloudSurfMap size: " << pointCloudSurfMap->size() << endl
             << "slideWindowEdge size: " << slideWindowEdge->size() << " slideWindowSurf size: " << slideWindowSurf->size() << endl
             << "edgeFeaturesDS size: " << edgeFeaturesDS->size() << " surfFeaturesDS size: " << surfFeaturesDS->size() << endl;

        is_keyframe_next = nextFrameToBeKeyframe();

        // translation to local submap -> borrow from f-loam
        getTransToSubMap();
//        getTransToSubMapGTSAM(edgeFeaturesDS, surfFeaturesDS);
        addToSubMap();
//        addToSubMapGTSAM(edgeFeaturesDS, surfFeaturesDS);


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

//        addToSubMapGTSAM(edgeFeaturesDS, surfFeaturesDS);
//        cout << "ceres:\n" << T_curr2world.matrix() << endl;
//        cout << "gtsam:\n" << odom.matrix() << endl;

//        if (Eigen::Isometry3d(T_curr2world.matrix()).translation().norm() < 10) {
//            pW1.write(T_curr2world, false);
//            pW2.write(odom, false);
//        } else {
//            pW1.close();
//            pW2.close();
//        }


        // translation to slide window
        if (!currFrame->is_keyframe()) {

           getTransToSlideWindow(3);
           addToLastKeyframe();

        } else {

           // not downsample, opti_counter twice
           getTransToSlideWindow(6);
           // not add to lastKeyframe's submap
           setCurrentFrameToLastKeyframe();

//           // last keyframe is available
//           int frameFrom = keyframeVec[keyframeVec.size() - 3];
//           int frameTo   = keyframeVec[keyframeVec.size() - 2];
//           addOdomFactorBetweenFrames(frameFrom, frameTo);
//
//           getTransBetweenKeyframes(frameFrom, frameTo);


        }
        cout << "submap:\n" << T_curr2world.matrix() << endl;
        cout << "slide window:\n" << T_curr2worldBySW.matrix() << endl;


        pubOdomAndPath();
        pubMapSubMap();

    }

    void pubMapSlideWindow() {

        pcl::PointCloud<PointT>::Ptr slideWindow(new pcl::PointCloud<PointT>());
        sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2());

        *slideWindow += *slideWindowEdge;
        *slideWindow += *slideWindowSurf;

        pcl::toROSMsg(*slideWindow, *cloud_msg);
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

        odometry_submap = poseToNavOdometry(cloud_in_time, T_curr2world, "map", "base_link");

        odometry_slide_window = poseToNavOdometry(cloud_in_time, T_curr2worldBySW, "map", "base_link");

        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(T_curr2world.translation().x(), T_curr2world.translation().y(), T_curr2world.translation().z()) );
        Eigen::Quaterniond q_current(T_curr2world.rotation());
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

    struct EdgeFeatures {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Vector3d curr_point;
        Eigen::Vector3d point_a;
        Eigen::Vector3d point_b;

        EdgeFeatures(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3)
            : curr_point(std::move(p1)), point_a(std::move(p2)), point_b(std::move(p3)) {

        }
    };

    struct PlaneFeatures {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Vector3d curr_point;
        Eigen::Vector3d norm;
        double negative_OA_dot_norm;

        PlaneFeatures(Eigen::Vector3d p1, Eigen::Vector3d norm_, double nor)
        : curr_point(std::move(p1)), norm(std::move(norm_)), negative_OA_dot_norm(nor) {

        }
    };

//    void addEdgeCostFactorToSubMap(
//            const pcl::PointCloud<PointT>::Ptr& pc_in,
//            const pcl::PointCloud<PointT>::Ptr& map_in,
//            ceres::Problem& problem,
//            ceres::LossFunction *loss_function) {
    void addEdgeCostFactorToSubMap(
            const pcl::PointCloud<PointT>::Ptr& pc_in,
            const pcl::PointCloud<PointT>::Ptr& map_in,
            ceres::Problem& problem,
            ceres::LossFunction *loss_function,
            gtsam::NonlinearFactorGraph& factors,
            const std::string& file_path) {
        int corner_num = 0;
        PointT point_temp;
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
//    std::ofstream f(file_path + "edge_correspondings.txt");
//    vector<EdgeFeatures> features;

        for (int i = 0; i < (int) pc_in->points.size(); i++) {

            pointAssociateToMap(&(pc_in->points[i]), &point_temp);
            kdtreeEdgeMap->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis);

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

//            features.emplace_back(curr_point, point_a, point_b);
                    ceres::CostFunction *cost_function = new EdgeAnalyticCostFunction(curr_point, point_a, point_b);
                    problem.AddResidualBlock(cost_function, loss_function, parameter);

//                    factors.emplace_shared<gtsam::PointToEdgeFactor>(
//                            X(0), curr_point, point_a, point_b, edge_noise_model);
                    corner_num++;
                }
            }
        }
//        for (auto& feature : features) {
//            f << feature.curr_point.x() << " " << feature.curr_point.y()  << " " << feature.curr_point.z() << " "
//                << feature.point_a.x() << " " << feature.point_a.y()  << " " << feature.point_a.z() << " "
//                << feature.point_b.x() << " " << feature.point_b.y()  << " " << feature.point_b.z() << endl;
//        }
//        f.close();
        if (corner_num < 20) {
            printf("not enough correct points\n");
        }

    }

//    void addSurfCostFactorToSubMap(
//            const pcl::PointCloud<PointT>::Ptr& pc_in,
//            const pcl::PointCloud<PointT>::Ptr& map_in,
//            ceres::Problem& problem,
//            ceres::LossFunction *loss_function) {
    void addSurfCostFactorToSubMap(
            const pcl::PointCloud<PointT>::Ptr& pc_in,
            const pcl::PointCloud<PointT>::Ptr& map_in,
            ceres::Problem& problem,
            ceres::LossFunction *loss_function,
            gtsam::NonlinearFactorGraph& factors,
            const string& file_path) {

        int surf_num=0;
        PointT point_temp;
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
//    std::ofstream f(file_path + "plane_correspondings.txt");
//    vector<PlaneFeatures> features;

        for (int i = 0; i < (int)pc_in->points.size(); i++) {

            pointAssociateToMap(&(pc_in->points[i]), &point_temp);
            kdtreeSurfMap->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis);

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
                    ceres::CostFunction *cost_function = new SurfNormAnalyticCostFunction(curr_point, norm, negative_OA_dot_norm);
                    problem.AddResidualBlock(cost_function, loss_function, parameter);
//                    factors.emplace_shared<gtsam::PointToPlaneFactor>(
//                            X(0), curr_point, norm, negative_OA_dot_norm, surf_noise_model);
//                features.emplace_back(curr_point, norm, negative_OA_dot_norm);
                    surf_num++;
                }
            }

        }
//        for (auto& feature : features) {
//            f << feature.curr_point.x() << " " << feature.curr_point.y()  << " " << feature.curr_point.z() << " "
//              << feature.norm.x() << " " << feature.norm.y() << " " << feature.norm.z() << " " << feature.negative_OA_dot_norm << endl;
//        }
//        f.close();
        if (surf_num < 20) {
            printf("not enough correct points\n");
        }

    }

    void addEdgeCostFactorToSlideWindow(const pcl::PointCloud<PointT>::Ptr& pc_in, const pcl::PointCloud<PointT>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function) {
        int corner_num=0;
        PointT point_temp;
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        for (int i = 0; i < (int)pc_in->points.size(); i++) {

            pointAssociateToSlideWindow(&(pc_in->points[i]), &point_temp);
            kdtreeEdgeSlideWindow->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis);
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

                    ceres::CostFunction *cost_function = new EdgeAnalyticCostFunction(curr_point, point_a, point_b);
                    problem.AddResidualBlock(cost_function, loss_function, parameter2);
                    corner_num++;
                }
            }
        }
        if(corner_num<20){
            printf("not enough correct points\n");
        }

    }

    void addSurfCostFactorToSlideWindow(const pcl::PointCloud<PointT>::Ptr& pc_in, const pcl::PointCloud<PointT>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function) {
        int surf_num=0;
        PointT point_temp;
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        for (int i = 0; i < (int)pc_in->points.size(); i++) {

            pointAssociateToSlideWindow(&(pc_in->points[i]), &point_temp);
            kdtreeSurfSlideWindow->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis);

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
                    ceres::CostFunction *cost_function = new SurfNormAnalyticCostFunction(curr_point, norm, negative_OA_dot_norm);
                    problem.AddResidualBlock(cost_function, loss_function, parameter2);

                    surf_num++;
                }
            }

        }
        if(surf_num<20){
            printf("not enough correct points\n");
        }

    }

    void initGTSAM() {
        auto priorNoise = noiseModel::Diagonal::Sigmas((Vector(6) << 1e-8, 1e-8, 1e-8, 1e-8, 1e-8, 1e-8).finished()); // rad*rad, meter*meter
        gtSAMgraph.add(PriorFactor<Pose3>(0, trans2gtsamPose3(parameter2), priorNoise));
        initialEstimate.insert(0, trans2gtsamPose3(parameter2));
        cout << "initGTSAM" << endl;
    }

    void addOdomFactorBetweenFrames(int frameFrom, int frameTo) {
        auto odometryNoise = noiseModel::Diagonal::Sigmas((Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished());
        gtsam::Pose3 poseFrom = trans2gtsamPose3(frameMap[frameFrom]->pose);
        gtsam::Pose3 poseTo   = trans2gtsamPose3(frameMap[frameTo]  ->pose);
        gtSAMgraph.add(BetweenFactor<Pose3>(frameFrom, frameTo, poseFrom.between(poseTo), odometryNoise));
        initialEstimate.insert(frameTo, poseTo);

        saveOdomGraph();
    }

    void getTransToSubMap() {

//        gtsam::Pose3 odom_prediction = odom * (last_odom.inverse() * odom);
//        last_odom = odom;
//        odom = odom_prediction;
//        odom.print();
//
//        // 'pose_w_c' to be optimize
//        pose_w_c = odom;
//
//        const auto state_key = X(0);

        auto currEdge = currFrame->edgeFeatures;
        auto currSurf = currFrame->surfFeatures;

        Eigen::Isometry3d odom_prediction = T_curr2world * (T_last.inverse() * T_curr2world);
        T_last = T_curr2world;
        T_curr2world = odom_prediction;

//        cout << "ceres_pred:\n" << odom_prediction.matrix() << endl;

        q_curr2world = Eigen::Quaterniond(T_curr2world.rotation());
        t_curr2world = T_curr2world.translation();

//        pose_w_c = gtsam::Pose3(T_curr2world.matrix());

        if (pointCloudEdgeMap->size() > 10 && pointCloudSurfMap->size() > 50) {

            kdtreeEdgeMap->setInputCloud(pointCloudEdgeMap);
            kdtreeSurfMap->setInputCloud(pointCloudSurfMap);

            for (int opti_counter = 0; opti_counter < 3; opti_counter++) {

                gtsam::NonlinearFactorGraph factors;
//                gtsam::Values init_values;
//
//                // 'pose_w_c' to be optimize
//                init_values.insert(state_key, pose_w_c);

                ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                ceres::Problem::Options problem_options;
                ceres::Problem problem(problem_options);
                problem.AddParameterBlock(parameter, 7, new PoseSE3Parameterization());
//            string file_path("/home/ziv/debuging/" + std::to_string(frameCount) + "/" + std::to_string(opti_counter) + "/");
//            string tmp_result_filename = file_path + "tmp_result.txt";
//            _mkdir(tmp_result_filename);

                addEdgeCostFactorToSubMap(currEdge, pointCloudEdgeMap, problem, loss_function, factors, "file_path");
                addSurfCostFactorToSubMap(currSurf, pointCloudSurfMap, problem, loss_function, factors, "file_path");

//                addEdgeCostFactorToSubMap(currEdge, pointCloudEdgeMap, problem, loss_function);
//                addSurfCostFactorToSubMap(currSurf, pointCloudSurfMap, problem, loss_function);

                ceres::Solver::Options options;
                options.linear_solver_type = ceres::DENSE_QR;
                options.max_num_iterations = 4;
                options.minimizer_progress_to_stdout = false;
                options.check_gradients = false;
                options.gradient_check_relative_precision = 1e-4;
                ceres::Solver::Summary summary;

                ceres::Solve(options, &problem, &summary);

//                // gtsam
//                gtsam::LevenbergMarquardtParams params;
//                params.setLinearSolverType("MULTIFRONTAL_CHOLESKY");
//                params.setRelativeErrorTol(1e-4);
//                params.maxIterations = 10;
//
//                // solve
//                auto result = gtsam::LevenbergMarquardtOptimizer(factors, init_values, params).optimize();
//
//                // write result
//                pose_w_c = result.at<gtsam::Pose3>(state_key);

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
        //gtsam
//        odom = pose_w_c;
        //ceres
        T_curr2world.linear() = q_curr2world.toRotationMatrix();
        T_curr2world.translation() = t_curr2world;
    }

    void getTransToSlideWindow(int opti_time = 3) {

        auto currEdge = currFrame->edgeFeatures;
        auto currSurf = currFrame->surfFeatures;

        updateSlideWindows();

        if (slideWindowEdge == nullptr || slideWindowSurf == nullptr) {
            printf("slidewindow error\n");
            return;
        }

        if (slideWindowEdge->size() > 10 && slideWindowSurf->size() > 50) {

            kdtreeEdgeSlideWindow->setInputCloud(slideWindowEdge);
            kdtreeSurfSlideWindow->setInputCloud(slideWindowSurf);

            for (int opti_counter = 0; opti_counter < opti_time; opti_counter++) {
                ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                ceres::Problem::Options problem_options;
                ceres::Problem problem(problem_options);
                problem.AddParameterBlock(parameter2, 7, new PoseSE3Parameterization());
                addEdgeCostFactorToSlideWindow(currEdge, slideWindowEdge, problem, loss_function);
                addSurfCostFactorToSlideWindow(currSurf, slideWindowSurf, problem, loss_function);

                ceres::Solver::Options options;
                options.linear_solver_type = ceres::DENSE_QR;
                options.max_num_iterations = 4;
                options.minimizer_progress_to_stdout = false;
                options.check_gradients = false;
                options.gradient_check_relative_precision = 1e-4;
                ceres::Solver::Summary summary;

                ceres::Solve(options, &problem, &summary);
            }
        } else {
            printf("not enough points in slide window to associate, error\n");
        }
        T_curr2worldBySW.linear() = q_curr2keyframe.toRotationMatrix();
        T_curr2worldBySW.translation() = t_curr2keyframe;
    }

    gtsam::Pose3 getTransBetweenKeyframes(int keyframeFrom, int keyframeTo) {

        gtsam::Pose3 poseBetween(gtsam::Pose3::identity());

        auto fromFrame = frameMap[keyframeFrom];
        auto toFrame   = frameMap[keyframeTo];

        auto edgeFrom  = fromFrame->getEdgeSubMap();
        auto surfFrom  = fromFrame->getSurfSubMap();
//        auto poseFrom  = fromFrame->pose;

        auto edgeTo    = toFrame->getEdgeSubMap();
        auto surfTo    = toFrame->getSurfSubMap();
//        auto toFrom    = toFrame->pose;

        if (edgeFrom == nullptr || surfFrom == nullptr || edgeTo == nullptr || surfTo == nullptr) {
            printf("getTransBetweenKeyframes error: empty map\n");
            return poseBetween;
        }

        if (edgeFrom->size() > 10 && edgeTo->size() > 10 && surfFrom->size() > 50 && surfTo->size() > 50) {

            kdtreeEdgeOpti->setInputCloud(edgeFrom);
            kdtreeSurfOpti->setInputCloud(surfFrom);

            for (int opti_counter = 0; opti_counter < 3; opti_counter++) {
                ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                ceres::Problem::Options problem_options;
                ceres::Problem problem(problem_options);
                problem.AddParameterBlock(parameter2, 7, new PoseSE3Parameterization());
                addEdgeCostFactorToSlideWindow(edgeTo, edgeFrom, problem, loss_function);
                addSurfCostFactorToSlideWindow(surfTo, surfFrom, problem, loss_function);

                ceres::Solver::Options options;
                options.linear_solver_type = ceres::DENSE_QR;
                options.max_num_iterations = 4;
                options.minimizer_progress_to_stdout = false;
                options.check_gradients = false;
                options.gradient_check_relative_precision = 1e-4;
                ceres::Solver::Summary summary;

                ceres::Solve(options, &problem, &summary);
            }
        } else {
            printf("not enough points in slide window to associate, error\n");
        }

    }

//    void getTransToKeyframeStrong(const pcl::PointCloud<PointT>::Ptr& currEdge, const pcl::PointCloud<PointT>::Ptr& currSurf) {
//
//        auto edgeSubMap = lastKeyframe->getEdgeSubMap();
//        auto surfSubMap = lastKeyframe->getSurfSubMap();
//
//        if (edgeSubMap == nullptr || surfSubMap == nullptr) {
//            printf("lastKeyframe not keyframe, error\n");
//            return;
//        }
//
//        if (edgeSubMap->size() > 10 && surfSubMap->size() > 50) {
//
//            kdtreeEdgeKeyframe->setInputCloud(edgeSubMap);
//            kdtreeSurfKeyframe->setInputCloud(surfSubMap);
//
//            for (int opti_counter = 0; opti_counter < 6; opti_counter++) {
//                ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
//                ceres::Problem::Options problem_options;
//                ceres::Problem problem(problem_options);
//                problem.AddParameterBlock(parameter2, 7, new PoseSE3Parameterization());
//                addEdgeCostFactorToKeyframe(currEdge, edgeSubMap, problem, loss_function);
//                addSurfCostFactorToKeyframe(currSurf, surfSubMap, problem, loss_function);
//
//                ceres::Solver::Options options;
//                options.linear_solver_type = ceres::DENSE_QR;
//                options.max_num_iterations = 4;
//                options.minimizer_progress_to_stdout = false;
//                options.check_gradients = false;
//                options.gradient_check_relative_precision = 1e-4;
//                ceres::Solver::Summary summary;
//
//                ceres::Solve(options, &problem, &summary);
//            }
//        } else {
//            printf("not enough points in keyframe to associate, error\n");
//        }
//        T_curr2worldByKeyframe.linear() = q_curr2keyframe.toRotationMatrix();
//        T_curr2worldByKeyframe.translation() = t_curr2keyframe;
//
//        //addToKeyframe(currEdge, currSurf);
//
//    }

//    void pointAssociateToMapGTSAM(pcl::PointXYZI const *const pi, pcl::PointXYZI *const po) {
//        Eigen::Vector3d point_curr (pi->x, pi->y, pi->z);
//        Eigen::Vector3d point_w = odom * point_curr;
//        po->x = point_w.x();
//        po->y = point_w.y();
//        po->z = point_w.z();
//        po->intensity = pi->intensity;
//        //po->intensity = 1.0;
//    }
//
//    void addToSubMapGTSAM(const pcl::PointCloud<PointT>::Ptr& currEdgeCloud, const pcl::PointCloud<PointT>::Ptr& currSurfCloud) {
//
//        PointT point_temp;
//        for (int i = 0; i < currEdgeCloud->size(); i++) {
//            pointAssociateToMapGTSAM(&currEdgeCloud->points[i], &point_temp);
//            pointCloudEdgeMapGTSAM->push_back(point_temp);
//        }
//
//        for (int i = 0; i < currSurfCloud->size(); i++) {
//            pointAssociateToMapGTSAM(&currSurfCloud->points[i], &point_temp);
//            pointCloudSurfMapGTSAM->push_back(point_temp);
//        }
//
//        double x_min = +T_curr2world.translation().x()-100;
//        double y_min = +T_curr2world.translation().y()-100;
//        double z_min = +T_curr2world.translation().z()-100;
//        double x_max = +T_curr2world.translation().x()+100;
//        double y_max = +T_curr2world.translation().y()+100;
//        double z_max = +T_curr2world.translation().z()+100;
//
//        //ROS_INFO("size : %f,%f,%f,%f,%f,%f", x_min, y_min, z_min,x_max, y_max, z_max);
//        cropBoxFilter.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0));
//        cropBoxFilter.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0));
//        cropBoxFilter.setNegative(false);
//
//        pcl::PointCloud<PointT>::Ptr tmpEdge(new pcl::PointCloud<PointT>());
//        pcl::PointCloud<PointT>::Ptr tmpSurf(new pcl::PointCloud<PointT>());
//        cropBoxFilter.setInputCloud(pointCloudSurfMapGTSAM);
//        cropBoxFilter.filter(*tmpSurf);
//        cropBoxFilter.setInputCloud(pointCloudEdgeMapGTSAM);
//        cropBoxFilter.filter(*tmpEdge);
//
//        downSizeFilterEdge.setInputCloud(tmpSurf);
//        downSizeFilterEdge.filter(*pointCloudSurfMapGTSAM);
//        downSizeFilterSurf.setInputCloud(tmpEdge);
//        downSizeFilterSurf.filter(*pointCloudEdgeMapGTSAM);
//    }
//
//    void addEdgeCostFactorToSubMapGTSAM(
//            const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in,
//            const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_in,
//            gtsam::NonlinearFactorGraph& factors,
//            gtsam::Values& values)
//    {
//        int corner_num=0;
//        for (int i = 0; i < (int)pc_in->points.size(); i++)
//        {
//            pcl::PointXYZI point_temp;
//            pointAssociateToMapGTSAM(&(pc_in->points[i]), &point_temp);
//
//            std::vector<int> pointSearchInd;
//            std::vector<float> pointSearchSqDis;
//            kdtreeEdgeMapGTSAM->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis);
//            if (pointSearchSqDis[4] < 1.0)
//            {
//                std::vector<Eigen::Vector3d> nearCorners;
//                Eigen::Vector3d center(0, 0, 0);
//                for (int j = 0; j < 5; j++)
//                {
//                    Eigen::Vector3d tmp(map_in->points[pointSearchInd[j]].x,
//                                        map_in->points[pointSearchInd[j]].y,
//                                        map_in->points[pointSearchInd[j]].z);
//                    center = center + tmp;
//                    nearCorners.push_back(tmp);
//                }
//                center = center / 5.0;
//
//                Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
//                for (int j = 0; j < 5; j++)
//                {
//                    Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
//                    covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
//                }
//
//                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);
//
//                Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
//                Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);
//                if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]) {
//                    Eigen::Vector3d point_on_line = center;
//                    Eigen::Vector3d point_a, point_b;
//                    point_a = 0.1 * unit_direction + point_on_line;
//                    point_b = -0.1 * unit_direction + point_on_line;
//
//                    factors.emplace_shared<gtsam::PointToEdgeFactor>(
//                            X(0), curr_point, point_a, point_b, edge_noise_model);
//
//                    corner_num++;
//                }
//            }
//        }
//        if(corner_num<20){
//            printf("not enough correct points gtsam\n");
//        }
//
//    }
//
//    void addSurfCostFactorToSubMapGTSAM(
//            const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in,
//            const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_in,
//            gtsam::NonlinearFactorGraph& factors,
//            gtsam::Values& values)
//    {
//        int surf_num=0;
//        for (int i = 0; i < (int)pc_in->points.size(); i++)
//        {
//            pcl::PointXYZI point_temp;
//            pointAssociateToMapGTSAM(&(pc_in->points[i]), &point_temp);
//            std::vector<int> pointSearchInd;
//            std::vector<float> pointSearchSqDis;
//            kdtreeSurfMapGTSAM->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis);
//
//            Eigen::Matrix<double, 5, 3> matA0;
//            Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
//            if (pointSearchSqDis[4] < 1.0)
//            {
//
//                for (int j = 0; j < 5; j++)
//                {
//                    matA0(j, 0) = map_in->points[pointSearchInd[j]].x;
//                    matA0(j, 1) = map_in->points[pointSearchInd[j]].y;
//                    matA0(j, 2) = map_in->points[pointSearchInd[j]].z;
//                }
//                // find the norm of plane
//                Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
//                double negative_OA_dot_norm = 1 / norm.norm();
//                norm.normalize();
//
//                bool planeValid = true;
//                for (int j = 0; j < 5; j++)
//                {
//                    // if OX * n > 0.2, then plane is not fit well
//                    if (fabs(norm(0) * map_in->points[pointSearchInd[j]].x +
//                             norm(1) * map_in->points[pointSearchInd[j]].y +
//                             norm(2) * map_in->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
//                    {
//                        planeValid = false;
//                        break;
//                    }
//                }
//                Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);
//                if (planeValid)
//                {
//                    factors.emplace_shared<gtsam::PointToPlaneFactor>(
//                            X(0), curr_point, norm, negative_OA_dot_norm, surf_noise_model);
//                    surf_num++;
//                }
//            }
//
//        }
//        if(surf_num<20){
//            printf("not enough correct points gtsam\n");
//        }
//
//    }
//
//    void getTransToSubMapGTSAM(const pcl::PointCloud<PointT>::Ptr& currEdge, const pcl::PointCloud<PointT>::Ptr& currSurf) {
//
//        gtsam::Pose3 odom_prediction = odom * (last_odom.inverse() * odom);
//        odom_prediction = pose_normalize(odom_prediction);
//
//        last_odom = odom;
//        odom = odom_prediction;
//
//        // 'pose_w_c' to be optimize
//        pose_w_c = odom;
//
//        const auto state_key = X(0);
//
//        if (pointCloudEdgeMap->size() > 10 && pointCloudSurfMap->size() > 50) {
//
//            kdtreeEdgeMapGTSAM->setInputCloud(pointCloudEdgeMapGTSAM);
//            kdtreeSurfMapGTSAM->setInputCloud(pointCloudSurfMapGTSAM);
//
//            for (int opti_counter = 0; opti_counter < 3; opti_counter++) {
//
//                gtsam::NonlinearFactorGraph factors;
//                gtsam::Values init_values;
//
//                // 'pose_w_c' to be optimize
//                init_values.insert(state_key, pose_w_c);
//
//                addEdgeCostFactorToSubMapGTSAM(currEdge, pointCloudEdgeMapGTSAM, factors, init_values);
//                addSurfCostFactorToSubMapGTSAM(currSurf, pointCloudSurfMapGTSAM, factors, init_values);
//
//                // std::ofstream out_graph("/home/iceytan/floam_graph.dot");
//                // factors.saveGraph(out_graph, init_values);
//
//                gtsam::LevenbergMarquardtParams params;
//                params.setLinearSolverType("MULTIFRONTAL_CHOLESKY");
//                params.setRelativeErrorTol(1e-4);
//                params.maxIterations = 10;
//
//                // solve
//                auto result = gtsam::LevenbergMarquardtOptimizer(factors, init_values, params).optimize();
//
//                // write result
//                pose_w_c = result.at<gtsam::Pose3>(state_key);
//            }
//
//        } else {
//            printf("not enough points in submap to associate, error\n");
//        }
//        odom = pose_w_c;
//    }


    void updateCurrentFrame(const pcl::PointCloud<PointT>::Ptr& cloud_in_edge, const pcl::PointCloud<PointT>::Ptr& cloud_in_surf) {

        if (is_keyframe_next) {
            currFrame = boost::make_shared<Keyframe>(cloud_in_edge, cloud_in_surf);
            is_keyframe_next = false;
        } else {
            currFrame = boost::make_shared<Frame>(cloud_in_edge, cloud_in_surf);
        }

    }

    void gps_ground_truth() {
        boost::optional<Eigen::Vector3d> zero_utm;
        while (ros::ok()) {
            if (!gps_queue.empty()) {

                gps_mBuf.lock();
                auto curr_gps = gps_queue.front();
                gps_queue.pop();
                gps_mBuf.unlock();

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

    void laser_odometry() {

        cloud_in_edge.reset(new pcl::PointCloud<PointT>());
        cloud_in_surf.reset(new pcl::PointCloud<PointT>());
        cloud_in_full.reset(new pcl::PointCloud<PointT>());

        while (ros::ok()) {

            if (!pointCloudFullBuf.empty() && !pointCloudEdgeBuf.empty() && !pointCloudSurfBuf.empty()) {

                Timer t_laser_odometry("laser_odometry");

                mBuf.lock();

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

                mBuf.unlock();

                updateCurrentFrame(cloud_in_edge, cloud_in_surf);

                updateOdomWithFrame();

                currFrame->pose = T_curr2worldBySW;
                frameMap[frameCount++] = currFrame;
                t_laser_odometry.count();
            }

            //sleep 2 ms every time
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    void allocateMem() {
        pointCloudEdgeMap       = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
        pointCloudSurfMap       = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
        slideWindowEdge         = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
        slideWindowSurf         = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
//        pointCloudEdgeMapGTSAM  = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
//        pointCloudSurfMapGTSAM  = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());


        kdtreeEdgeMap           = pcl::KdTreeFLANN<PointT>::Ptr(new pcl::KdTreeFLANN<PointT>());
        kdtreeSurfMap           = pcl::KdTreeFLANN<PointT>::Ptr(new pcl::KdTreeFLANN<PointT>());
        // kdtreeEdgeMapGTSAM      = pcl::KdTreeFLANN<PointT>::Ptr(new pcl::KdTreeFLANN<PointT>());
        // kdtreeSurfMapGTSAM      = pcl::KdTreeFLANN<PointT>::Ptr(new pcl::KdTreeFLANN<PointT>());
        kdtreeEdgeSlideWindow      = pcl::KdTreeFLANN<PointT>::Ptr(new pcl::KdTreeFLANN<PointT>());
        kdtreeSurfSlideWindow      = pcl::KdTreeFLANN<PointT>::Ptr(new pcl::KdTreeFLANN<PointT>());
        kdtreeEdgeKeyframeGTSAM = pcl::KdTreeFLANN<PointT>::Ptr(new pcl::KdTreeFLANN<PointT>());
        kdtreeSurfKeyframeGTSAM = pcl::KdTreeFLANN<PointT>::Ptr(new pcl::KdTreeFLANN<PointT>());
        kdtreeEdgeOpti          = pcl::KdTreeFLANN<PointT>::Ptr(new pcl::KdTreeFLANN<PointT>());
        kdtreeSurfOpti          = pcl::KdTreeFLANN<PointT>::Ptr(new pcl::KdTreeFLANN<PointT>());
    }

    void initROSHandler() {

        sub_laser_cloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points_2", 100, &LaserOdometry::pointCloudFullHandler, this);

        sub_laser_cloud_edge = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_edge", 100, &LaserOdometry::pointCloudEdgeHandler, this);

        sub_laser_cloud_surf = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf", 100, &LaserOdometry::pointCloudSurfHandler, this);

        sub_gps = nh.subscribe<sensor_msgs::NavSatFix>("/airsim_node/drone_1/global_gps", 1000, &LaserOdometry::gpsHandler, this);

        pub_path_submap = nh.advertise<nav_msgs::Path>("/path_submap", 100);

        pub_path_slide_window = nh.advertise<nav_msgs::Path>("/path_slide_window", 100);

        pub_path_gt = nh.advertise<nav_msgs::Path>("/path_gt", 100);

//        pub_transform_stamped = nh.advertise<geometry_msgs::TransformStamped>("/transform_stamped", 100);

        pub_map_slide_window = nh.advertise<sensor_msgs::PointCloud2>("/map_slide_window", 5);

        pub_map_submap = nh.advertise<sensor_msgs::PointCloud2>("/map_submap", 5);

        pub_curr_edge = nh.advertise<sensor_msgs::PointCloud2>("/curr_edge", 10);

        pub_curr_surf = nh.advertise<sensor_msgs::PointCloud2>("/curr_surf", 10);

    }

    void initParam() {

        T_curr2world = T_lastKeyframe = T_curr2worldBySW = T_last = Eigen::Isometry3d::Identity();

        map_resolution = nh.param<double>("map_resolution", 0.2);

        downSizeFilterEdge.setLeafSize(map_resolution, map_resolution, map_resolution);
        downSizeFilterSurf.setLeafSize(map_resolution * 2, map_resolution * 2, map_resolution * 2);

        pose_w_c = gtsam::Pose3::identity();
        odom    = gtsam::Pose3::identity();
        last_odom = gtsam::Pose3::identity();
        last_keyframe = gtsam::Pose3::identity();

        edge_gaussian_model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(3) << 1.0, 1.0, 1.0).finished());
        surf_gaussian_model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(1) << 1.0).finished());

        edge_noise_model = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(0.1), edge_gaussian_model);
        surf_noise_model = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(0.1), surf_gaussian_model);

    }


    LaserOdometry() :   q_curr2world(parameter), t_curr2world(parameter + 4),
                        q_curr2keyframe(parameter2), t_curr2keyframe(parameter2 + 4),
                        pW1("ceres.txt"),
                        pW2("gtsam.txt")
    {

        allocateMem();

        initROSHandler();

        initParam();
    }

    void saveOdomGraph() {
        std::cout << "saveOdomFactor" << std::endl;
        std::ofstream if_graph("/home/ziv/mloam.dot");
        gtSAMgraph.saveGraph(if_graph, initialEstimate);
    }

private:

    int frameCount = 0;

    PoseWriter pW1;
    PoseWriter pW2;

    ros::NodeHandle nh;
    ros::Time cloud_in_time;

    Eigen::Isometry3d T_curr2world;                 // odometry of current frame to world by submap
    Eigen::Isometry3d T_lastKeyframe;               // odometry of last keyframe to world
    Eigen::Isometry3d T_curr2worldBySW;             // odometry of current frame to world of by slide window
    Eigen::Isometry3d T_last;

    double parameter[7] = {0, 0, 0, 1, 0, 0, 0};
    Eigen::Map<Eigen::Quaterniond>  q_curr2world;
    Eigen::Map<Eigen::Vector3d>     t_curr2world;

    double parameter2[7] = {0, 0, 0, 1, 0, 0, 0};
    Eigen::Map<Eigen::Quaterniond>  q_curr2keyframe;
    Eigen::Map<Eigen::Vector3d>     t_curr2keyframe;

    Frame::Ptr currFrame;
    Frame::Ptr lastKeyframe;

    // map of each frame or keyframe
    std::unordered_map<int, Frame::Ptr> frameMap;
    std::vector<int> keyframeVec;

    // queue of ros pointcloud msg
    std::mutex mBuf;
    std::mutex gps_mBuf;
    std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudEdgeBuf;
    std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudSurfBuf;
    std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudFullBuf;
    std::queue<geographic_msgs::GeoPointStampedConstPtr> gps_queue;

    pcl::PointCloud<PointT>::Ptr cloud_in_edge;
    pcl::PointCloud<PointT>::Ptr cloud_in_surf;
    pcl::PointCloud<PointT>::Ptr cloud_in_full;

    // frame-to-submap
    pcl::PointCloud<PointT>::Ptr pointCloudEdgeMap;
    pcl::PointCloud<PointT>::Ptr pointCloudSurfMap;
    // frame-to-slidewindow
    pcl::PointCloud<PointT>::Ptr slideWindowEdge;
    pcl::PointCloud<PointT>::Ptr slideWindowSurf;

    //    pcl::PointCloud<PointT>::Ptr pointCloudEdgeMapGTSAM;
    //    pcl::PointCloud<PointT>::Ptr pointCloudSurfMapGTSAM;

    pcl::KdTreeFLANN<PointT>::Ptr kdtreeEdgeMap;
    pcl::KdTreeFLANN<PointT>::Ptr kdtreeSurfMap;
//    pcl::KdTreeFLANN<PointT>::Ptr kdtreeEdgeMapGTSAM;
//    pcl::KdTreeFLANN<PointT>::Ptr kdtreeSurfMapGTSAM;
    pcl::KdTreeFLANN<PointT>::Ptr kdtreeEdgeSlideWindow;
    pcl::KdTreeFLANN<PointT>::Ptr kdtreeSurfSlideWindow;

    pcl::KdTreeFLANN<PointT>::Ptr kdtreeEdgeKeyframeGTSAM;
    pcl::KdTreeFLANN<PointT>::Ptr kdtreeSurfKeyframeGTSAM;

    pcl::KdTreeFLANN<PointT>::Ptr kdtreeEdgeOpti;
    pcl::KdTreeFLANN<PointT>::Ptr kdtreeSurfOpti;

    // downsample filter
    pcl::VoxelGrid<PointT> downSizeFilterEdge;
    pcl::VoxelGrid<PointT> downSizeFilterSurf;
    //local map
    pcl::CropBox<PointT> cropBoxFilter;

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

    const int SLIDE_KEYFRAME_SUBMAP_LEN = 3;
    const int SLIDE_WINDOW_LEN = 30;

    double keyframeDistThreshold = 0.8;
    double keyframeAngleThreshold = 0.15;



    // gtsam
    NonlinearFactorGraph gtSAMgraph;
    Values initialEstimate;
    Values optimizedEstimate;
    ISAM2 *isam;
    Values isamCurrentEstimate;
    Eigen::MatrixXd poseCovariance;

    gtsam::Pose3 pose_w_c;  // world to current
    gtsam::Pose3 last_odom;
    gtsam::Pose3 odom;
    gtsam::Pose3 last_keyframe;

    gtsam::SharedNoiseModel edge_gaussian_model;
    gtsam::SharedNoiseModel surf_gaussian_model;

    gtsam::SharedNoiseModel edge_noise_model;
    gtsam::SharedNoiseModel surf_noise_model;


};

int main(int argc, char **argv) {

    ros::init(argc, argv, "laserOdometry");

    LaserOdometry laserOdometry;

    std::thread laser_odometry_thread{&LaserOdometry::laser_odometry, &laserOdometry};

    std::thread gps_ground_truth_thread{&LaserOdometry::gps_ground_truth, &laserOdometry};

    ros::spin();

}