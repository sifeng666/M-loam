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

    void pointAssociateToKeyframe(PointT const *const pi, PointT *const po) {
        Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
        Eigen::Vector3d point_w = T_curr2worldByKeyframe.rotation() * point_curr + T_curr2worldByKeyframe.translation();
        po->x = point_w.x();
        po->y = point_w.y();
        po->z = point_w.z();
        po->intensity = pi->intensity;
    }

    void addToLastKeyframe(const pcl::PointCloud<PointT>::Ptr& currEdgeCloud, const pcl::PointCloud<PointT>::Ptr& currSurfCloud) {

        PointT point_temp;
        for (int i = 0; i < currEdgeCloud->size(); i++) {
            pointAssociateToKeyframe(&currEdgeCloud->points[i], &point_temp);
            lastKeyframe->addEdgeFeaturesToSubMap(point_temp);
        }

        for (int i = 0; i < currSurfCloud->size(); i++) {
            pointAssociateToKeyframe(&currSurfCloud->points[i], &point_temp);
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

    void addToSubMap(const pcl::PointCloud<PointT>::Ptr& currEdgeCloud, const pcl::PointCloud<PointT>::Ptr& currSurfCloud) {

        PointT point_temp;
        for (int i = 0; i < currEdgeCloud->size(); i++) {
            pointAssociateToMap(&currEdgeCloud->points[i], &point_temp);
            pointCloudEdgeMap->push_back(point_temp);
        }

        for (int i = 0; i < currSurfCloud->size(); i++) {
            pointAssociateToMap(&currSurfCloud->points[i], &point_temp);
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

    bool nextFrameToBeKeyframe() {
        if (toBeKeyframeInterval < MIN_KEYFRAME_INTERVAL) {
            toBeKeyframeInterval++;
            return false;
        }
        if (toBeKeyframeInterval >= MAX_KEYFRAME_INTERVAL) {
            toBeKeyframeInterval = 0;
            return true;
        }

        Eigen::Isometry3d T_delta = T_lastKeyframe.inverse() * T_curr2worldByKeyframe;

        Eigen::Vector3d eulerAngle = T_delta.rotation().eulerAngles(2, 1, 0);

        bool isKeyframe = min(abs(eulerAngle.x()), abs(abs(eulerAngle.x()) - M_PI)) > keyframeAngleThreshold ||
                          min(abs(eulerAngle.y()), abs(abs(eulerAngle.y()) - M_PI)) > keyframeAngleThreshold ||
                          min(abs(eulerAngle.z()), abs(abs(eulerAngle.z()) - M_PI)) > keyframeAngleThreshold ||
                          T_delta.translation().norm() > keyframeDistThreshold;
        cout << min(abs(eulerAngle.x()), abs(abs(eulerAngle.x()) - M_PI)) << " " << min(abs(eulerAngle.y()), abs(abs(eulerAngle.y()) - M_PI)) << " "
                << min(abs(eulerAngle.z()), abs(abs(eulerAngle.z()) - M_PI)) << " " << T_delta.translation().norm() << endl;
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
        addToLastKeyframe(currFrame->edgeFeatures, currFrame->surfFeatures);
        T_lastKeyframe = T_curr2worldByKeyframe;
        keyframeVec.push_back(frameCount);
    }

    void initWithFirstFrame() {

        // init local submap
        *pointCloudEdgeMap += *currFrame->edgeFeatures;
        *pointCloudSurfMap += *currFrame->surfFeatures;

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
        downSizeFilterEdge.setInputCloud(currFrame->surfFeatures);
        downSizeFilterEdge.filter(*surfFeaturesDS);

        cout << "pointCloudEdgeMap size: "      << pointCloudEdgeMap->size()                << " pointCloudSurfMap size: "      << pointCloudSurfMap->size()                << endl
             << "lastKeyframeEdgeMap size: "    << lastKeyframe->getEdgeSubMap()->size()    << " lastKeyframeSurfMap size: "    << lastKeyframe->getSurfSubMap()->size()    << endl
             << "edgeFeaturesDS size: "         << edgeFeaturesDS->size()                   << " surfFeaturesDS size: "         << surfFeaturesDS->size()                   << endl;

        is_keyframe_next = nextFrameToBeKeyframe();

        // translation to local submap -> borrow from f-loam
        getTransToSubMap(edgeFeaturesDS, surfFeaturesDS);

//        getTransToSubMapGTSAM(edgeFeaturesDS, surfFeaturesDS);
        addToSubMap(edgeFeaturesDS, surfFeaturesDS);


        if (frameCount <= 50 && frameCount >= 1) {
            string local_path = "/home/ziv/debuging/" + std::to_string(frameCount) + "/";
            pcl::io::savePCDFileASCII(local_path + "edge_submap.pcd", *pointCloudEdgeMap);
            pcl::io::savePCDFileASCII(local_path + "plane_submap.pcd", *pointCloudSurfMap);
            pcl::io::savePCDFileASCII(local_path + "edge_feature.pcd", *edgeFeaturesDS);
            pcl::io::savePCDFileASCII(local_path + "plane_feature.pcd", *surfFeaturesDS);

            std::ofstream f(local_path + "result.txt");
            f << "ceres:\n" << T_curr2world.matrix() << endl;
            f << "gtsam:\n" << odom.matrix() << endl;
            f.close();
        }

//        addToSubMapGTSAM(edgeFeaturesDS, surfFeaturesDS);
        cout << "ceres:\n" << T_curr2world.matrix() << endl;
        cout << "gtsam:\n" << odom.matrix() << endl;

        if (Eigen::Isometry3d(odom.matrix()).translation().norm() < 20) {
            pW1.write(T_curr2world, false);
            pW2.write(odom, false);
        } else {
            pW1.close();
            pW2.close();
        }

        // translation to keyframe
//        if (!currFrame->is_keyframe()) {
//
//            getTransToKeyframe(edgeFeaturesDS, surfFeaturesDS);
//            addToLastKeyframe(edgeFeaturesDS, surfFeaturesDS);
//
//        } else {
//
//            // not downsample, opti_counter twice
//            getTransToKeyframeStrong(currFrame->edgeFeatures, currFrame->surfFeatures); // getTransToKeyframeStrong(edgeFeaturesDS, surfFeaturesDS);
//            // not add to lastKeyframe's submap
//            setCurrentFrameToLastKeyframe();
//
////            addOdomFactor();
//            int frameFrom = keyframeVec[keyframeVec.size() - 2];
//            int frameTo   = keyframeVec[keyframeVec.size() - 1];
//            addOdomFactorBetweenFrames(frameFrom, frameTo);
//
//
//        }



        pubOdomAndPath();

    }


    void pubOdomAndPath() {

        // publish odometry
        odometry.header.frame_id = "/base_link";
        odometry.child_frame_id = "/laser_odom";
        odometry.header.stamp = cloud_in_time;
        Eigen::Quaterniond q(T_curr2world.rotation().matrix());
        odometry.pose.pose.orientation.x    = q.x();
        odometry.pose.pose.orientation.y    = q.y();
        odometry.pose.pose.orientation.z    = q.z();
        odometry.pose.pose.orientation.w    = q.w();
        odometry.pose.pose.position.x       = T_curr2world.translation().x();
        odometry.pose.pose.position.y       = T_curr2world.translation().y();
        odometry.pose.pose.position.z       = T_curr2world.translation().z();

        odometry2.header.frame_id = "/base_link";
        odometry2.child_frame_id = "/laser_odom";
        odometry2.header.stamp = cloud_in_time;
        Eigen::Quaterniond q2(odom.rotation().matrix());
        odometry2.pose.pose.orientation.x   = q2.x();
        odometry2.pose.pose.orientation.y   = q2.y();
        odometry2.pose.pose.orientation.z   = q2.z();
        odometry2.pose.pose.orientation.w   = q2.w();
        odometry2.pose.pose.position.x      = odom.translation().x();
        odometry2.pose.pose.position.y      = odom.translation().y();
        odometry2.pose.pose.position.z      = odom.translation().z();

        geometry_msgs::PoseStamped laserPose;
        laserPose.header = odometry.header;
        laserPose.pose = odometry.pose.pose;
        path.header.stamp = odometry.header.stamp;
        path.poses.push_back(laserPose);
        path.header.frame_id = "/base_link";
        pubPath2SubMap.publish(path);

        laserPose.header = odometry2.header;
        laserPose.pose = odometry2.pose.pose;
        path2.header.stamp = odometry2.header.stamp;
        path2.poses.push_back(laserPose);
        path2.header.frame_id = "/base_link";
        pubPath2Keyframe.publish(path2);
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
    std::ofstream f(file_path + "edge_correspondings.txt");
    vector<EdgeFeatures> features;

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

            features.emplace_back(curr_point, point_a, point_b);
                    ceres::CostFunction *cost_function = new EdgeAnalyticCostFunction(curr_point, point_a, point_b);
                    problem.AddResidualBlock(cost_function, loss_function, parameter);

//                    factors.emplace_shared<gtsam::LidarPose3EdgeFactor>(
//                            X(0), curr_point, point_a, point_b, edge_noise_model);

                    factors.emplace_shared<gtsam::PointToEdgeFactor>(
                            X(0), curr_point, point_a, point_b, edge_noise_model);
                    corner_num++;
                }
            }
        }
        for (auto& feature : features) {
            f << feature.curr_point.x() << " " << feature.curr_point.y()  << " " << feature.curr_point.z() << " "
                << feature.point_a.x() << " " << feature.point_a.y()  << " " << feature.point_a.z() << " "
                << feature.point_b.x() << " " << feature.point_b.y()  << " " << feature.point_b.z() << endl;
        }
        f.close();
        if (corner_num < 20) {
            printf("not enough correct points\n");
        }

    }

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
    std::ofstream f(file_path + "plane_correspondings.txt");
    vector<PlaneFeatures> features;

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
                    factors.emplace_shared<gtsam::PointToPlaneFactor>(
                            X(0), curr_point, norm, negative_OA_dot_norm, surf_noise_model);
                features.emplace_back(curr_point, norm, negative_OA_dot_norm);
//                    factors.emplace_shared<gtsam::LidarPose3PlaneNormFactor>(
//                            X(0), curr_point, norm, negative_OA_dot_norm, surf_noise_model);
                    surf_num++;
                }
            }

        }
        for (auto& feature : features) {
            f << feature.curr_point.x() << " " << feature.curr_point.y()  << " " << feature.curr_point.z() << " "
              << feature.norm.x() << " " << feature.norm.y() << " " << feature.norm.z() << " " << feature.negative_OA_dot_norm << endl;
        }
        f.close();
        if (surf_num < 20) {
            printf("not enough correct points\n");
        }

    }

    void addEdgeCostFactorToKeyframe(const pcl::PointCloud<PointT>::Ptr& pc_in, const pcl::PointCloud<PointT>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function) {
        int corner_num=0;
        PointT point_temp;
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        for (int i = 0; i < (int)pc_in->points.size(); i++) {

            pointAssociateToKeyframe(&(pc_in->points[i]), &point_temp);
            kdtreeEdgeKeyframe->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis);
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

    void addSurfCostFactorToKeyframe(const pcl::PointCloud<PointT>::Ptr& pc_in, const pcl::PointCloud<PointT>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function) {
        int surf_num=0;
        PointT point_temp;
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        for (int i = 0; i < (int)pc_in->points.size(); i++) {

            pointAssociateToKeyframe(&(pc_in->points[i]), &point_temp);
            kdtreeSurfKeyframe->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis);

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

    void getTransToSubMap(const pcl::PointCloud<PointT>::Ptr& currEdge, const pcl::PointCloud<PointT>::Ptr& currSurf) {

        gtsam::Pose3 odom_prediction = odom * (last_odom.inverse() * odom);
        last_odom = odom;
        odom = odom_prediction;

        // 'pose_w_c' to be optimize
        pose_w_c = odom;

        const auto state_key = X(0);

        if (pointCloudEdgeMap->size() > 10 && pointCloudSurfMap->size() > 50) {

            kdtreeEdgeMap->setInputCloud(pointCloudEdgeMap);
            kdtreeSurfMap->setInputCloud(pointCloudSurfMap);

            for (int opti_counter = 0; opti_counter < 3; opti_counter++) {

                gtsam::NonlinearFactorGraph factors;
                gtsam::Values init_values;
//
//                // 'pose_w_c' to be optimize
                init_values.insert(state_key, pose_w_c);

                ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                ceres::Problem::Options problem_options;
                ceres::Problem problem(problem_options);
                problem.AddParameterBlock(parameter, 7, new PoseSE3Parameterization());
            string file_path("/home/ziv/debuging/" + std::to_string(frameCount) + "/" + std::to_string(opti_counter) + "/");
                addEdgeCostFactorToSubMap(currEdge, pointCloudEdgeMap, problem, loss_function, factors, file_path);
                addSurfCostFactorToSubMap(currSurf, pointCloudSurfMap, problem, loss_function, factors, file_path);

                ceres::Solver::Options options;
                options.linear_solver_type = ceres::DENSE_QR;
                options.max_num_iterations = 4;
                options.minimizer_progress_to_stdout = false;
                options.check_gradients = false;
                options.gradient_check_relative_precision = 1e-4;
                ceres::Solver::Summary summary;

                ceres::Solve(options, &problem, &summary);

                // gtsam
                gtsam::LevenbergMarquardtParams params;
                params.setLinearSolverType("MULTIFRONTAL_CHOLESKY");
                params.setRelativeErrorTol(1e-4);
                params.maxIterations = 10;

                // solve
                auto result = gtsam::LevenbergMarquardtOptimizer(factors, init_values, params).optimize();

                // write result
                pose_w_c = result.at<gtsam::Pose3>(state_key);
                // gtsam

                std::ofstream f(file_path + "tmp_result.txt");
                Eigen::Isometry3d temp = Eigen::Isometry3d::Identity();
                temp.linear() = q_curr2world.toRotationMatrix();
                temp.translation() = t_curr2world;
                f << "ceres:\n" << temp.matrix() << endl;
                f << "gtsam:\n" << pose_w_c.matrix() << endl;
                f.close();
            }

        } else {
            printf("not enough points in submap to associate, error\n");
        }
        odom = pose_w_c;
        T_curr2world.linear() = q_curr2world.toRotationMatrix();
        T_curr2world.translation() = t_curr2world;
    }

    void getTransToKeyframe(const pcl::PointCloud<PointT>::Ptr& currEdge, const pcl::PointCloud<PointT>::Ptr& currSurf) {

        auto edgeSubMap = lastKeyframe->getEdgeSubMap();
        auto surfSubMap = lastKeyframe->getSurfSubMap();

        if (edgeSubMap == nullptr || surfSubMap == nullptr) {
            printf("lastKeyframe not keyframe, error");
            return;
        }

        if (edgeSubMap->size() > 10 && surfSubMap->size() > 50) {

            kdtreeEdgeKeyframe->setInputCloud(edgeSubMap);
            kdtreeSurfKeyframe->setInputCloud(surfSubMap);

            for (int opti_counter = 0; opti_counter < 3; opti_counter++) {
                ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                ceres::Problem::Options problem_options;
                ceres::Problem problem(problem_options);
                problem.AddParameterBlock(parameter2, 7, new PoseSE3Parameterization());
                addEdgeCostFactorToKeyframe(currEdge, edgeSubMap, problem, loss_function);
                addSurfCostFactorToKeyframe(currSurf, surfSubMap, problem, loss_function);

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
            printf("not enough points in keyframe to associate, error\n");
        }
        T_curr2worldByKeyframe.linear() = q_curr2keyframe.toRotationMatrix();
        T_curr2worldByKeyframe.translation() = t_curr2keyframe;
    }

    void getTransToKeyframeStrong(const pcl::PointCloud<PointT>::Ptr& currEdge, const pcl::PointCloud<PointT>::Ptr& currSurf) {

        auto edgeSubMap = lastKeyframe->getEdgeSubMap();
        auto surfSubMap = lastKeyframe->getSurfSubMap();

        if (edgeSubMap == nullptr || surfSubMap == nullptr) {
            printf("lastKeyframe not keyframe, error\n");
            return;
        }

        if (edgeSubMap->size() > 10 && surfSubMap->size() > 50) {

            kdtreeEdgeKeyframe->setInputCloud(edgeSubMap);
            kdtreeSurfKeyframe->setInputCloud(surfSubMap);

            for (int opti_counter = 0; opti_counter < 6; opti_counter++) {
                ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                ceres::Problem::Options problem_options;
                ceres::Problem problem(problem_options);
                problem.AddParameterBlock(parameter2, 7, new PoseSE3Parameterization());
                addEdgeCostFactorToKeyframe(currEdge, edgeSubMap, problem, loss_function);
                addSurfCostFactorToKeyframe(currSurf, surfSubMap, problem, loss_function);

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
            printf("not enough points in keyframe to associate, error\n");
        }
        T_curr2worldByKeyframe.linear() = q_curr2keyframe.toRotationMatrix();
        T_curr2worldByKeyframe.translation() = t_curr2keyframe;

        //addToKeyframe(currEdge, currSurf);

    }

    void pointAssociateToMapGTSAM(pcl::PointXYZI const *const pi, pcl::PointXYZI *const po) {
        Eigen::Vector3d point_curr (pi->x, pi->y, pi->z);
        Eigen::Vector3d point_w = T_curr2world * point_curr;
        po->x = point_w.x();
        po->y = point_w.y();
        po->z = point_w.z();
        po->intensity = pi->intensity;
        //po->intensity = 1.0;
    }

    void addToSubMapGTSAM(const pcl::PointCloud<PointT>::Ptr& currEdgeCloud, const pcl::PointCloud<PointT>::Ptr& currSurfCloud) {

        PointT point_temp;
        for (int i = 0; i < currEdgeCloud->size(); i++) {
            pointAssociateToMapGTSAM(&currEdgeCloud->points[i], &point_temp);
            pointCloudEdgeMapGTSAM->push_back(point_temp);
        }

        for (int i = 0; i < currSurfCloud->size(); i++) {
            pointAssociateToMapGTSAM(&currSurfCloud->points[i], &point_temp);
            pointCloudSurfMapGTSAM->push_back(point_temp);
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
        cropBoxFilter.setInputCloud(pointCloudSurfMapGTSAM);
        cropBoxFilter.filter(*tmpSurf);
        cropBoxFilter.setInputCloud(pointCloudEdgeMapGTSAM);
        cropBoxFilter.filter(*tmpEdge);

        downSizeFilterEdge.setInputCloud(tmpSurf);
        downSizeFilterEdge.filter(*pointCloudSurfMapGTSAM);
        downSizeFilterSurf.setInputCloud(tmpEdge);
        downSizeFilterSurf.filter(*pointCloudEdgeMapGTSAM);
    }

    void addEdgeCostFactorToSubMapGTSAM(
            const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in,
            const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_in,
            gtsam::NonlinearFactorGraph& factors,
            gtsam::Values& values)
    {
        int corner_num=0;
        for (int i = 0; i < (int)pc_in->points.size(); i++)
        {
            pcl::PointXYZI point_temp;
            pointAssociateToMapGTSAM(&(pc_in->points[i]), &point_temp);

            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;
            kdtreeEdgeMap->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis);
            if (pointSearchSqDis[4] < 1.0)
            {
                std::vector<Eigen::Vector3d> nearCorners;
                Eigen::Vector3d center(0, 0, 0);
                for (int j = 0; j < 5; j++)
                {
                    Eigen::Vector3d tmp(map_in->points[pointSearchInd[j]].x,
                                        map_in->points[pointSearchInd[j]].y,
                                        map_in->points[pointSearchInd[j]].z);
                    center = center + tmp;
                    nearCorners.push_back(tmp);
                }
                center = center / 5.0;

                Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
                for (int j = 0; j < 5; j++)
                {
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
                            X(0), curr_point, point_a, point_b, edge_noise_model);

                    corner_num++;
                }
            }
        }
        if(corner_num<20){
            printf("not enough correct points gtsam\n");
        }

    }

    void addSurfCostFactorToSubMapGTSAM(
            const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in,
            const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_in,
            gtsam::NonlinearFactorGraph& factors,
            gtsam::Values& values)
    {
        int surf_num=0;
        for (int i = 0; i < (int)pc_in->points.size(); i++)
        {
            pcl::PointXYZI point_temp;
            pointAssociateToMapGTSAM(&(pc_in->points[i]), &point_temp);
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;
            kdtreeSurfMap->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis);

            Eigen::Matrix<double, 5, 3> matA0;
            Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
            if (pointSearchSqDis[4] < 1.0)
            {

                for (int j = 0; j < 5; j++)
                {
                    matA0(j, 0) = map_in->points[pointSearchInd[j]].x;
                    matA0(j, 1) = map_in->points[pointSearchInd[j]].y;
                    matA0(j, 2) = map_in->points[pointSearchInd[j]].z;
                }
                // find the norm of plane
                Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
                double negative_OA_dot_norm = 1 / norm.norm();
                norm.normalize();

                bool planeValid = true;
                for (int j = 0; j < 5; j++)
                {
                    // if OX * n > 0.2, then plane is not fit well
                    if (fabs(norm(0) * map_in->points[pointSearchInd[j]].x +
                             norm(1) * map_in->points[pointSearchInd[j]].y +
                             norm(2) * map_in->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
                    {
                        planeValid = false;
                        break;
                    }
                }
                Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);
                if (planeValid)
                {
                    factors.emplace_shared<gtsam::PointToPlaneFactor>(
                            X(0), curr_point, norm, negative_OA_dot_norm, surf_noise_model);
                    surf_num++;
                }
            }

        }
        if(surf_num<20){
            printf("not enough correct points gtsam\n");
        }

    }

    void getTransToSubMapGTSAM(const pcl::PointCloud<PointT>::Ptr& currEdge, const pcl::PointCloud<PointT>::Ptr& currSurf) {

        gtsam::Pose3 odom_prediction = odom * (last_odom.inverse() * odom);
        last_odom = odom;
        odom = odom_prediction;

        // 'pose_w_c' to be optimize
        pose_w_c = odom;

        const auto state_key = X(0);

        if (pointCloudEdgeMap->size() > 10 && pointCloudSurfMap->size() > 50) {

//            kdtreeEdgeMap->setInputCloud(pointCloudEdgeMap);
//            kdtreeSurfMap->setInputCloud(pointCloudSurfMap);

            for (int opti_counter = 0; opti_counter < 3; opti_counter++) {

                gtsam::NonlinearFactorGraph factors;
                gtsam::Values init_values;

                // 'pose_w_c' to be optimize
                init_values.insert(state_key, pose_w_c);

                addEdgeCostFactorToSubMapGTSAM(currEdge, pointCloudEdgeMap, factors, init_values);
                addSurfCostFactorToSubMapGTSAM(currSurf, pointCloudSurfMap, factors, init_values);

                // std::ofstream out_graph("/home/iceytan/floam_graph.dot");
                // factors.saveGraph(out_graph, init_values);

                gtsam::LevenbergMarquardtParams params;
                params.verbosity = gtsam::LevenbergMarquardtParams::Verbosity::SILENT;
                params.maxIterations = 10;

                // solve
                auto result = gtsam::LevenbergMarquardtOptimizer(factors, init_values, params).optimize();

                // write result
                pose_w_c = result.at<gtsam::Pose3>(state_key);
            }

        } else {
            printf("not enough points in submap to associate, error\n");
        }
        odom = pose_w_c;
    }


    void updateCurrentFrame(const pcl::PointCloud<PointT>::Ptr& cloud_in_edge, const pcl::PointCloud<PointT>::Ptr& cloud_in_surf) {
        if (is_keyframe_next) {
            currFrame = boost::make_shared<Keyframe>(cloud_in_edge, cloud_in_surf);
            is_keyframe_next = false;
        } else {
            currFrame = boost::make_shared<Frame>(cloud_in_edge, cloud_in_surf);
        }
    }

    void laser_odometry() {

        pcl::PointCloud<PointT>::Ptr cloud_in_edge(new pcl::PointCloud<PointT>());
        pcl::PointCloud<PointT>::Ptr cloud_in_surf(new pcl::PointCloud<PointT>());
        pcl::PointCloud<PointT>::Ptr cloud_in_full(new pcl::PointCloud<PointT>());

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

                currFrame->pose = T_curr2worldByKeyframe;
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
        pointCloudEdgeMapGTSAM  = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
        pointCloudSurfMapGTSAM  = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());

        kdtreeEdgeMap           = pcl::KdTreeFLANN<PointT>::Ptr(new pcl::KdTreeFLANN<PointT>());
        kdtreeSurfMap           = pcl::KdTreeFLANN<PointT>::Ptr(new pcl::KdTreeFLANN<PointT>());
        kdtreeEdgeKeyframe      = pcl::KdTreeFLANN<PointT>::Ptr(new pcl::KdTreeFLANN<PointT>());
        kdtreeSurfKeyframe      = pcl::KdTreeFLANN<PointT>::Ptr(new pcl::KdTreeFLANN<PointT>());
    }

    void initROSHandler() {

        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points_2", 100, &LaserOdometry::pointCloudFullHandler, this);

        subEdgeLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_edge", 100, &LaserOdometry::pointCloudEdgeHandler, this);

        subSurfLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf", 100, &LaserOdometry::pointCloudSurfHandler, this);

        pubPath2SubMap = nh.advertise<nav_msgs::Path>("/path_to_submap", 100);

        pubPath2Keyframe = nh.advertise<nav_msgs::Path>("/path2_to_keyframe", 100);
    }

    void initParam() {

        T_curr2world = T_lastKeyframe = T_curr2worldByKeyframe = Eigen::Isometry3d::Identity();

        map_resolution = nh.param<double>("map_resolution", 0.2);

        downSizeFilterEdge.setLeafSize(map_resolution, map_resolution, map_resolution);
        downSizeFilterSurf.setLeafSize(map_resolution * 2, map_resolution * 2, map_resolution * 2);

        pose_w_c = gtsam::Pose3::identity();
        odom = gtsam::Pose3::identity();
        last_odom = gtsam::Pose3::identity();

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

    PoseWriter pW1;
    PoseWriter pW2;

    ros::NodeHandle nh;
    ros::Time cloud_in_time;

    Eigen::Isometry3d T_curr2world;                 // odometry of current frame to world by submap
    Eigen::Isometry3d T_lastKeyframe;               // odometry of last keyframe to world
    Eigen::Isometry3d T_curr2worldByKeyframe;       // odometry of current frame to world of by keyframe

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
    std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudEdgeBuf;
    std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudSurfBuf;
    std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudFullBuf;

    pcl::PointCloud<PointT>::Ptr pointCloudEdgeMap;
    pcl::PointCloud<PointT>::Ptr pointCloudSurfMap;

    pcl::PointCloud<PointT>::Ptr pointCloudEdgeMapGTSAM;
    pcl::PointCloud<PointT>::Ptr pointCloudSurfMapGTSAM;

    pcl::KdTreeFLANN<PointT>::Ptr kdtreeEdgeMap;
    pcl::KdTreeFLANN<PointT>::Ptr kdtreeSurfMap;
    pcl::KdTreeFLANN<PointT>::Ptr kdtreeEdgeKeyframe;
    pcl::KdTreeFLANN<PointT>::Ptr kdtreeSurfKeyframe;

    // downsample filter
    pcl::VoxelGrid<PointT> downSizeFilterEdge;
    pcl::VoxelGrid<PointT> downSizeFilterSurf;
    //l ocal map
    pcl::CropBox<PointT> cropBoxFilter;

    ros::Subscriber subLaserCloud, subEdgeLaserCloud, subSurfLaserCloud;
    ros::Publisher pubLaserOdometry, pubLaserOdometry2;

    ros::Publisher pubPath2SubMap;
    ros::Publisher pubPath2Keyframe;

    nav_msgs::Path path;
    nav_msgs::Path path2;
    nav_msgs::Odometry odometry;
    nav_msgs::Odometry odometry2;

    double map_resolution;

    double timePointCloudFull = 0, timePointCloudEdge = 0, timePointCloudSurf = 0;
    bool is_init = false;
    bool is_keyframe_next = true;

    const int MAX_KEYFRAME_INTERVAL = 20;
    const int MIN_KEYFRAME_INTERVAL = 3;
    const int SLIDE_WINDOW_LEN = 10;

    double keyframeDistThreshold = 0.8;
    double keyframeAngleThreshold = 0.15;
    int frameCount = 0;
    int toBeKeyframeInterval = 0;

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

    gtsam::SharedNoiseModel edge_gaussian_model;
    gtsam::SharedNoiseModel surf_gaussian_model;

    gtsam::SharedNoiseModel edge_noise_model;
    gtsam::SharedNoiseModel surf_noise_model;


};

int main(int argc, char **argv) {

    ros::init(argc, argv, "laserOdometry");

    LaserOdometry laserOdometry;

    std::thread laser_odometry_thread{&LaserOdometry::laser_odometry, &laserOdometry};

    ros::spin();

}