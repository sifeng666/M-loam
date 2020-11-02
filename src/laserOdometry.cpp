//
// Created by ziv on 2020/10/26.
//

#include "helper.h"
#include "frame.h"
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

    void initWithFirstFrame() {
        *pointCloudEdgeMap += *currFrame->edgeFeatures;
        *pointCloudSurfMap += *currFrame->surfFeatures;

        currFrame->alloc();
        lastKeyframe = currFrame;
        addToLastKeyframe(currFrame->edgeFeatures, currFrame->surfFeatures);

        keyframeVec.push_back(frameCount);

        initGTSAM();
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

    void addToLastKeyframe(const pcl::PointCloud<PointT>::Ptr& currEdgeDS, const pcl::PointCloud<PointT>::Ptr& currSurfDS) {

        PointT point_temp;
        for (int i = 0; i < currEdgeDS->size(); i++) {
            pointAssociateToKeyframe(&currEdgeDS->points[i], &point_temp);
            lastKeyframe->addEdgeFeaturesToSubMap(point_temp);
        }

        for (int i = 0; i < currSurfDS->size(); i++) {
            pointAssociateToKeyframe(&currSurfDS->points[i], &point_temp);
            lastKeyframe->addSurfFeaturesToSubMap(point_temp);
        }

        downSizeFilterEdge.setInputCloud(lastKeyframe->edgeFeatures);
        downSizeFilterEdge.filter(*lastKeyframe->edgeFeatures);
        downSizeFilterSurf.setInputCloud(lastKeyframe->surfFeatures);
        downSizeFilterSurf.filter(*lastKeyframe->surfFeatures);
    }

    void addToSubMap(const pcl::PointCloud<PointT>::Ptr& currEdgeDS, const pcl::PointCloud<PointT>::Ptr& currSurfDS) {

        PointT point_temp;
        for (int i = 0; i < currEdgeDS->size(); i++) {
            pointAssociateToMap(&currEdgeDS->points[i], &point_temp);
            pointCloudEdgeMap->push_back(point_temp);
        }

        for (int i = 0; i < currSurfDS->size(); i++) {
            pointAssociateToMap(&currSurfDS->points[i], &point_temp);
            pointCloudSurfMap->push_back(point_temp);
        }

        downSizeFilterEdge.setInputCloud(pointCloudEdgeMap);
        downSizeFilterEdge.filter(*pointCloudEdgeMap);
        downSizeFilterSurf.setInputCloud(pointCloudSurfMap);
        downSizeFilterSurf.filter(*pointCloudSurfMap);

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

        pcl::PointCloud<pcl::PointXYZI>::Ptr tmpCorner(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr tmpSurf(new pcl::PointCloud<pcl::PointXYZI>());
        cropBoxFilter.setInputCloud(pointCloudSurfMap);
        cropBoxFilter.filter(*tmpSurf);
        cropBoxFilter.setInputCloud(pointCloudEdgeMap);
        cropBoxFilter.filter(*tmpCorner);

        downSizeFilterEdge.setInputCloud(tmpSurf);
        downSizeFilterEdge.filter(*pointCloudSurfMap);
        downSizeFilterSurf.setInputCloud(tmpCorner);
        downSizeFilterSurf.filter(*pointCloudEdgeMap);

    }

    bool toBeKeyframe() {
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

        is_keyframe_next = toBeKeyframe();

        // translation to local submap -> borrow from f-loam
        getTransToSubMap(edgeFeaturesDS, surfFeaturesDS);

        // translation to keyframe
        if (!currFrame->is_keyframe()) {
            getTransToKeyframe(edgeFeaturesDS, surfFeaturesDS);
        } else {
            // not downsample, opti_counter is more
            getTransToKeyframeStrong(currFrame->edgeFeatures, currFrame->surfFeatures); // getTransToKeyframeStrong(edgeFeaturesDS, surfFeaturesDS);
            // not add to lastKeyframe's submap
            lastKeyframe = currFrame;
            currFrame->alloc();
            addToLastKeyframe(currFrame->edgeFeatures, currFrame->surfFeatures); // addToLastKeyframe(edgeFeaturesDS, surfFeaturesDS);




            addOdomFactor();

            T_lastKeyframe = T_curr2worldByKeyframe;
            keyframeVec.push_back(frameCount);
        }



        pubOdomAndPath();

    }

    void pubOdomAndPath() {
        pW1.write(T_curr2world, false);
        pW2.write(T_curr2worldByKeyframe, false);

        // publish odometry
        odom.header.frame_id = "/base_link";
        odom.child_frame_id = "/laser_odom";
        odom.header.stamp = cloud_in_time;
        Eigen::Quaterniond q(T_curr2world.rotation().matrix());
        odom.pose.pose.orientation.x    = q.x();
        odom.pose.pose.orientation.y    = q.y();
        odom.pose.pose.orientation.z    = q.z();
        odom.pose.pose.orientation.w    = q.w();
        odom.pose.pose.position.x       = T_curr2world.translation().x();
        odom.pose.pose.position.y       = T_curr2world.translation().y();
        odom.pose.pose.position.z       = T_curr2world.translation().z();

        odom2.header.frame_id = "/base_link";
        odom2.child_frame_id = "/laser_odom";
        odom2.header.stamp = cloud_in_time;
        Eigen::Quaterniond q2(T_curr2worldByKeyframe.rotation().matrix());
        odom2.pose.pose.orientation.x   = q2.x();
        odom2.pose.pose.orientation.y   = q2.y();
        odom2.pose.pose.orientation.z   = q2.z();
        odom2.pose.pose.orientation.w   = q2.w();
        odom2.pose.pose.position.x      = T_curr2worldByKeyframe.translation().x();
        odom2.pose.pose.position.y      = T_curr2worldByKeyframe.translation().y();
        odom2.pose.pose.position.z      = T_curr2worldByKeyframe.translation().z();

        geometry_msgs::PoseStamped laserPose;
        laserPose.header = odom.header;
        laserPose.pose = odom.pose.pose;
        path.header.stamp = odom.header.stamp;
        path.poses.push_back(laserPose);
        path.header.frame_id = "/base_link";
        pubPath2SubMap.publish(path);

        laserPose.header = odom2.header;
        laserPose.pose = odom2.pose.pose;
        path2.header.stamp = odom2.header.stamp;
        path2.poses.push_back(laserPose);
        path2.header.frame_id = "/base_link";
        pubPath2Keyframe.publish(path2);
    }

    void addEdgeCostFactorToSubMap(const pcl::PointCloud<PointT>::Ptr& pc_in, const pcl::PointCloud<PointT>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function) {
        int corner_num=0;
        PointT point_temp;
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        for (int i = 0; i < (int)pc_in->points.size(); i++) {

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
                Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z); {
                    Eigen::Vector3d point_on_line = center;
                    Eigen::Vector3d point_a, point_b;
                    point_a = 0.1 * unit_direction + point_on_line;
                    point_b = -0.1 * unit_direction + point_on_line;

                    ceres::CostFunction *cost_function = new EdgeAnalyticCostFunction(curr_point, point_a, point_b);
                    problem.AddResidualBlock(cost_function, loss_function, parameter);
                    corner_num++;
                }
            }
        }
        if(corner_num<20){
            printf("not enough correct points");
        }

    }

    void addSurfCostFactorToSubMap(const pcl::PointCloud<PointT>::Ptr& pc_in, const pcl::PointCloud<PointT>::Ptr& map_in, ceres::Problem& problem, ceres::LossFunction *loss_function) {
        int surf_num=0;
        PointT point_temp;
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

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

                    surf_num++;
                }
            }

        }
        if(surf_num<20){
            printf("not enough correct points");
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
                Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z); {
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
            printf("not enough correct points");
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
            printf("not enough correct points");
        }

    }

    void initGTSAM() {
        auto priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8).finished()); // rad*rad, meter*meter
        gtSAMgraph.add(PriorFactor<Pose3>(0, trans2gtsamPose3(parameter2), priorNoise));
        initialEstimate.insert(0, trans2gtsamPose3(parameter2));
    }

    void addOdomFactor() {
        auto odometryNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
        gtsam::Pose3 poseFrom = trans2gtsamPose3(frameMap[keyframeVec.back()]->pose);
        gtsam::Pose3 poseTo   = trans2gtsamPose3(parameter2);
        gtSAMgraph.add(BetweenFactor<Pose3>(frameCount - 1, frameCount, poseFrom.between(poseTo), odometryNoise));
        initialEstimate.insert(frameCount, poseTo);
    }

    void getTransToSubMap(const pcl::PointCloud<PointT>::Ptr& currEdge, const pcl::PointCloud<PointT>::Ptr& currSurf) {

        if (pointCloudEdgeMap->size() > 10 && pointCloudSurfMap->size() > 50) {

            kdtreeEdgeMap->setInputCloud(pointCloudEdgeMap);
            kdtreeSurfMap->setInputCloud(pointCloudSurfMap);

            for (int opti_counter = 0; opti_counter < 3; opti_counter++) {
                ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                ceres::Problem::Options problem_options;
                ceres::Problem problem(problem_options);
                problem.AddParameterBlock(parameter, 7, new PoseSE3Parameterization());
                addEdgeCostFactorToSubMap(currEdge, pointCloudEdgeMap, problem, loss_function);
                addSurfCostFactorToSubMap(currSurf, pointCloudSurfMap, problem, loss_function);

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
            printf("not enough points in submap to associate, error");
        }

        T_curr2world.linear() = q_curr2world.toRotationMatrix();
        T_curr2world.translation() = t_curr2world;

        addToSubMap(currEdge, currSurf);
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
            printf("not enough points in keyframe to associate, error");
        }
        T_curr2worldByKeyframe.linear() = q_curr2keyframe.toRotationMatrix();
        T_curr2worldByKeyframe.translation() = t_curr2keyframe;

        addToLastKeyframe(currEdge, currSurf);

    }

    void getTransToKeyframeStrong(const pcl::PointCloud<PointT>::Ptr& currEdge, const pcl::PointCloud<PointT>::Ptr& currSurf) {

        auto edgeSubMap = lastKeyframe->getEdgeSubMap();
        auto surfSubMap = lastKeyframe->getSurfSubMap();

        if (edgeSubMap == nullptr || surfSubMap == nullptr) {
            printf("lastKeyframe not keyframe, error");
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
            printf("not enough points in keyframe to associate, error");
        }
        T_curr2worldByKeyframe.linear() = q_curr2keyframe.toRotationMatrix();
        T_curr2worldByKeyframe.translation() = t_curr2keyframe;

        //addToKeyframe(currEdge, currSurf);

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
        pointCloudEdgeMap   = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
        pointCloudSurfMap   = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
        kdtreeEdgeMap       = pcl::KdTreeFLANN<PointT>::Ptr(new pcl::KdTreeFLANN<PointT>());
        kdtreeSurfMap       = pcl::KdTreeFLANN<PointT>::Ptr(new pcl::KdTreeFLANN<PointT>());
        kdtreeEdgeKeyframe  = pcl::KdTreeFLANN<PointT>::Ptr(new pcl::KdTreeFLANN<PointT>());
        kdtreeSurfKeyframe  = pcl::KdTreeFLANN<PointT>::Ptr(new pcl::KdTreeFLANN<PointT>());
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
    }


    LaserOdometry() :   q_curr2world(parameter), t_curr2world(parameter + 4),
                        q_curr2keyframe(parameter2), t_curr2keyframe(parameter2 + 4),
                        pW1("floam.txt"),
                        pW2("aloam.txt")
    {

        allocateMem();

        initROSHandler();

        initParam();
    }

private:

    PoseWriter pW1;
    PoseWriter pW2;

    ros::NodeHandle nh;
    ros::Time cloud_in_time;

    Eigen::Isometry3d T_curr2world;                 // odom of current frame to world by submap
    Eigen::Isometry3d T_lastKeyframe;               // odom of last keyframe to world
    Eigen::Isometry3d T_curr2worldByKeyframe;       // odom of current frame to world of by keyframe

    double parameter[7] = {0, 0, 0, 1, 0, 0, 0};
    Eigen::Map<Eigen::Quaterniond>  q_curr2world;
    Eigen::Map<Eigen::Vector3d>     t_curr2world;

    double parameter2[7] = {0, 0, 0, 1, 0, 0, 0};
    Eigen::Map<Eigen::Quaterniond>  q_curr2keyframe;
    Eigen::Map<Eigen::Vector3d>     t_curr2keyframe;

    Frame::Ptr currFrame;
    Frame::Ptr lastKeyframe;

    std::unordered_map<int, Frame::Ptr> frameMap;
    std::vector<int> keyframeVec;

    std::mutex mBuf;
    std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudEdgeBuf;
    std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudSurfBuf;
    std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudFullBuf;

    pcl::PointCloud<PointT>::Ptr pointCloudEdgeMap;
    pcl::PointCloud<PointT>::Ptr pointCloudSurfMap;

    pcl::KdTreeFLANN<PointT>::Ptr kdtreeEdgeMap;
    pcl::KdTreeFLANN<PointT>::Ptr kdtreeSurfMap;
    pcl::KdTreeFLANN<PointT>::Ptr kdtreeEdgeKeyframe;
    pcl::KdTreeFLANN<PointT>::Ptr kdtreeSurfKeyframe;

    pcl::VoxelGrid<PointT> downSizeFilterEdge;
    pcl::VoxelGrid<PointT> downSizeFilterSurf;
    //local map
    pcl::CropBox<PointT> cropBoxFilter;

    ros::Subscriber subLaserCloud, subEdgeLaserCloud, subSurfLaserCloud;
    ros::Publisher pubLaserOdometry, pubLaserOdometry2;

    ros::Publisher pubPath2SubMap;
    ros::Publisher pubPath2Keyframe;

    nav_msgs::Path path;
    nav_msgs::Path path2;
    nav_msgs::Odometry odom;
    nav_msgs::Odometry odom2;

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

};

int main(int argc, char **argv) {

    ros::init(argc, argv, "laserOdometry");

    LaserOdometry laserOdometry;

    std::thread laser_odometry_thread{&LaserOdometry::laser_odometry, &laserOdometry};

    ros::spin();

}