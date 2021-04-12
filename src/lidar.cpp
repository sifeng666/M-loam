//
// Created by ziv on 2020/12/31.
//

#include "lidar.h"
gtsam::SharedNoiseModel LidarSensor::edge_noise_model;
gtsam::SharedNoiseModel LidarSensor::surf_noise_model;

gtsam::Pose3 pose_normalize(const gtsam::Pose3& pose) {
    return gtsam::Pose3(pose.rotation().normalized(), pose.translation());
}

gtsam::Pose3 toPose3(const Eigen::Quaterniond& q, const Eigen::Vector3d& t) {
    return gtsam::Pose3(gtsam::Rot3(q), t);
}

void LidarMsgReader::pointCloudFullHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
    std::lock_guard lg(pcd_msg_mtx);
    pointCloudFullBuf.push(laserCloudMsg);
}

void LidarMsgReader::pointCloudSurfHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
    std::lock_guard lg(pcd_msg_mtx);
    pointCloudSurfBuf.push(laserCloudMsg);
}

void LidarMsgReader::pointCloudEdgeHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
    std::lock_guard lg(pcd_msg_mtx);
    pointCloudEdgeBuf.push(laserCloudMsg);
}

void LidarMsgReader::pointCloudLessSurfHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
    std::lock_guard lg(pcd_msg_mtx);
    pointCloudLessSurfBuf.push(laserCloudMsg);
}

void LidarMsgReader::pointCloudLessEdgeHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
    std::lock_guard lg(pcd_msg_mtx);
    pointCloudLessEdgeBuf.push(laserCloudMsg);
}

void LidarSensor::addCornCostFactor(const pcl::PointCloud<PointT>::Ptr &pc_in, const pcl::PointCloud<PointT>::Ptr &map_in,
                                    const pcl::KdTreeFLANN<PointT> &kdtree_corn, const gtsam::Pose3 &odom,
                                    gtsam::NonlinearFactorGraph &factors, const gtsam::Pose3 &point_transform) {
    int corner_num = 0;
    PointT point_temp, point_transformed;
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    Eigen::Affine3d odom_(odom.matrix());
    Eigen::Affine3d point_transform_(point_transform.matrix());
    bool need_transform = !point_transform.equals(gtsam::Pose3::identity());

    for (int i = 0; i < (int) pc_in->points.size(); i++) {

        point_temp = pcl::transformPoint(pc_in->points[i], odom_);
        kdtree_corn.nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis);

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
            Eigen::Vector3d curr_point;
            if (need_transform) {
                point_transformed = pcl::transformPoint(pc_in->points[i], point_transform_);
                curr_point = Eigen::Vector3d(point_transformed.x, point_transformed.y, point_transformed.z);
            } else {
                curr_point = Eigen::Vector3d(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);
            }
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

    if (corner_num < 20) {
        printf("not enough correct points\n");
    }
}

void LidarSensor::addSurfCostFactor(const pcl::PointCloud<PointT>::Ptr &pc_in, const pcl::PointCloud<PointT>::Ptr &map_in,
                                    const pcl::KdTreeFLANN<PointT> &kdtree_surf, const gtsam::Pose3 &odom,
                                    gtsam::NonlinearFactorGraph &factors, const gtsam::Pose3 &point_transform) {
    int surf_num=0;
    PointT point_temp, point_transformed;
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    Eigen::Affine3d odom_(odom.matrix());
    Eigen::Affine3d point_transform_(point_transform.matrix());
    bool need_transform = !point_transform.equals(gtsam::Pose3::identity());

    for (int i = 0; i < (int)pc_in->points.size(); i++) {

        point_temp = pcl::transformPoint(pc_in->points[i], odom_);
        kdtree_surf.nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis);

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
            Eigen::Vector3d curr_point;
            if (need_transform) {
                point_transformed = pcl::transformPoint(pc_in->points[i], point_transform_);
                curr_point = Eigen::Vector3d(point_transformed.x, point_transformed.y, point_transformed.z);
            } else {
                curr_point = Eigen::Vector3d(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);
            }
            if (planeValid) {
                factors.emplace_shared<gtsam::PointToPlaneFactor>(
                        X(0), curr_point, norm, negative_OA_dot_norm, surf_noise_model);
                surf_num++;
            }
        }

    }

    if (surf_num < 20) {
        printf("not enough correct points\n");
    }
}

void LidarSensor::updateSubmap(size_t range_from, size_t range_to) {
    submap_corn = MapGenerator::generate_cloud(keyframeVec, range_from, range_to, FeatureType::Corn);
    submap_surf = MapGenerator::generate_cloud(keyframeVec, range_from, range_to, FeatureType::Surf);
    down_sampling_voxel(*submap_corn, corn_filter_length);
    down_sampling_voxel(*submap_surf, surf_filter_length);
}

bool LidarSensor::nextFrameToBeKeyframe() {
    Eigen::Isometry3d T_delta((current_keyframe->pose_world_curr.inverse() * odom).matrix());
    Eigen::Quaterniond q_delta(T_delta.rotation().matrix());
    q_delta.normalize();
    Eigen::Vector3d eulerAngle = q_delta.toRotationMatrix().eulerAngles(2, 1, 0);

    bool isKeyframe = min(abs(eulerAngle.x()), abs(abs(eulerAngle.x()) - M_PI)) > keyframe_angle_thres ||
                      min(abs(eulerAngle.y()), abs(abs(eulerAngle.y()) - M_PI)) > keyframe_angle_thres ||
                      min(abs(eulerAngle.z()), abs(abs(eulerAngle.z()) - M_PI)) > keyframe_angle_thres ||
                      T_delta.translation().norm() > keyframe_distance_thres;
    return isKeyframe;
}

void LidarSensor::scan2scan(pcl::PointCloud<PointT>::Ptr corn, pcl::PointCloud<PointT>::Ptr surf, int n_scans) {

    TicToc t_gen_scans;
    // MapGenerator::generate_cloud gen pcd range from: [begin, end), so need to add 1
    int c_index = current_keyframe->index + 1;

    pcl::PointCloud<PointT>::Ptr corn_scans(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr surf_scans(new pcl::PointCloud<PointT>);

    corn_scans = MapGenerator::generate_cloud(keyframeVec, c_index - n_scans, c_index, FeatureType::Corn);
    surf_scans = MapGenerator::generate_cloud(keyframeVec, c_index - n_scans, c_index, FeatureType::Surf);

    ds_corn.setInputCloud(corn_scans);
    ds_corn.filter(*corn_scans);
    ds_surf.setInputCloud(surf_scans);
    ds_surf.filter(*surf_scans);

    printf("prepare %d scans: %.3f ms", n_scans, t_gen_scans.toc());



}

void LidarSensor::scan2submap(pcl::PointCloud<PointT>::Ptr corn, pcl::PointCloud<PointT>::Ptr surf) {

}

void LidarSensor::handleRegistration() {
    if (!current_keyframe->is_init()) {

        Keyframe::Ptr last_keyframe = keyframeVec->keyframes[current_keyframe->index - 1];

        // push to loopDetectBuf
        BA_mtx.lock();
        BAKeyframeBuf.push(last_keyframe);
        BA_mtx.unlock();

        current_keyframe->set_init(odom);

        if (last_keyframe->index < window_size) {
            updateSubmap();
        }

    } else {
        is_keyframe_next = nextFrameToBeKeyframe();
        current_keyframe->add_frame();
    }
}


void LidarSensor::initParam() {

    submap_corn.reset(new pcl::PointCloud<PointT>());
    submap_surf.reset(new pcl::PointCloud<PointT>());

    ds_corn.setLeafSize(corn_filter_length, corn_filter_length, corn_filter_length);
    ds_surf.setLeafSize(surf_filter_length, surf_filter_length, surf_filter_length);

    opti_counter = 2;

    keyframeVec = std::make_shared<KeyframeVec>();
    keyframeVec->keyframes.reserve(200);
    status = std::make_shared<LidarStatus>();

    pose_w_c = odom = delta = gtsam::Pose3();

    auto edge_gaussian_model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(3) << 0.2, 0.2, 0.2).finished());
    auto surf_gaussian_model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(1) << 0.2).finished());

    edge_noise_model = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(0.58), edge_gaussian_model);
    surf_noise_model = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(0.02), surf_gaussian_model);
    prior_noise_model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished());
    odometry_noise_model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-2, 1e-2, 1e-1).finished());

    gtsam::ISAM2Params isam2Params;
    isam2Params.relinearizeThreshold = 0.1;
    isam2Params.relinearizeSkip = 1;
    isam = std::make_shared<gtsam::ISAM2>(isam2Params);

    initBALMParam();

}

void LidarSensor::update_keyframe(const ros::Time& cloud_in_time,
                                  pcl::PointCloud<PointT>::Ptr corn,
                                  pcl::PointCloud<PointT>::Ptr surf,
                                  pcl::PointCloud<PointT>::Ptr full)
{
    current_keyframe = std::make_shared<Keyframe>(keyframeVec->size(), cloud_in_time, corn, surf, full);
    keyframeVec->emplace_back(current_keyframe);
    is_keyframe_next = false;
}

void LidarSensor::getTransToSubmap(pcl::PointCloud<PointT>::Ptr corn, pcl::PointCloud<PointT>::Ptr surf) {

    odom_pred = pose_normalize(odom * delta);

    last_odom = odom;
    odom = odom_pred;

    // 'pose_w_c' to be optimize
    pose_w_c = odom;

    const auto state_key = X(0);

    submap_mtx.lock();
    auto submapEdge_copy = submap_corn->makeShared();
    auto submapSurf_copy = submap_surf->makeShared();
    submap_mtx.unlock();


    if (submapEdge_copy->size() > 10 && submapSurf_copy->size() > 50) {

        kdtree_corn.setInputCloud(submapEdge_copy);
        kdtree_surf.setInputCloud(submapSurf_copy);

        for (int j = 0; j < opti_counter; j++) {
//            TimeCounter tc; auto t1 = tc.count();
            gtsam::NonlinearFactorGraph factors;
            gtsam::Values init_values;
            init_values.insert(state_key, pose_w_c);

            addCornCostFactor(corn, submapEdge_copy, kdtree_corn, pose_w_c, factors);
//            std::cout << "addEdgeCostFactor: " << j << " " << tc.count() - t1 << "ms" << std::endl; t1 = tc.count();
            addSurfCostFactor(surf, submapSurf_copy, kdtree_surf, pose_w_c, factors);
//            std::cout << "addSurfCostFactor: " << j << " " << tc.count() - t1 << "ms" << std::endl; t1 = tc.count();

            // gtsam
            gtsam::LevenbergMarquardtParams params;
            params.setLinearSolverType("MULTIFRONTAL_CHOLESKY");
            params.setRelativeErrorTol(1e-3);
            params.maxIterations = 4;

            auto result = gtsam::LevenbergMarquardtOptimizer(factors, init_values, params).optimize();
//            std::cout << "gtsam result: " << j << " " << tc.count() - t1 << "ms, total: " << tc.count() << "ms" << std::endl;update
            pose_w_c = result.at<gtsam::Pose3>(state_key);

        }

    } else {
        printf("not enough points in submap to associate, error\n");
    }

    odom = pose_w_c;
    delta = pose_normalize(last_odom.inverse() * odom);

}



gtsam::Pose3 LidarSensor::update(const ros::Time cloud_in_time,
                    pcl::PointCloud<PointT>::Ptr corn,
                    pcl::PointCloud<PointT>::Ptr surf,
                    pcl::PointCloud<PointT>::Ptr raw,
                    pcl::PointCloud<PointT>::Ptr less_corn,
                    pcl::PointCloud<PointT>::Ptr less_surf)
{
    if (is_keyframe_next) {
        update_keyframe(cloud_in_time, less_corn, less_surf, raw);
    }

    if (!is_init) {
        current_keyframe->set_init();
        updateSubmap();
        is_init = true;
        return gtsam::Pose3();
    }

    down_sampling_voxel(*less_corn, corn_filter_length);
    down_sampling_voxel(*less_surf, surf_filter_length);

    scan2scan();

    scan2submap();

    handleRegistration();

    return odom;
}

void LidarSensor::addOdomFactor(int last_index, int curr_index) {

    if (last_index < 0) {
        BAGraph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0), gtsam::Pose3(), prior_noise_model);
        BAEstimate.insert(X(0), gtsam::Pose3());
        std::cout << "init Odom factor!" << std::endl;
        return;
    }

    // read lock
    gtsam::Pose3 last_pose = keyframeVec->read_pose(last_index);
    gtsam::Pose3 curr_pose = keyframeVec->read_pose(curr_index);
    gtsam::Pose3 pose_between = last_pose.between(curr_pose);

    auto information = GetInformationMatrixFromPointClouds(keyframeVec->keyframes[curr_index]->raw,
                                                           keyframeVec->keyframes[last_index]->raw,
                                                           2.0,
                                                           pose_between.matrix());

    BAGraph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(last_index),
                                                            X(curr_index),
                                                            pose_between,
//                                                            odometry_noise_model);
                                                            gtsam::noiseModel::Diagonal::Information(information));
    BAEstimate.insert(X(curr_index), curr_pose);
    printf("add Odom factor between [%d] and [%d]\n", last_index, curr_index);
}

void LidarSensor::loop_detect_thread() {

    int last_loop_found_index = -1;

    while (ros::ok()) {

        std::vector<Keyframe::Ptr> fixedKeyframes_buf;
        {
            std::lock_guard<std::mutex> lg(fixed_mtx);
            while (!FixedKeyframeBuf.empty()) {
                fixedKeyframes_buf.push_back(FixedKeyframeBuf.front());
                FixedKeyframeBuf.pop();
            }
        }
        std::vector<FactorPtr> factors;
        for (const auto& keyframe: fixedKeyframes_buf) {
            if (loopDetector.loop_detector(keyframeVec, keyframe, factors, last_loop_found_index)) {
                std::lock_guard<std::mutex> lg(loop_factors_mtx);
                for (const auto& factor: factors) {
                    factorsBuf.push(factor);
                }
                status->last_loop_found_index = max(status->last_loop_found_index, last_loop_found_index);
            }
        }

        //sleep 5 ms every time
        std::this_thread::sleep_for(std::chrono::milliseconds(backend_sleep));
    }

}

void LidarSensor::BA_optimization() {

    int last_index = -1;
    int current_latency = 0;

    while (ros::ok()) {

        std::vector<Keyframe::Ptr> keyframes_buf;
        {
            std::lock_guard<std::mutex> lg(BA_mtx);
            while (!BAKeyframeBuf.empty()) {
                keyframes_buf.push_back(BAKeyframeBuf.front());
                BAKeyframeBuf.pop();
            }
        }

        if (!keyframes_buf.empty()) {

            std::vector<Keyframe::Ptr> fixedKeyframes_buf;

            // from balm backend
            BALM_backend(keyframes_buf, fixedKeyframes_buf);

            // deal with fixed keyframes with factorGraph
            std::lock_guard<std::mutex> lg(fixed_mtx);
            for (const auto& keyframe: fixedKeyframes_buf) {

                int curr_index = keyframe->index;
                addOdomFactor(last_index, curr_index);
                last_index = curr_index;

                if (need_loop) {
                    FixedKeyframeBuf.push(keyframe);
                }
            }

            // isam update
            isam->update(BAGraph, BAEstimate);
            isam->update();
            BAGraph.resize(0);
            BAEstimate.clear();
            {
                std::lock_guard<std::mutex> lg(loop_factors_mtx);
                if (!factorsBuf.empty()) {
                    while (!factorsBuf.empty()) {
                        BAGraph.add(factorsBuf.front());
                        factorsBuf.pop();
                    }
                    isam->update(BAGraph, BAEstimate);
                    isam->update();
                    isam->update();
                    BAGraph.resize(0);
                    BAEstimate.clear();
                    status->status_change = true;
                }
            }

            isamOptimize = isam->calculateEstimate();
            isam->getFactorsUnsafe().saveGraph("/home/ziv/mloam/factor_graph.dot", isamOptimize);


            {   // write lock
                std::unique_lock<std::shared_mutex> ul(keyframeVec->pose_mtx);
                for (size_t i = 0; i < isamOptimize.size(); i++) {
                    keyframeVec->keyframes[i]->set_fixed(isamOptimize.at<gtsam::Pose3>(X(i)));
                }
            }

            // debug
            auto poseVec = keyframeVec->read_poses(0, keyframeVec->size(), true);
            {
                std::ofstream f(nh.param<std::string>("file_save_path", "") + "without_loop.txt");
                for (size_t i = 0; i < poseVec.size(); i++) {
                    f << "i: " << i << "\n" << poseVec[i].matrix() << endl;
                }
                f.close();
            }

            current_latency = 0; // if there is new keyframe, clear end signal
        } else {
            current_latency += backend_sleep;
            if (last_index > 0 && current_latency >= end_signal && current_latency < 5 * end_signal && !status->is_end) {
                status->is_end = true;
            }
        }

        //sleep 5 ms every time
        std::this_thread::sleep_for(std::chrono::milliseconds(backend_sleep));
    }
}

LidarSensor::LidarSensor()
{
    nh.param<bool>  ("need_loop", need_loop, true);
    nh.param<double>("keyframe_distance_threshold", keyframe_distance_thres, 0.6);
    nh.param<double>("keyframe_angle_threshold",    keyframe_angle_thres,    0.1);
    nh.param<double>("surf_filter_length", surf_filter_length, 0.4);
    nh.param<double>("corn_filter_length", corn_filter_length, 0.2);

    initParam();
}

