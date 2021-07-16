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
    //the pcl::transformPoint in PCL1.8 use affine3f, so here should be changed
    //Eigen::Affine3d odom_(odom.matrix());
    //Eigen::Affine3d point_transform_(point_transform.matrix());
    Eigen::Affine3f odom_(odom.matrix().cast<float>());
    Eigen::Affine3f point_transform_(point_transform.matrix().cast<float>());

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
    //the pcl::transformPoint in PCL1.8 use affine3f, so here should be changed
    //Eigen::Affine3d odom_(odom.matrix());
    //Eigen::Affine3d point_transform_(point_transform.matrix());
    Eigen::Affine3f odom_(odom.matrix().cast<float>());
    Eigen::Affine3f point_transform_(point_transform.matrix().cast<float>());

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

bool LidarSensor::nextFrameToBeKeyframe() {
    Eigen::Isometry3d T_delta(pose_normalize(T_wmap_lastkey.between(T_wmap_curr)).matrix());
    Eigen::Quaterniond q_delta(T_delta.rotation().matrix());
    q_delta.normalize();
    Eigen::Vector3d eulerAngle = q_delta.toRotationMatrix().eulerAngles(2, 1, 0);

    bool isKeyframe = min(abs(eulerAngle.x()), abs(abs(eulerAngle.x()) - M_PI)) > keyframe_angle_thres ||
                      min(abs(eulerAngle.y()), abs(abs(eulerAngle.y()) - M_PI)) > keyframe_angle_thres ||
                      min(abs(eulerAngle.z()), abs(abs(eulerAngle.z()) - M_PI)) > keyframe_angle_thres ||
                      T_delta.translation().norm() > keyframe_distance_thres;
    return isKeyframe;
}

void LidarSensor::feature_based_scan_matching(const pcl::PointCloud<PointT>::Ptr& corn, const pcl::PointCloud<PointT>::Ptr& surf,
                                              const pcl::PointCloud<PointT>::Ptr& corns, const pcl::PointCloud<PointT>::Ptr& surfs,
                                              pcl::KdTreeFLANN<PointT>& kdtree_corn, pcl::KdTreeFLANN<PointT>& kdtree_surf,
                                              gtsam::Pose3& T_w_c) {
    kdtree_corn.setInputCloud(corns);
    kdtree_surf.setInputCloud(surfs);

    for (int j = 0; j < opti_counter; j++) {
        gtsam::NonlinearFactorGraph factors;
        gtsam::Values init_values;
        init_values.insert(X(0), T_w_c);

        addCornCostFactor(corn, corns, kdtree_corn, T_w_c, factors);
        addSurfCostFactor(surf, surfs, kdtree_surf, T_w_c, factors);

        gtsam::LevenbergMarquardtParams params;
        params.setLinearSolverType("MULTIFRONTAL_CHOLESKY");
        params.setRelativeErrorTol(1e-3);
        params.maxIterations = 4;

        auto result = gtsam::LevenbergMarquardtOptimizer(factors, init_values, params).optimize();
        T_w_c = result.at<gtsam::Pose3>(X(0));
    }
}

void LidarSensor::point_wise_scan_matching(const pcl::PointCloud<PointT>::Ptr& corn, const pcl::PointCloud<PointT>::Ptr& surf,
                                           const pcl::PointCloud<PointT>::Ptr& corns, const pcl::PointCloud<PointT>::Ptr& surfs,
                                           gtsam::Pose3& T_w_c) {
    Eigen::Matrix4d final;

    pcl::PointCloud<PointT>::Ptr submap_pcd(new pcl::PointCloud<PointT>);
    *submap_pcd += *submap_corn;
    *submap_pcd += *submap_surf;

    pcl::PointCloud<PointT>::Ptr curr_pcd(new pcl::PointCloud<PointT>);
    *curr_pcd += *corn;
    *curr_pcd += *surf;

    bool ok = tools::FastGeneralizedRegistration(curr_pcd, submap_pcd, final, T_w_c.matrix(), 1.0, 1.0);
    if (ok) {
        T_w_c = gtsam::Pose3(final);
    } else {
        std::cout << "GICP fail to match! Use feature-based method!" << std::endl;
        pcl::KdTreeFLANN<PointT> kdtree_corn_submap;
        pcl::KdTreeFLANN<PointT> kdtree_surf_submap;
        feature_based_scan_matching(corn, surf, submap_corn, submap_surf,
                                    kdtree_corn_submap, kdtree_surf_submap, T_w_c);
    }
}

gtsam::Pose3 LidarSensor::scan2scan(const ros::Time& cloud_in_time, const pcl::PointCloud<PointT>::Ptr& corn,
                                    const pcl::PointCloud<PointT>::Ptr& surf, const pcl::PointCloud<PointT>::Ptr& raw) {

    auto T_w_c = pose_normalize(T_wodom_curr * T_last_curr);

    if (!corn || !surf) {
        std::cerr << "Error nullptr corn or surf" << std::endl;
        return T_w_c;
    }

    TicToc t_gen_scans;

    pcl::PointCloud<PointT>::Ptr corn_scans(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr surf_scans(new pcl::PointCloud<PointT>);

    nscans_gen.read(corn_scans, surf_scans);

    ds_corn.setInputCloud(corn_scans);
    ds_corn.filter(*corn_scans);
    ds_surf.setInputCloud(surf_scans);
    ds_surf.filter(*surf_scans);

//    printf("prepare %d scans: %.3f ms\n", n_scans, t_gen_scans.toc());

    if (corn_scans->size() < 20 || surf_scans->size() < 100) {
        printf("too less features! corn = [%d], surf = [%d], exit!\n", int(corn_scans->size()), int(surf_scans->size()));
        return T_w_c;
    }

    TicToc t_odom;

    feature_based_scan_matching(corn, surf, corn_scans, surf_scans,
                                kdtree_corn_nscans, kdtree_surf_nscans, T_w_c);

    T_wodom_last = T_wodom_curr;
    T_wodom_curr = T_w_c;
    T_last_curr = pose_normalize(T_wodom_last.between(T_wodom_curr));
    nscans_gen.add(std::make_shared<Frame>(frameCount++, cloud_in_time, corn, surf, raw, T_wodom_curr));

//    printf("odom, scan to %d scans: %.3f ms\n", n_scans, t_odom.toc());
    return T_w_c;
}

gtsam::Pose3 LidarSensor::scan2submap(const pcl::PointCloud<PointT>::Ptr& corn, const pcl::PointCloud<PointT>::Ptr& surf,
                                      const gtsam::Pose3& guess, int method) {

    auto T_w_c = guess; // T_w_mapcurr

    if (!corn || !surf) {
        std::cerr << "Error nullptr corn or surf" << std::endl;
        return T_w_c;
    }

    TicToc t_gen_submap;

    // MapGenerator::generate_cloud gen pcd range from: [begin, end), so need to add 1
    int c_index = current_keyframe->index + 1;

    submap_corn = MapGenerator::generate_cloud(keyframeVec, c_index - submap_len, c_index, FeatureType::Corn);
    submap_surf = MapGenerator::generate_cloud(keyframeVec, c_index - submap_len, c_index, FeatureType::Surf);

//    printf("prepare submap: %.3f ms\n", t_gen_submap.toc());

    ds_corn_submap.setInputCloud(submap_corn);
    ds_surf_submap.setInputCloud(submap_surf);
    ds_corn_submap.filter(*submap_corn);
    ds_surf_submap.filter(*submap_surf);

    TicToc t_mapping;
    if (method == 0) {          // feature-based
        feature_based_scan_matching(corn, surf, submap_corn, submap_surf,
                                    kdtree_corn_submap, kdtree_surf_submap, T_w_c);
    } else if (method == 1) {   // GICP
        point_wise_scan_matching(corn, surf, submap_corn, submap_surf, T_w_c);
    } else {
        std::cerr << "scan2submap, no such mehtod!" << std::endl;
        return T_w_c;
    }

//    T_odom_map = pose_normalize(T_w_odomcurr.inverse() * T_w_c);
//    printf("mapping, scan to submap: %.3f ms\n", t_mapping.toc());
    return T_w_c;
}

void LidarSensor::initParam() {

    submap_corn.reset(new pcl::PointCloud<PointT>());
    submap_surf.reset(new pcl::PointCloud<PointT>());

    ds_corn.setLeafSize(corn_filter_length, corn_filter_length, corn_filter_length);
    ds_surf.setLeafSize(surf_filter_length, surf_filter_length, surf_filter_length);
    ds_surf_2.setLeafSize(surf_filter_length * 2, surf_filter_length * 2, surf_filter_length * 2);
    ds_raw.setLeafSize(0.1, 0.1, 0.1);

    ds_corn_submap.setLeafSize(corn_filter_length, corn_filter_length, corn_filter_length);
    ds_surf_submap.setLeafSize(surf_filter_length, surf_filter_length, surf_filter_length);

    opti_counter = 2;

    keyframeVec = std::make_shared<KeyframeVec>();
    keyframeVec->keyframes.reserve(200);
    status = std::make_shared<LidarStatus>();
    frameChannel = std::make_shared<FrameChannel>();

    nscans_gen = NScans(n_scans);

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
}

void LidarSensor::update_keyframe(const ros::Time& cloud_in_time, const pcl::PointCloud<PointT>::Ptr& corn,
                                  const pcl::PointCloud<PointT>::Ptr& surf, const pcl::PointCloud<PointT>::Ptr& full)
{
    current_keyframe = std::make_shared<Keyframe>(keyframeVec->size(), cloud_in_time, corn, surf, full);
    keyframeVec->emplace_back(current_keyframe);
    is_keyframe_next = false;
}

gtsam::Pose3 LidarSensor::update_odom(const ros::Time& cloud_in_time, const pcl::PointCloud<PointT>::Ptr& corn,
                                      const pcl::PointCloud<PointT>::Ptr& surf, const pcl::PointCloud<PointT>::Ptr& raw,
                                      gtsam::Pose3& pose_raw) {

    if (frameCount == 0) {
        curr_frame = std::make_shared<Frame>(frameCount++, cloud_in_time, corn, surf, raw, gtsam::Pose3());
        frameChannel->push(curr_frame);
        nscans_gen.add(curr_frame);
        return gtsam::Pose3();
    }

    double a, b, c;
    TicToc tic;
    ds_corn.setInputCloud(corn);
    ds_corn.filter(*corn);
    a = tic.toc(); tic.tic();
    ds_surf.setInputCloud(surf);
    ds_surf.filter(*surf);
    b = tic.toc(); tic.tic();
    ds_raw.setInputCloud(raw);
    ds_raw.filter(*raw);
    c = tic.toc(); tic.tic();
//    printf("Downsampling corn: %.3f ms, surf: %.3f ms, raw: %.3f ms\n", a, b, c);

    pcl::PointCloud<PointT>::Ptr surf_ds2(new pcl::PointCloud<PointT>);
    ds_surf_2.setInputCloud(surf);
    ds_surf_2.filter(*surf_ds2);

    pose_raw = scan2scan(cloud_in_time, corn, surf_ds2, raw);
    pose_raw = pose_normalize(pose_raw);

    curr_frame = std::make_shared<Frame>(frameCount++, cloud_in_time, corn, surf, raw, pose_raw);
    frameChannel->push(curr_frame);

    std::lock_guard<std::mutex> lg0(T_lock0), lg1(T_lock1);
    T_w_curr = pose_normalize(T_w_wmap * T_wmap_wodom * pose_raw); // 10Hz
    return T_w_curr;
}

gtsam::Pose3 LidarSensor::update_mapping(const Frame::Ptr& frame) {

    if (is_keyframe_next) {
        update_keyframe(frame->cloud_in_time, frame->corn_features, frame->surf_features, frame->raw);
    }

    if (!is_init) {
        current_keyframe->set_init(gtsam::Pose3());
        T_wmap_lastkey = gtsam::Pose3();
        factor_graph_opti();
        is_init = true;
        return gtsam::Pose3();
    }

    T_wmap_last = T_wmap_curr;
//    gtsam::Pose3 T_pred = T_wmap_last * T_wmap_delta;
    gtsam::Pose3 T_pred = T_wmap_wodom * frame->pose_w_curr;

    T_wmap_curr = scan2submap(frame->corn_features, frame->surf_features, T_pred, mapping_method);
    T_wmap_curr = pose_normalize(T_wmap_curr);

    if (!current_keyframe->is_init()) {
        current_keyframe->set_increment(T_wmap_lastkey.between(T_wmap_curr));
        current_keyframe->set_init(T_wmap_curr);
        T_wmap_lastkey = T_wmap_curr;
        factor_graph_opti();

        auto T_opti = current_keyframe->pose_fixed;
        {
            T_lock0.lock();
            T_w_wmap = pose_normalize(T_opti * T_wmap_curr.inverse());
            T_lock0.unlock();
        }
    }

    T_wmap_delta = T_wmap_last.between(T_wmap_curr);
    is_keyframe_next = nextFrameToBeKeyframe();
    {
        T_lock1.lock();
        T_wmap_wodom = pose_normalize(T_wmap_curr * frame->pose_w_curr.inverse());
        T_lock1.unlock();
    }

    return T_w_wmap * T_wmap_curr;
}

void LidarSensor::addOdomFactor(int last_index, int curr_index) {

    if (last_index < 0) {
        BAGraph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0), gtsam::Pose3(), prior_noise_model);
        BAEstimate.insert(X(0), gtsam::Pose3());
        std::cout << "init Odom factor!" << std::endl;
        return;
    }

    // read lock
    gtsam::Pose3 last_pose = keyframeVec->read_pose(last_index, true);
    gtsam::Pose3 curr_pose = last_pose * current_keyframe->pose_last_curr;

//    TicToc tic;

//    auto information = GetInformationMatrixFromPointClouds(keyframeVec->keyframes[curr_index]->raw,
//                                                           keyframeVec->keyframes[last_index]->raw,
//                                                           2.0,
//                                                           current_keyframe->pose_last_curr.matrix());
//    printf("information: %.3f ms\n", tic.toc());


    BAGraph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(last_index),
                                                                X(curr_index),
                                                                current_keyframe->pose_last_curr,
                                                                odometry_noise_model);
//                                                            gtsam::noiseModel::Diagonal::Information(information));
    BAEstimate.insert(X(curr_index), curr_pose);
//    printf("add Odom factor between [%d] and [%d]\n", last_index, curr_index);
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
        std::this_thread::sleep_for(std::chrono::milliseconds(4));
    }

}


void LidarSensor::factor_graph_opti() {

    TicToc t_opti;

    int curr_index = current_keyframe->index;

    addOdomFactor(curr_index - 1, curr_index);

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
//    isam->getFactorsUnsafe().saveGraph("/home/ziv/catkin_loam/src/factor_graph.dot", isamOptimize);


    {   // write lock
        std::unique_lock<std::shared_mutex> ul(keyframeVec->pose_mtx);
        for (size_t i = 0; i < isamOptimize.size(); i++) {
            keyframeVec->at(i)->set_fixed(isamOptimize.at<gtsam::Pose3>(X(i)));
        }
    }
    if (need_loop) {
        FixedKeyframeBuf.push(current_keyframe);
    }


//    cout << "opti pose:\n" << keyframeVec->read_pose(curr_index, true).matrix() << endl;
//    cout << "########################################" << endl << endl;
    // debug
//    auto poseVec = keyframeVec->read_poses(0, keyframeVec->size(), true);
//    auto poseVec1 = keyframeVec->read_poses(0, keyframeVec->size(), false);
//    {
//        std::ofstream f(file_save_path + "debug.txt");
//        for (size_t i = 0; i < poseVec.size(); i++) {
//            f << "i: " << i << "\n" << poseVec[i].matrix() << endl;
//        }
//        for (size_t i = 0; i < poseVec1.size(); i++) {
//            f << "i: " << i << "\n" << poseVec1[i].matrix() << endl;
//        }
//        f.close();
//    }

//    printf("factor graph opti: %.3f ms\n", t_opti.toc());
}


LidarSensor::LidarSensor()
{
    nh.param<bool>  ("need_loop", need_loop, true);
    nh.param<double>("keyframe_distance_threshold", keyframe_distance_thres, 0.6);
    nh.param<double>("keyframe_angle_threshold",    keyframe_angle_thres,    0.1);
    nh.param<double>("surf_filter_length", surf_filter_length, 0.4);
    nh.param<double>("corn_filter_length", corn_filter_length, 0.2);

    /* mapping_method
     * 0 = feature-based
     * 1 = GICP
     */
    nh.param<int>("mapping_method", mapping_method, 0);
    nh.param<int>("submap_len", submap_len, 6);
    nh.param<int>("n_scans", n_scans, 3);

    file_save_path = nh.param<std::string>("file_save_path", "");
//    f_origin.open(file_save_path + "origin.txt");

    initParam();
}





