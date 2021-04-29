//
// Created by ziv on 2020/12/31.
//

#include "lidar.h"

int OCTO_TREE::voxel_windowsize;
gtsam::SharedNoiseModel LidarSensor::edge_noise_model;
gtsam::SharedNoiseModel LidarSensor::surf_noise_model;
static const int backend_sleep = 5;    // 5ms

// extract from balm back
static double voxel_size[2] = {1, 1};

static void
cut_voxel(unordered_map<VOXEL_LOC, OCTO_TREE *> &feat_map, pcl::PointCloud<PointT>::Ptr pl_feat, Eigen::Matrix3d R_p,
          Eigen::Vector3d t_p, int feattype, int fnum, int capacity) {
    uint plsize = pl_feat->size();
    for (uint i = 0; i < plsize; i++) {
        PointT &p_c = pl_feat->points[i];
        Eigen::Vector3d pvec_orig(p_c.x, p_c.y, p_c.z);
        Eigen::Vector3d pvec_tran = R_p * pvec_orig + t_p;

        float loc_xyz[3];
        for (int j = 0; j < 3; j++) {
            loc_xyz[j] = pvec_tran[j] / voxel_size[feattype];
            if (loc_xyz[j] < 0) {
                loc_xyz[j] -= 1.0;
            }
        }

        VOXEL_LOC position((int64_t) loc_xyz[0], (int64_t) loc_xyz[1], (int64_t) loc_xyz[2]);
        auto iter = feat_map.find(position);
        if (iter != feat_map.end()) {
            iter->second->plvec_orig[fnum]->push_back(pvec_orig);
            iter->second->plvec_tran[fnum]->push_back(pvec_tran);
            iter->second->is2opt = true;
        } else {
            OCTO_TREE *ot = new OCTO_TREE(feattype, capacity);
            ot->plvec_orig[fnum]->push_back(pvec_orig);
            ot->plvec_tran[fnum]->push_back(pvec_tran);

            ot->voxel_center[0] = (0.5 + position.x) * voxel_size[feattype];
            ot->voxel_center[1] = (0.5 + position.y) * voxel_size[feattype];
            ot->voxel_center[2] = (0.5 + position.z) * voxel_size[feattype];
            ot->quater_length = voxel_size[feattype] / 4.0;
            feat_map[position] = ot;
        }
    }
}

gtsam::Pose3 pose_normalize(const gtsam::Pose3 &pose) {
    return gtsam::Pose3(pose.rotation().normalized(), pose.translation());
}

gtsam::Pose3 toPose3(const Eigen::Quaterniond &q, const Eigen::Vector3d &t) {
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

void
LidarSensor::addCornCostFactor(const pcl::PointCloud<PointT>::Ptr &pc_in, const pcl::PointCloud<PointT>::Ptr &map_in,
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

void
LidarSensor::addSurfCostFactor(const pcl::PointCloud<PointT>::Ptr &pc_in, const pcl::PointCloud<PointT>::Ptr &map_in,
                               const pcl::KdTreeFLANN<PointT> &kdtree_surf, const gtsam::Pose3 &odom,
                               gtsam::NonlinearFactorGraph &factors, const gtsam::Pose3 &point_transform) {
    int surf_num = 0;
    PointT point_temp, point_transformed;
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    Eigen::Affine3d odom_(odom.matrix());
    Eigen::Affine3d point_transform_(point_transform.matrix());
    bool need_transform = !point_transform.equals(gtsam::Pose3::identity());

    for (int i = 0; i < (int) pc_in->points.size(); i++) {

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

    std::lock_guard<std::mutex> lg(submap->mtx);

    auto submap_corn = MapGenerator::generate_cloud(keyframeVec, range_from, range_to, FeatureType::Corn);
    auto submap_surf = MapGenerator::generate_cloud(keyframeVec, range_from, range_to, FeatureType::Surf);

    ds_corn_submap.setInputCloud(submap_corn);
    ds_surf_submap.setInputCloud(submap_surf);
    ds_corn_submap.filter(*submap_corn);
    ds_surf_submap.filter(*submap_surf);
//    down_sampling_voxel(*submap_corn, corn_filter_length);
//    down_sampling_voxel(*submap_surf, surf_filter_length);
//    voxelGrid_corn.setInputCloud(submap_corn);
//    voxelGrid_surf.setInputCloud(submap_surf);
//    voxelGrid_corn.filter(*submap_corn);
//    voxelGrid_surf.filter(*submap_surf);

    submap->submap_corn = submap_corn;
    submap->submap_surf = submap_surf;
    submap->from = range_from;
    submap->end = range_to;
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

void LidarSensor::feature_based_scan_matching(const pcl::PointCloud<PointT>::Ptr &corn,
                                              const pcl::PointCloud<PointT>::Ptr &surf,
                                              const pcl::PointCloud<PointT>::Ptr &corns,
                                              const pcl::PointCloud<PointT>::Ptr &surfs,
                                              pcl::KdTreeFLANN<PointT> &kdtree_corn,
                                              pcl::KdTreeFLANN<PointT> &kdtree_surf,
                                              gtsam::Pose3 &T_w_c) {
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

void LidarSensor::point_wise_scan_matching(const pcl::PointCloud<PointT>::Ptr &corn,
                                           const pcl::PointCloud<PointT>::Ptr &surf,
                                           const pcl::PointCloud<PointT>::Ptr &corns,
                                           const pcl::PointCloud<PointT>::Ptr &surfs,
                                           gtsam::Pose3 &T_w_c) {
    Eigen::Matrix4d final;

    pcl::PointCloud<PointT>::Ptr submap_pcd(new pcl::PointCloud<PointT>);
    *submap_pcd += *corns;
    *submap_pcd += *surfs;
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
        feature_based_scan_matching(corn, surf, corns, surfs, kdtree_corn_submap, kdtree_surf_submap, T_w_c);
    }
}

gtsam::Pose3 LidarSensor::scan2scan(const ros::Time &cloud_in_time, const pcl::PointCloud<PointT>::Ptr &corn,
                                    const pcl::PointCloud<PointT>::Ptr &surf, const pcl::PointCloud<PointT>::Ptr &raw) {

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
        printf("too less features! corn = [%d], surf = [%d], exit!\n", int(corn_scans->size()),
               int(surf_scans->size()));
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

gtsam::Pose3
LidarSensor::scan2submap(const pcl::PointCloud<PointT>::Ptr &corn, const pcl::PointCloud<PointT>::Ptr &surf,
                         const gtsam::Pose3 &guess, int method) {

    auto T_w_c = guess; // T_w_mapcurr

    if (opti_counter > 2) opti_counter--;

    if (!corn || !surf) {
        std::cerr << "Error nullptr corn or surf" << std::endl;
        return T_w_c;
    }

    TicToc t_gen_submap;
    // MapGenerator::generate_cloud gen pcd range from: [begin, end), so need to add 1
    int c_index = current_keyframe->index + 1;
    updateSubmap(c_index - window_size, c_index);
    printf("prepare submap: %.3f ms\n", t_gen_submap.toc());

    submap->mtx.lock();
    auto submapCorn_copy = submap->submap_corn->makeShared();
    auto submapSurf_copy = submap->submap_surf->makeShared();
    submap->mtx.unlock();

    TicToc t_mapping;
    if (method == 0) {          // feature-based
        feature_based_scan_matching(corn, surf, submapCorn_copy, submapSurf_copy,
                                    kdtree_corn_submap, kdtree_surf_submap, T_w_c);
    } else if (method == 1) {   // GICP
        point_wise_scan_matching(corn, surf, submapCorn_copy, submapSurf_copy, T_w_c);
    } else {
        std::cerr << "scan2submap, no such mehtod!" << std::endl;
        return T_w_c;
    }

//    printf("mapping, scan to submap: %.3f ms\n", t_mapping.toc());
    return T_w_c;
}

gtsam::Pose3 LidarSensor::update_odom(const ros::Time &cloud_in_time, const pcl::PointCloud<PointT>::Ptr &corn,
                                      const pcl::PointCloud<PointT>::Ptr &surf, const pcl::PointCloud<PointT>::Ptr &raw,
                                      gtsam::Pose3 &pose_raw) {

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
    a = tic.toc();
    tic.tic();
    ds_surf.setInputCloud(surf);
    ds_surf.filter(*surf);
    b = tic.toc();
    tic.tic();
    ds_raw.setInputCloud(raw);
    ds_raw.filter(*raw);
    c = tic.toc();
    tic.tic();
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

gtsam::Pose3 LidarSensor::update_mapping(const Frame::Ptr &frame) {

    if (is_keyframe_next) {
        update_keyframe(frame->cloud_in_time, frame->corn_features, frame->surf_features, frame->raw);
    }

    if (!is_init) {
        current_keyframe->set_init(gtsam::Pose3());
        current_keyframe->set_fixed(gtsam::Pose3());
        T_wmap_lastkey = gtsam::Pose3();
//        updateSubmap(0, 1);
        BAKeyframeChannel->push(current_keyframe);
//        factor_graph_opti();
        is_init = true;
        return gtsam::Pose3();
    }

    T_wmap_last = T_wmap_curr;
//    gtsam::Pose3 T_pred = T_wmap_last * T_wmap_delta;
    gtsam::Pose3 T_pred = T_wmap_wodom * frame->pose_w_curr;

//    while (current_keyframe->index > submap->end + 2 && current_keyframe->index > window_base + 10) {
//        std::this_thread::sleep_for(std::chrono::milliseconds(1));
//    }

    T_wmap_curr = scan2submap(frame->corn_features, frame->surf_features, T_pred, mapping_method);
    T_wmap_curr = pose_normalize(T_wmap_curr);

//    cout << "T_wmap_curr:\n" << T_wmap_curr.matrix() << endl;

    if (!current_keyframe->is_init()) {
        current_keyframe->set_increment(T_wmap_lastkey.between(T_wmap_curr));
        current_keyframe->set_init(T_wmap_curr);

//        cout << "keyframe!!!! " << current_keyframe->index << ", T_wmap_curr:\n" << T_wmap_curr.matrix() << endl;
        f_backend_timecost << T_wmap_curr.matrix() << endl;
        T_wmap_lastkey = T_wmap_curr;

        BAKeyframeChannel->push(current_keyframe);

//        if (current_keyframe->index < window_size) {
//            updateSubmap(0, current_keyframe->index);
//        }
    }

//    if (!current_keyframe->is_init()) {
//        current_keyframe->set_increment(T_wmap_lastkey.between(T_wmap_curr));
//        current_keyframe->set_init(T_wmap_curr);
//        T_wmap_lastkey = T_wmap_curr;
//        factor_graph_opti();
//
//        auto T_opti = current_keyframe->pose_fixed;
//        {
//            T_lock0.lock();
//            T_w_wmap = pose_normalize(T_opti * T_wmap_curr.inverse());
//            T_lock0.unlock();
//        }
//    }

    T_wmap_delta = T_wmap_last.between(T_wmap_curr);
    is_keyframe_next = nextFrameToBeKeyframe();
    {
        T_lock1.lock();
        T_wmap_wodom = pose_normalize(T_wmap_curr * frame->pose_w_curr.inverse());
        T_lock1.unlock();
    }

    return T_w_wmap * T_wmap_curr;
}

void LidarSensor::initBALMParam() {
    // balm init
    q_odom.setIdentity();
    q_gather_pose.setIdentity();
    q_last.setIdentity();
    t_odom.setZero();
    t_gather_pose.setZero();
    t_last.setZero();

    pl_full_buf.clear();
    surf_map.clear();
    corn_map.clear();
    q_poses.clear();
    t_poses.clear();
    plcount = 0;
    window_base = 0;
}

void LidarSensor::initParam() {

    submap = std::make_shared<Submap>();
    ds_corn.setLeafSize(corn_filter_length, corn_filter_length, corn_filter_length);
    ds_surf.setLeafSize(surf_filter_length, surf_filter_length, surf_filter_length);
    ds_surf_2.setLeafSize(surf_filter_length * 2, surf_filter_length * 2, surf_filter_length * 2);
    ds_raw.setLeafSize(0.1, 0.1, 0.1);

    ds_corn_submap.setLeafSize(corn_filter_length, corn_filter_length, corn_filter_length);
    ds_surf_submap.setLeafSize(surf_filter_length, surf_filter_length, surf_filter_length);

    opti_counter = 10;

    keyframeVec = std::make_shared<KeyframeVec>();
    keyframeVec->keyframes.reserve(200);
    status = std::make_shared<LidarStatus>();
    frameChannel = std::make_shared<Channel<Frame::Ptr>>();
    BAKeyframeChannel = std::make_shared<Channel<Keyframe::Ptr>>();
    MargiKeyframeChannel = std::make_shared<Channel<Keyframe::Ptr>>();
    factorsChannel = std::make_shared<Channel<FactorPtr>>();

    auto edge_gaussian_model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(3) << 0.2, 0.2, 0.2).finished());
    auto surf_gaussian_model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(1) << 0.2).finished());

    edge_noise_model = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(0.58),
                                                         edge_gaussian_model);
    surf_noise_model = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(0.02),
                                                         surf_gaussian_model);
    prior_noise_model = gtsam::noiseModel::Diagonal::Variances(
            (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished());

    gtsam::ISAM2Params isam2Params;
    isam2Params.relinearizeThreshold = 0.1;
    isam2Params.relinearizeSkip = 1;
    isam = std::make_shared<gtsam::ISAM2>(isam2Params);

    initBALMParam();

}

//pcl::PointCloud<PointT>::Ptr LidarSensor::extract_planes(pcl::PointCloud<PointT>::Ptr currSurf) {
//    TimeCounter t1;
//    pcl::PointCloud<NormalT>::Ptr cloud_normals(new pcl::PointCloud<NormalT>());
//    std::vector<pcl::PointIndices> clusters;
//    ne.setInputCloud(currSurf);
//    ne.compute(*cloud_normals);
//    regionGrowing.setInputCloud(currSurf);
//    regionGrowing.setInputNormals(cloud_normals);
//    regionGrowing.extract(clusters);
//
//    pcl::PointCloud<PointT>::Ptr planes(new pcl::PointCloud<PointT>());
//
//    for (const auto& cluster : clusters) {
//        size_t size = cluster.indices.size();
//        Eigen::MatrixXd matA0(size, 3), matB0(size, 1);
//        matB0.setOnes();
//        matB0 = -1 * matB0;
//
//        for (size_t j = 0; j < size; j++) {
//            int idx = cluster.indices[j];
//            matA0(j, 0) = currSurf->points[idx].x;
//            matA0(j, 1) = currSurf->points[idx].y;
//            matA0(j, 2) = currSurf->points[idx].z;
//        }
//        Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
//        double d = 1 / norm.norm();
//        norm.normalize();
//
//        bool planeValid = true;
//        for (size_t j = 0; j < size; j++) {
//            int idx = cluster.indices[j];
//            if (fabs(norm(0) * currSurf->points[idx].x +
//                     norm(1) * currSurf->points[idx].y +
//                     norm(2) * currSurf->points[idx].z + d) > 0.25) {
//                planeValid = false;
//                break;
//            }
//        }
//
//        if (planeValid) {
//            for (size_t j = 0; j < size; j++) {
//                planes->push_back(currSurf->points[cluster.indices[j]]);
//            }
//        }
//    }
//    return planes;
//}

void LidarSensor::update_keyframe(const ros::Time &cloud_in_time,
                                  pcl::PointCloud<PointT>::Ptr currEdge,
                                  pcl::PointCloud<PointT>::Ptr currSurf,
                                  pcl::PointCloud<PointT>::Ptr currFull) {
    current_keyframe = std::make_shared<Keyframe>(keyframeVec->keyframes.size(), cloud_in_time, currEdge, currSurf,
                                                  currFull);
    keyframeVec->keyframes.emplace_back(current_keyframe);
    is_keyframe_next = false;
}

void LidarSensor::addOdomFactor(int last_index, int curr_index) {

    using namespace gtsam;

    if (last_index < 0) {
        BAGraph.addPrior(X(0), Pose3::identity(), prior_noise_model);
        BAEstimate.insert(X(0), Pose3::identity());
        f_pose_fixed << Eigen::Matrix4d::Identity() << endl;
//        pcl::io::savePCDFile(file_save_path + "0.pcd", *keyframeVec->keyframes[curr_index]->raw);
        std::cout << "init Odom factor!" << std::endl;
        return;
    }

    // read lock
    gtsam::Pose3 pose_last = keyframeVec->read_pose(last_index, Type::Opti);
    gtsam::Pose3 pose_delta = keyframeVec->read_pose(curr_index, Type::Delta);
    gtsam::Pose3 pose_curr = pose_last * pose_delta;
    keyframeVec->at(curr_index)->set_fixed(pose_curr);

    printf("add Odom factor between [%d] and [%d]\n", last_index, curr_index);
//    cout << "last_pose:\n" << pose_last.matrix() << endl;
//    cout << "curr pose:\n" << pose_curr.matrix() << endl;

    auto information = GetInformationMatrixFromPointClouds(keyframeVec->keyframes[curr_index]->raw,
                                                           keyframeVec->keyframes[last_index]->raw,
                                                           2.0,
                                                           pose_delta.matrix());

    f_pose_fixed << pose_curr.matrix() << endl;
//    pcl::io::savePCDFile(file_save_path + to_string(curr_index) + ".pcd", *keyframeVec->keyframes[curr_index]->raw);
//    f_pose_fixed << information << endl << endl;

    BAGraph.emplace_shared<BetweenFactor<Pose3>>(X(last_index), X(curr_index),
                                                 pose_delta, noiseModel::Gaussian::Information(information));
    BAEstimate.insert(X(curr_index), pose_curr);
//    printf("add Odom factor between [%d] and [%d]\n", last_index, curr_index);

}

void LidarSensor::BALM_backend(const std::vector<Keyframe::Ptr> &keyframes_buf,
                               std::vector<Keyframe::Ptr> &fixedKeyframes_buf) {

    for (const auto &keyframe : keyframes_buf) {
//        Timer t_BALM("BALM keyframe_" + std::to_string(keyframe->index));
//        TicToc tic;

        gtsam::Pose3 curr_pose = keyframe->pose_odom;
        q_odom = Eigen::Quaterniond(curr_pose.rotation().matrix());
        t_odom = Eigen::Vector3d(curr_pose.translation());

        Eigen::Vector3d delta_t(q_last.matrix().transpose() * (t_odom - t_last));
        Eigen::Quaterniond delta_q(q_last.matrix().transpose() * q_odom.matrix());
        q_last = q_odom;
        t_last = t_odom;

        t_gather_pose = t_gather_pose + q_gather_pose * delta_t;
        q_gather_pose = q_gather_pose * delta_q;

        if (plcount == 0) {
            q_poses.push_back(q_gather_pose);
            t_poses.push_back(t_gather_pose);
        } else {
            t_poses.push_back(t_poses[plcount - 1] + q_poses[plcount - 1] * t_gather_pose);
            q_poses.push_back(q_poses[plcount - 1] * q_gather_pose);
        }

        pl_corn = keyframe->corn_features;
        pl_surf = keyframe->surf_features;
        plcount++;
        OCTO_TREE::voxel_windowsize = plcount - window_base;
        q_gather_pose.setIdentity();
        t_gather_pose.setZero();

        int frame_head = plcount - 1 - window_base;
        cut_voxel(surf_map, pl_surf, q_poses[plcount - 1].matrix(), t_poses[plcount - 1], 0, frame_head, window_size);
        cut_voxel(corn_map, pl_corn, q_poses[plcount - 1].matrix(), t_poses[plcount - 1], 1, frame_head, window_size);

        for (auto iter = surf_map.begin(); iter != surf_map.end(); ++iter) {
            if (iter->second->is2opt) {
                iter->second->root_centors.clear();
                iter->second->recut(0, frame_head, iter->second->root_centors);
            }
        }

        for (auto iter = corn_map.begin(); iter != corn_map.end(); ++iter) {
            if (iter->second->is2opt) {
                iter->second->root_centors.clear();
                iter->second->recut(0, frame_head, iter->second->root_centors);
            }
        }

        if (plcount >= window_base + window_size) {

            for (int i = 0; i < window_size; i++) {
                opt_lsv.so3_poses[i].setQuaternion(q_poses[window_base + i]);
                opt_lsv.t_poses[i] = t_poses[window_base + i];
            }

            // optimize step
            if (window_base != 0) {
                for (auto iter = surf_map.begin(); iter != surf_map.end(); ++iter) {
                    if (iter->second->is2opt) {
                        iter->second->traversal_opt(opt_lsv);
                    }
                }

                for (auto iter = corn_map.begin(); iter != corn_map.end(); ++iter) {
                    if (iter->second->is2opt) {
                        iter->second->traversal_opt(opt_lsv);
                    }
                }

                opt_lsv.damping_iter();
            }

            {
                std::unique_lock<std::shared_mutex> ul(keyframeVec->pose_mtx); // write lock

                for (int i = 0; i < window_size; i++) {
                    q_poses[window_base + i] = opt_lsv.so3_poses[i].unit_quaternion();
                    t_poses[window_base + i] = opt_lsv.t_poses[i];
                    keyframeVec->keyframes[window_base + i]->pose_odom = toPose3(q_poses[window_base + i],
                                                                                 t_poses[window_base + i]);

                    f_balm << keyframeVec->keyframes[window_base + i]->pose_odom.matrix() << endl;

                }

                // optimized pose value
                for (int i = 0; i < margi_size; i++) {
                    auto keyframe = keyframeVec->keyframes[window_base + i];
                    if (keyframe->index > 0) {
                        auto last_pose_odom = keyframeVec->at(keyframe->index - 1)->pose_odom;
                        auto curr_pose_odom = keyframeVec->at(keyframe->index)->pose_odom;
                        keyframe->set_increment(pose_normalize(last_pose_odom.between(curr_pose_odom)));
                    }
                    fixedKeyframes_buf.push_back(keyframe);
                }
            }

            // update submap here, front end will use
//            updateSubmap(window_base, window_base + window_size);

            for (auto iter = surf_map.begin(); iter != surf_map.end(); ++iter) {
                if (iter->second->is2opt) {
                    iter->second->root_centors.clear();
                    iter->second->marginalize(0, margi_size, q_poses, t_poses, window_base, iter->second->root_centors);
                }
            }

            for (auto iter = corn_map.begin(); iter != corn_map.end(); ++iter) {
                if (iter->second->is2opt) {
                    iter->second->root_centors.clear();
                    iter->second->marginalize(0, margi_size, q_poses, t_poses, window_base, iter->second->root_centors);
                }
            }

            window_base += margi_size;
            opt_lsv.free_voxel();
        }
//        t_BALM.count();
//        f_backend_timecost << tic.toc() << endl;
    }
}

void LidarSensor::loop_detect_thread() {

    int last_loop_found_index = -1;

    while (ros::ok()) {

        std::vector<Keyframe::Ptr> fixedKeyframes_buf = MargiKeyframeChannel->get_all();

        std::vector<FactorPtr> factors;
        for (const auto &keyframe: fixedKeyframes_buf) {
            if (loopDetector.loop_detector(keyframeVec, keyframe, factors,
                                           last_loop_found_index)) {
                for (const auto &factor: factors) {
                    factorsChannel->push(factor);
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

    while (ros::ok()) {

        auto keyframes_buf = BAKeyframeChannel->get_all();

        if (!keyframes_buf.empty() || !factorsChannel->empty()) {
            std::vector<Keyframe::Ptr> margiKeyframes;
            // from balm backend
            BALM_backend(keyframes_buf, margiKeyframes);

            // deal with fixed keyframes with factorGraph
            if (!margiKeyframes.empty()) {
                for (const auto &keyframe: margiKeyframes) {
                    int curr_index = keyframe->index;
                    addOdomFactor(last_index, curr_index);
                    last_index = curr_index;
                    if (need_loop) {
                        MargiKeyframeChannel->push(keyframe);
                    }
                }
            }
            bool has_loop = false;
            if (!factorsChannel->empty()) {
                auto vec = factorsChannel->get_all();
                for (const auto &factor : vec) {
                    BAGraph.add(factor);
                }
                has_loop = true;
            }

            // isam update
            isam->update(BAGraph, BAEstimate);
            isam->update();
            isamOptimize = isam->calculateEstimate();

            BAGraph.resize(0);
            BAEstimate.clear();

            if (has_loop) {
                status->status_change = true;
            }

            {   // write lock
                std::unique_lock<std::shared_mutex> ul(keyframeVec->pose_mtx);
                for (size_t i = 0; i < isamOptimize.size(); i++) {
                    keyframeVec->at(i)->set_fixed(isamOptimize.at<gtsam::Pose3>(X(i)));
                }
            }
            auto T_opti = keyframeVec->read_pose(isamOptimize.size() - 1, true);
            auto T_before = keyframeVec->read_pose(isamOptimize.size() - 1, false);
            T_lock0.lock();
            T_w_wmap = pose_normalize(T_opti * T_before.inverse());
            T_lock0.unlock();
            // debug
//            auto poseVec = keyframeVec->read_poses(0, keyframeVec->size(), true);
//            {
//                std::ofstream f(nh.param<std::string>("file_save_path", "") + "debug_pose.txt");
//                for (size_t i = 0; i < poseVec.size(); i++) {
//                    f << "i: " << i << "\n" << poseVec[i].matrix() << endl;
//                }
//                f.close();
//            }
        }

        //sleep 5 ms every time
        std::this_thread::sleep_for(std::chrono::milliseconds(backend_sleep));
    }
}

void _mkdir(const std::string &filename) {
    boost::filesystem::path save_path(filename);
    auto folder_path = save_path.parent_path();
    if (!boost::filesystem::exists(folder_path)) {
        boost::filesystem::create_directories(folder_path);
    }
}

LidarSensor::LidarSensor(int i) : opt_lsv(window_size, filter_num, thread_num), loopDetector(i) {

    file_save_path = nh.param<std::string>("file_save_path", "");
    _mkdir(file_save_path + "test.txt");
    f_pose_fixed.open(file_save_path + "lidar" + to_string(i) + "fixed_poses_raw.txt");
    f_backend_timecost.open(file_save_path + "lidar" + to_string(i) + "backend_timecost.txt");
    f_balm.open(file_save_path + "lidar" + to_string(i) + "balm.txt");
//    cout << "#######################################" << endl;
//    cout << "current_lidar: " << i << endl;
//    cout << "f_pose_fixed path: " << file_save_path + "lidar" + to_string(i) + "fixed_poses_raw.txt" << endl;
//    cout << "f_pose_fixed isopen? " << f_pose_fixed.is_open() << endl;
//    cout << "f_backend_timecost path: " << "lidar" + to_string(i) + "backend_timecost.txt" << endl;
//    cout << "f_backend_timecost isopen? " << f_backend_timecost.is_open() << endl;
//    cout << "#######################################" << endl;


    nh.param<bool>("need_loop", need_loop, true);
    nh.param<double>("keyframe_distance_threshold", keyframe_distance_thres, 0.6);
    nh.param<double>("keyframe_angle_threshold", keyframe_angle_thres, 0.1);
    nh.param<int>("window_size", window_size, 6);
    nh.param<int>("margi_size", margi_size, 3);
    nh.param<double>("surf_filter_length", surf_filter_length, 0.4);
    nh.param<double>("corn_filter_length", corn_filter_length, 0.2);

    nh.param<int>("mapping_method", mapping_method, 0);
    nh.param<int>("n_scans", n_scans, 3);

    opt_lsv.set_window_size(window_size);
    initParam();
}
