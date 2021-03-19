//
// Created by ziv on 2020/12/31.
//

#include "lidar.h"

int OCTO_TREE::voxel_windowsize;
gtsam::SharedNoiseModel LidarSensor::edge_noise_model;
gtsam::SharedNoiseModel LidarSensor::surf_noise_model;
static const int backend_sleep = 5;    // 5ms
static const int end_signal = 20000;   // 20000ms

// extract from balm back
static double voxel_size[2] = {1, 1};
static void cut_voxel(unordered_map<VOXEL_LOC, OCTO_TREE*> &feat_map, pcl::PointCloud<PointT>::Ptr pl_feat, Eigen::Matrix3d R_p, Eigen::Vector3d t_p, int feattype, int fnum, int capacity)
{
    uint plsize = pl_feat->size();
    for(uint i=0; i<plsize; i++)
    {
        PointT &p_c = pl_feat->points[i];
        Eigen::Vector3d pvec_orig(p_c.x, p_c.y, p_c.z);
        Eigen::Vector3d pvec_tran = R_p*pvec_orig + t_p;

        float loc_xyz[3];
        for(int j=0; j<3; j++)
        {
            loc_xyz[j] = pvec_tran[j] / voxel_size[feattype];
            if(loc_xyz[j] < 0)
            {
                loc_xyz[j] -= 1.0;
            }
        }

        VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
        auto iter = feat_map.find(position);
        if(iter != feat_map.end())
        {
            iter->second->plvec_orig[fnum]->push_back(pvec_orig);
            iter->second->plvec_tran[fnum]->push_back(pvec_tran);
            iter->second->is2opt = true;
        }
        else
        {
            OCTO_TREE *ot = new OCTO_TREE(feattype, capacity);
            ot->plvec_orig[fnum]->push_back(pvec_orig);
            ot->plvec_tran[fnum]->push_back(pvec_tran);

            ot->voxel_center[0] = (0.5+position.x) * voxel_size[feattype];
            ot->voxel_center[1] = (0.5+position.y) * voxel_size[feattype];
            ot->voxel_center[2] = (0.5+position.z) * voxel_size[feattype];
            ot->quater_length = voxel_size[feattype] / 4.0;
            feat_map[position] = ot;
        }
    }
}

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
    std::lock_guard<std::mutex> lg(submap_mtx);
    if (range_to == 0) {
        range_to = keyframeVec->keyframes.size();
    }
    if (range_from == 0) {
        range_from = range_to < window_size ? 0 : range_to - window_size;
    } else if (range_from >= range_to) {
        return;
    }

    submap_corn = MapGenerator::generate_cloud(keyframeVec, range_from, range_to, FeatureType::Edge);
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

void LidarSensor::initBALMParam() {
    // balm init
    q_odom.setIdentity(); q_gather_pose.setIdentity(); q_last.setIdentity();
    t_odom.setZero();     t_gather_pose.setZero();     t_last.setZero();

    pl_full_buf.clear(); surf_map.clear(); corn_map.clear();
    q_poses.clear(); t_poses.clear();
    plcount = 0; window_base = 0;
}

void LidarSensor::initParam() {

    submap_corn.reset(new pcl::PointCloud<PointT>());
    submap_surf.reset(new pcl::PointCloud<PointT>());
    voxelGrid.setLeafSize(0.01, 0.01, 0.01);

    opti_counter = 12;

    keyframeVec = std::make_shared<KeyframeVec>();
    keyframeVec->keyframes.reserve(200);
    status = std::make_shared<LidarStatus>();
    fixedKeyframeChannel = boost::make_shared<FixedKeyframeChannel>();

    pose_w_c = odom = delta = gtsam::Pose3();

    auto edge_gaussian_model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(3) << 0.2, 0.2, 0.2).finished());
    auto surf_gaussian_model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(1) << 0.2).finished());

    edge_noise_model = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(0.58), edge_gaussian_model);
    surf_noise_model = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(0.02), surf_gaussian_model);
    prior_noise_model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished());

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

void LidarSensor::update_keyframe(const ros::Time& cloud_in_time,
                                  pcl::PointCloud<PointT>::Ptr currEdge,
                                  pcl::PointCloud<PointT>::Ptr currSurf,
                                  pcl::PointCloud<PointT>::Ptr currFull)
{
    current_keyframe = std::make_shared<Keyframe>(keyframeVec->keyframes.size(), cloud_in_time, currEdge, currSurf, currFull);
    keyframeVec->keyframes.emplace_back(current_keyframe);
    is_keyframe_next = false;
}

void LidarSensor::getTransToSubmap(pcl::PointCloud<PointT>::Ptr currEdge, pcl::PointCloud<PointT>::Ptr currSurf) {

    if (opti_counter > 2) opti_counter--;

    gtsam::Pose3 odom_prediction = pose_normalize(odom * delta);

    last_odom = odom;
    odom = odom_prediction;

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

            addCornCostFactor(currEdge, submapEdge_copy, kdtree_corn, pose_w_c, factors);
//            std::cout << "addEdgeCostFactor: " << j << " " << tc.count() - t1 << "ms" << std::endl; t1 = tc.count();
            addSurfCostFactor(currSurf, submapSurf_copy, kdtree_surf, pose_w_c, factors);
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
            pcl::PointCloud<PointT>::Ptr currEdge,
            pcl::PointCloud<PointT>::Ptr currSurf,
            pcl::PointCloud<PointT>::Ptr currRaw)
{
    // if is keyframe, append to keyframes vector
    if (is_keyframe_next) {
        down_sampling_voxel(*currRaw, 0.02);
        update_keyframe(cloud_in_time, currEdge, currSurf, currRaw);
    }

    if (!is_init) {
        current_keyframe->set_init();
        updateSubmap();
        is_init = true;
        return gtsam::Pose3();
    }

    down_sampling_voxel(*currEdge, corn_filter_length);
    down_sampling_voxel(*currSurf, surf_filter_length);

    getTransToSubmap(currEdge, currSurf);

    handleRegistration();

    return odom;
}

void LidarSensor::addOdomFactor(int last_index, int curr_index) {

    using namespace gtsam;

    if (last_index < 0) {
        BAGraph.addPrior(X(0), Pose3::identity(), prior_noise_model);
        BAEstimate.insert(X(0), Pose3::identity());
        f_pose_fixed << 0 << endl << Pose3().matrix() << endl;
        std::cout << "init Odom factor!" << std::endl;
        return;
    }

    // read lock
    Pose3 last_pose = keyframeVec->read_pose(last_index);
    Pose3 curr_pose = keyframeVec->read_pose(curr_index);
    Pose3 pose_curr_last = last_pose.between(curr_pose);

    auto information = GetInformationMatrixFromPointClouds(keyframeVec->keyframes[curr_index]->raw,
                                                           keyframeVec->keyframes[last_index]->raw,
                                                           2.0,
                                                           pose_curr_last.matrix());

    f_pose_fixed << curr_index << endl << curr_pose.matrix() << endl << endl;
    f_pose_fixed << information << endl << endl;

    BAGraph.emplace_shared<BetweenFactor<Pose3>>(X(last_index), X(curr_index), pose_curr_last,
//                                                            odometry_noise_model);
                                                            noiseModel::Gaussian::Information(information));
    BAEstimate.insert(X(curr_index), curr_pose);
    printf("add Odom factor between [%d] and [%d]\n", last_index, curr_index);

    updateISAM();

}

void LidarSensor::BALM_backend(const std::vector<Keyframe::Ptr>& keyframes_buf,
                               std::vector<Keyframe::Ptr>& fixedKeyframes_buf) {

    for (const auto& keyframe : keyframes_buf) {
        Timer t_BALM("BALM keyframe_" + std::to_string(keyframe->index));
        TicToc tic;

        gtsam::Pose3 curr_pose = keyframe->pose_world_curr;
        q_odom = Eigen::Quaterniond(curr_pose.rotation().matrix());
        t_odom = Eigen::Vector3d(curr_pose.translation());

        Eigen::Vector3d delta_t(q_last.matrix().transpose() * (t_odom - t_last));
        Eigen::Quaterniond delta_q(q_last.matrix().transpose() * q_odom.matrix());
        q_last = q_odom; t_last = t_odom;

        t_gather_pose = t_gather_pose + q_gather_pose * delta_t;
        q_gather_pose = q_gather_pose * delta_q;

        if (plcount == 0) {
            q_poses.push_back(q_gather_pose);
            t_poses.push_back(t_gather_pose);
        } else {
            t_poses.push_back(t_poses[plcount-1] + q_poses[plcount-1] * t_gather_pose);
            q_poses.push_back(q_poses[plcount-1] * q_gather_pose);
        }

        pl_corn = keyframe->corn_features; pl_surf = keyframe->surf_features; plcount++;
        OCTO_TREE::voxel_windowsize = plcount - window_base;
        q_gather_pose.setIdentity(); t_gather_pose.setZero();

        int frame_head = plcount-1-window_base;
        cut_voxel(surf_map, pl_surf, q_poses[plcount-1].matrix(), t_poses[plcount-1], 0, frame_head, window_size);
        cut_voxel(corn_map, pl_corn, q_poses[plcount-1].matrix(), t_poses[plcount-1], 1, frame_head, window_size);

        for (auto iter=surf_map.begin(); iter!=surf_map.end(); ++iter) {
            if (iter->second->is2opt) {
                iter->second->root_centors.clear();
                iter->second->recut(0, frame_head, iter->second->root_centors);
            }
        }

        for (auto iter=corn_map.begin(); iter!=corn_map.end(); ++iter) {
            if (iter->second->is2opt) {
                iter->second->root_centors.clear();
                iter->second->recut(0, frame_head, iter->second->root_centors);
            }
        }

        if (plcount >= window_base+window_size) {
            // send to opti
            for (int i=0; i<window_size; i++) {
                opt_lsv.so3_poses[i].setQuaternion(q_poses[window_base + i]);
                opt_lsv.t_poses[i] = t_poses[window_base + i];
            }

            // optimize step
            if (window_base != 0) {
                for (auto iter=surf_map.begin(); iter!=surf_map.end(); ++iter) {
                    if (iter->second->is2opt) {
                        iter->second->traversal_opt(opt_lsv);
                    }
                }

                for (auto iter=corn_map.begin(); iter!=corn_map.end(); ++iter) {
                    if (iter->second->is2opt) {
                        iter->second->traversal_opt(opt_lsv);
                    }
                }

                opt_lsv.damping_iter();
            }

            // sub-opti value
            {
                std::unique_lock<std::shared_mutex> ul(keyframeVec->pose_mtx); // write lock
                for (int i=0; i<window_size; i++) {
                    q_poses[window_base + i] = opt_lsv.so3_poses[i].unit_quaternion();
                    t_poses[window_base + i] = opt_lsv.t_poses[i];
                    keyframeVec->keyframes[window_base + i]->pose_world_curr = toPose3(q_poses[window_base + i], t_poses[window_base + i]);
                }
            }

            // optimized pose value
            for (int i = 0; i < margi_size; i++) {
                auto keyframe = keyframeVec->keyframes[window_base + i];
                keyframe->fix();
                fixedKeyframes_buf.push_back(keyframe);
            }

            // update submap here, front end will use
            updateSubmap(window_base);

            for (auto iter=surf_map.begin(); iter!=surf_map.end(); ++iter) {
                if (iter->second->is2opt) {
                    iter->second->root_centors.clear();
                    iter->second->marginalize(0, margi_size, q_poses, t_poses, window_base, iter->second->root_centors);
                }
            }

            for (auto iter=corn_map.begin(); iter!=corn_map.end(); ++iter) {
                if (iter->second->is2opt) {
                    iter->second->root_centors.clear();
                    iter->second->marginalize(0, margi_size, q_poses, t_poses, window_base, iter->second->root_centors);
                }
            }

            window_base += margi_size;
            opt_lsv.free_voxel();
        }
        t_BALM.count();
        f_backend_timecost << tic.toc() << endl;
    }
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

void LidarSensor::updateISAM() {
    // isam update
    isam->update(BAGraph, BAEstimate);
    isam->update();
    isamOptimize = isam->calculateEstimate();

    BAGraph.resize(0);
    BAEstimate.clear();
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

        if (!keyframes_buf.empty() || !factorsBuf.empty()) {

            // if there is new keyframe, clear end signal
            current_latency = 0;

            std::vector<Keyframe::Ptr> fixedKeyframes_buf;
            // from balm backend
            BALM_backend(keyframes_buf, fixedKeyframes_buf);

            // deal with fixed keyframes with factorGraph
            std::lock_guard<std::mutex> lg(fixed_mtx);
            if (!fixedKeyframes_buf.empty() || !factorsBuf.empty()) {
                for (const auto& keyframe: fixedKeyframes_buf) {
                    int curr_index = keyframe->index;
                    addOdomFactor(last_index, curr_index);
                    last_index = curr_index;
                    if (need_loop) {
                        FixedKeyframeBuf.push(keyframe);
                    }
                }
                bool has_loop = false;
                {
                    std::lock_guard<std::mutex> lg(loop_factors_mtx);
                    if (!factorsBuf.empty()) {
                        while (!factorsBuf.empty()) {
                            BAGraph.add(factorsBuf.front());
                            factorsBuf.pop();
                            updateISAM();
                        }
                        has_loop = true;
                    }
                }

                isam->getFactorsUnsafe().saveGraph("/home/ziv/mloam/factor_graph.dot", isamOptimize);
                if (has_loop) {
                    status->status_change = true;
                }
                {   // write lock
                    std::unique_lock<std::shared_mutex> ul(keyframeVec->pose_mtx);
                    for (size_t i = 0; i < isamOptimize.size(); i++) {
                        keyframeVec->keyframes[i]->set_fixed(isamOptimize.at<gtsam::Pose3>(X(i)));
                    }
                }
            }

            // debug
//            auto poseVec = keyframeVec->read_poses(0, keyframeVec->size(), true);
//            {
//                std::ofstream f(nh.param<std::string>("file_save_path", "") + "debug_pose.txt");
//                for (size_t i = 0; i < poseVec.size(); i++) {
//                    f << "i: " << i << "\n" << poseVec[i].matrix() << endl;
//                }
//                f.close();
//            }


        } else {
            current_latency += backend_sleep;
            bool reach_end = current_latency >= end_signal;
            bool under_max_delay = current_latency < 3 * end_signal;
            if (last_index > 0 && reach_end && under_max_delay && !status->is_end) {
                status->is_end = true;
            }
        }

        //sleep 5 ms every time
        std::this_thread::sleep_for(std::chrono::milliseconds(backend_sleep));
    }
}

void _mkdir(const std::string& filename) {
    boost::filesystem::path save_path(filename);
    auto folder_path = save_path.parent_path();
    if (!boost::filesystem::exists(folder_path)) {
        boost::filesystem::create_directories(folder_path);
    }
}

LidarSensor::LidarSensor(int i) : opt_lsv(window_size, filter_num, thread_num), loopDetector(i)
{

    string file_save_path = nh.param<std::string>("file_save_path", "");
    _mkdir(file_save_path+"test.txt");
    f_pose_fixed.open(file_save_path+"lidar"+to_string(i)+"fixed_poses_raw.txt");
    f_backend_timecost.open(file_save_path+ "lidar"+to_string(i) + "backend_timecost.txt");
    cout << "#######################################" << endl;
    cout << "current_lidar: " << i << endl;
    cout << "f_pose_fixed path: " << file_save_path+"lidar"+to_string(i)+"fixed_poses_raw.txt" << endl;
    cout << "f_pose_fixed isopen? " << f_pose_fixed.is_open() << endl;
    cout << "f_backend_timecost path: " << "lidar"+to_string(i) + "backend_timecost.txt" << endl;
    cout << "f_backend_timecost isopen? " << f_backend_timecost.is_open() << endl;
    cout << "#######################################" << endl;



    nh.param<bool>  ("need_loop", need_loop, true);
    nh.param<double>("keyframe_distance_threshold", keyframe_distance_thres, 0.6);
    nh.param<double>("keyframe_angle_threshold",    keyframe_angle_thres,    0.1);
    nh.param<int>   ("window_size", window_size, 6);
    nh.param<int>   ("margi_size",  margi_size,  3);
    nh.param<double>("surf_filter_length", surf_filter_length, 0.4);
    nh.param<double>("corn_filter_length", corn_filter_length, 0.2);

    opt_lsv.set_window_size(window_size);
    initParam();
}
