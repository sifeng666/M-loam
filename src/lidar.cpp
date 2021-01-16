//
// Created by ziv on 2020/12/31.
//

#include "lidar.h"

gtsam::Pose3 pose_normalize(const gtsam::Pose3& pose) {
    return gtsam::Pose3(pose.rotation().normalized(), pose.translation());
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

void LidarMsgReader::lock() {
    pcd_msg_mtx.lock();
}

void LidarMsgReader::unlock() {
    pcd_msg_mtx.unlock();
}

void
LidarSensor::downsampling(const pcl::PointCloud<PointT>::Ptr &input, pcl::PointCloud<PointT> &output, FeatureType featureType) {
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

int
LidarSensor::addEdgeCostFactor(const pcl::PointCloud<PointT>::Ptr &pc_in, const pcl::PointCloud<PointT>::Ptr &map_in,
                                   const pcl::KdTreeFLANN<PointT>::Ptr &kdtreeEdge, const gtsam::Pose3 &odom,
                                   gtsam::NonlinearFactorGraph &factors) {
    int corner_num = 0;
    PointT point_temp;
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    Eigen::Affine3d odom_(odom.matrix());

    for (int i = 0; i < (int) pc_in->points.size(); i++) {

        point_temp = pcl::transformPoint(pc_in->points[i], odom_);
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
                        X(0), curr_point, point_a, point_b, edge_noise_model);
                corner_num++;
            }
        }
    }

    if (corner_num < 20) {
        printf("not enough correct points\n");
    }
    return corner_num;
}

int
LidarSensor::addSurfCostFactor(const pcl::PointCloud<PointT>::Ptr &pc_in, const pcl::PointCloud<PointT>::Ptr &map_in,
                                   const pcl::KdTreeFLANN<PointT>::Ptr &kdtreeSurf, const gtsam::Pose3 &odom,
                                   gtsam::NonlinearFactorGraph &factors) {
    int surf_num=0;
    PointT point_temp;
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    Eigen::Affine3d odom_(odom.matrix());

    for (int i = 0; i < (int)pc_in->points.size(); i++) {

        point_temp = pcl::transformPoint(pc_in->points[i], odom_);
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
                        X(0), curr_point, norm, negative_OA_dot_norm, surf_noise_model);
                surf_num++;
            }
        }

    }

    if (surf_num < 20) {
        printf("not enough correct points\n");
    }

    return surf_num;
}

void LidarSensor::updateSubmap() {
    // update only when new keyframe goes in with correct pose
//    if (current_keyframe->valid_frames == 1) {
        size_t size = keyframeVec->keyframes.size();
        size_t start = size < SUBMAP_LEN ? 0 : size - SUBMAP_LEN;

        submapEdge = MapGenerator::generate_cloud(keyframeVec, start, size, FeatureType::Edge);
        submapSurf = MapGenerator::generate_cloud(keyframeVec, start, size, FeatureType::Surf);
        downsampling(submapEdge, *submapEdge, FeatureType::Edge);
        downsampling(submapSurf, *submapSurf, FeatureType::Surf);
//    }
}

bool LidarSensor::nextFrameToBeKeyframe() {
    Eigen::Isometry3d T_delta((current_keyframe->pose_world_curr.inverse() * odom).matrix());
    Eigen::Quaterniond q_delta(T_delta.rotation().matrix());
    q_delta.normalize();
    Eigen::Vector3d eulerAngle = q_delta.toRotationMatrix().eulerAngles(2, 1, 0);

    bool isKeyframe = min(abs(eulerAngle.x()), abs(abs(eulerAngle.x()) - M_PI)) > KEYFRAME_ANGLE_THRES ||
                      min(abs(eulerAngle.y()), abs(abs(eulerAngle.y()) - M_PI)) > KEYFRAME_ANGLE_THRES ||
                      min(abs(eulerAngle.z()), abs(abs(eulerAngle.z()) - M_PI)) > KEYFRAME_ANGLE_THRES ||
                      T_delta.translation().norm() > KEYFRAME_DIST_THRES;
    return isKeyframe;
}

void LidarSensor::initOdomFactor() {
    BAGraph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0), gtsam::Pose3(), prior_noise_model);
    BAEstimate.insert(X(0), gtsam::Pose3());
}

void LidarSensor::addOdomFactor() {
    int index = current_keyframe->index;
    // read lock
    gtsam::Pose3 last_pose = keyframeVec->read_pose(index - 1);

    BAGraph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(index - 1), X(index),
                                                               last_pose.between(current_keyframe->pose_world_curr),
                                                               odometry_noise_model);
    BAEstimate.insert(X(index), current_keyframe->pose_world_curr);
}

void LidarSensor::handleRegistration() {

    if (!current_keyframe->is_init()) {

        Keyframe::Ptr last_keyframe = keyframeVec->keyframes[current_keyframe->index - 1];

        // push to loopDetectBuf
        BA_mtx.lock();
        BAKeyframeBuf.push(last_keyframe);
        BA_mtx.unlock();

        current_keyframe->set_init(odom);

        addOdomFactor();

        updatePoses();

        odom = keyframeVec->read_pose(current_keyframe->index);

        updateSubmap();

    } else {

        is_keyframe_next = nextFrameToBeKeyframe();

        current_keyframe->add_frame();

    }
}

void LidarSensor::updatePoses() {

    // isam update
    isam->update(BAGraph, BAEstimate);
    isam->update();

    if (status->status_change) {
        isam->update();
        isam->update();
        isam->update();
        status->status_change = false;
    }

    BAGraph.resize(0);
    BAEstimate.clear();

    isamOptimize = isam->calculateEstimate();
    isam->getFactorsUnsafe().saveGraph("/home/ziv/mloam/factor_graph.dot", isamOptimize);

//    // write lock
    std::unique_lock<std::shared_mutex> ul(keyframeVec->pose_mtx);

    for (size_t i = 0; i < isamOptimize.size(); i++) {
        keyframeVec->keyframes[i]->pose_world_curr = isamOptimize.at<gtsam::Pose3>(X(i));
    }

}

void LidarSensor::initParam() {

    submapEdge = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
    submapSurf = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
    kdtreeEdgeSubmap = pcl::KdTreeFLANN<PointT>::Ptr(new pcl::KdTreeFLANN<PointT>());
    kdtreeSurfSubmap = pcl::KdTreeFLANN<PointT>::Ptr(new pcl::KdTreeFLANN<PointT>());

    ne.setNumberOfThreads(std::thread::hardware_concurrency());

    opti_counter = 12;

    keyframeVec = boost::make_shared<KeyframeVec>();
    keyframeVec->keyframes.reserve(200);
    status = boost::make_shared<LidarStatus>();

    downSizeFilterEdge.setLeafSize(map_resolution / 2, map_resolution / 2, map_resolution / 2);
    downSizeFilterSurf.setLeafSize(map_resolution, map_resolution, map_resolution);
    downSizeFilterNor.setLeafSize(0.1, 0.1, 0.1);

    pose_w_c = odom = delta = gtsam::Pose3::identity();

    edge_gaussian_model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(3) << 0.2, 0.2, 0.2).finished());
    surf_gaussian_model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(1) << 0.2).finished());

    edge_noise_model = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(0.58), edge_gaussian_model);
    surf_noise_model = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(0.02), surf_gaussian_model);

    prior_noise_model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished());
    odometry_noise_model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-2, 1e-2, 1e-1).finished());

    gtsam::ISAM2Params isam2Params;
    isam2Params.relinearizeThreshold = 0.1;
    isam2Params.relinearizeSkip = 1;
    isam = std::make_shared<gtsam::ISAM2>(isam2Params);
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
    current_keyframe = boost::make_shared<Keyframe>(keyframeVec->keyframes.size(), cloud_in_time, currEdge, currSurf, currFull);
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

    if (submapEdge->size() > 10 && submapSurf->size() > 50) {

        kdtreeEdgeSubmap->setInputCloud(submapEdge);
        kdtreeSurfSubmap->setInputCloud(submapSurf);

        for (int j = 0; j < opti_counter; j++) {
//            TimeCounter tc; auto t1 = tc.count();
            gtsam::NonlinearFactorGraph factors;
            gtsam::Values init_values;
            init_values.insert(state_key, pose_w_c);

            addEdgeCostFactor(currEdge, submapEdge, kdtreeEdgeSubmap, pose_w_c, factors);
//            std::cout << "addEdgeCostFactor: " << j << " " << tc.count() - t1 << "ms" << std::endl; t1 = tc.count();
            addSurfCostFactor(currSurf, submapSurf, kdtreeSurfSubmap, pose_w_c, factors);
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
            pcl::PointCloud<PointT>::Ptr currLessEdge,
            pcl::PointCloud<PointT>::Ptr currLessSurf,
            pcl::PointCloud<PointT>::Ptr currRaw)
{
    // if is keyframe, append to keyframes vector
    if (is_keyframe_next) {
        update_keyframe(cloud_in_time, currLessEdge, currLessSurf, currRaw);
    }

    if (!is_init) {
        current_keyframe->set_init(gtsam::Pose3());
        updateSubmap();
        initOdomFactor();
        is_init = true;
        return gtsam::Pose3();
    }
//    getTransToSubmap(currEdge, currSurf);
    downsampling(currLessEdge, *currLessEdge, FeatureType::Edge);
    downsampling(currLessSurf, *currLessSurf, FeatureType::Edge);
    getTransToSubmap(currLessEdge, currLessSurf);

    handleRegistration();

    return odom;
}

KeyframeVec::Ptr LidarSensor::get_keyframeVec() const {
    return keyframeVec;
}

void LidarSensor::BA_optimization() {
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
                if (need_loop) {
                    loopDetector.loop_detector(keyframeVec, keyframe, factors);
                }
            }

            // todo use openmp
            for (const auto& factor: factors) {
                BAGraph.add(factor);
            }

            status->status_change = (factors.size() > 0);
        }

        //sleep 20 ms every time
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

std::string LidarSensor::get_lidar_name() const {
    return lidar_name;
}

void LidarSensor::setName(const string &lidar_name_) {
    lidar_name = lidar_name_;
}

LidarStatus::Ptr LidarSensor::get_status() const {
    return status;
}


LidarSensor::LidarSensor() {
    need_loop            = nh.param<bool>("need_loop", true);
    map_resolution       = nh.param<double>("map_resolution", 0.2);
    KEYFRAME_DIST_THRES  = nh.param<double>("keyframe_dist_threshold", 0.6);
    KEYFRAME_ANGLE_THRES = nh.param<double>("keyframe_angle_threshold", 0.1);
    SUBMAP_LEN           = nh.param<int>("submap_len", 6);
    initParam();
}


LidarSensor::~LidarSensor() {

}