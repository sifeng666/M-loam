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

//add the corn factor to the Graph factors
void
LidarSensor::addCornCostFactor(const pcl::PointCloud<PointT>::Ptr &pc_in, const pcl::PointCloud<PointT>::Ptr &map_in,
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

    bool need_transform = !point_transform.equals(gtsam::Pose3::identity()); //judge whether the point is need transform
    //each point find nearest
    for (int i = 0; i < (int) pc_in->points.size(); i++) {

        point_temp = pcl::transformPoint(pc_in->points[i], odom_);
        //find 5 nearest point to this point_temp
        kdtree_corn.nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis);

        //all 5 points are closed to point_temp than 1.0
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

//add the surface factor to the graph
void
LidarSensor::addSurfCostFactor(const pcl::PointCloud<PointT>::Ptr &pc_in, const pcl::PointCloud<PointT>::Ptr &map_in,
                               const pcl::KdTreeFLANN<PointT> &kdtree_surf, const gtsam::Pose3 &odom,
                               gtsam::NonlinearFactorGraph &factors, const gtsam::Pose3 &point_transform) {
    int surf_num = 0;
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
//update the submap(used to match) by 2 frame id 通过两个frameid来更新子地图，之后赋值给全局变量submap
//get the frames between time(range_from) to the time(range to) and create the new submap
//then assign the new submap to the lidar class member [submap]
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

//judge whether the nextFrame is a keyFrame  判断下一帧是否关键帧
//use two global variance T_wmap_lastkey and the T_wmap_curr
//keyframe_angle_thres is a assigned variance, default value is 0.1
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
//input: cornFrame,surfFrame,cornMap,surfMap,kdTree * 2, T_w_c(current)
//基于面特征和角特征点的matching，使用LM算法进行优化，最终优化的输出是位姿T_w_c
void LidarSensor::feature_based_scan_matching(const pcl::PointCloud<PointT>::Ptr &corn,
                                              const pcl::PointCloud<PointT>::Ptr &surf,
                                              const pcl::PointCloud<PointT>::Ptr &corns,
                                              const pcl::PointCloud<PointT>::Ptr &surfs,
                                              pcl::KdTreeFLANN<PointT> &kdtree_corn,
                                              pcl::KdTreeFLANN<PointT> &kdtree_surf,
                                              gtsam::Pose3 &T_w_c) {
    kdtree_corn.setInputCloud(corns); //input is current corns map
    kdtree_surf.setInputCloud(surfs); //input is current surfs map

    for (int j = 0; j < opti_counter; j++) {  //opti = optimizer 优化
        gtsam::NonlinearFactorGraph factors; //用于优化的非线性因子图 by gtsam
        gtsam::Values init_values;
        init_values.insert(X(0), T_w_c);

        addCornCostFactor(corn, corns, kdtree_corn, T_w_c, factors); //在因子图中加入角特征点
        addSurfCostFactor(surf, surfs, kdtree_surf, T_w_c, factors); //在因子图中加入面特征点

        gtsam::LevenbergMarquardtParams params; //构建LM算法参数类(相当于g2o options)
        params.setLinearSolverType("MULTIFRONTAL_CHOLESKY");//设置分解算法
        params.setRelativeErrorTol(1e-3);//迭代变化量小于1e-3则退出，收敛值
        params.maxIterations = 4; //最大迭代次数，会影响速度?
        //使用LM算法进行优化，输入包括带有特征的因子图（factors），初始值（init_values）以及参数（params）
        auto result = gtsam::LevenbergMarquardtOptimizer(factors, init_values, params).optimize();
        T_w_c = result.at<gtsam::Pose3>(X(0));//result赋值给T_w_c，是一个变换矩阵？
    }
}

//输入包括新的角特征和面特征点云，总的角特征和面特征点云；输出是当前帧到累积点云的位姿T_w_c
void LidarSensor::point_wise_scan_matching(const pcl::PointCloud<PointT>::Ptr &corn,
                                           const pcl::PointCloud<PointT>::Ptr &surf,
                                           const pcl::PointCloud<PointT>::Ptr &corns,
                                           const pcl::PointCloud<PointT>::Ptr &surfs,
                                           gtsam::Pose3 &T_w_c) {
    Eigen::Matrix4d final;

    //基于目前总的角特征和面特征构造一个总的点云图
    pcl::PointCloud<PointT>::Ptr submap_pcd(new pcl::PointCloud<PointT>);
    *submap_pcd += *corns;
    *submap_pcd += *surfs;
    //基于当前帧的角特征和面特征构造一个点云图
    pcl::PointCloud<PointT>::Ptr curr_pcd(new pcl::PointCloud<PointT>);
    *curr_pcd += *corn;
    *curr_pcd += *surf;
    //调用GICP来进行两帧点云的配准，配准得到的变换矩阵赋值给final；输入的参数包括配准的两帧点云；变换矩阵final；indentity；两点之间的最大距离；阈值；
    bool ok = tools::FastGeneralizedRegistration(curr_pcd, submap_pcd, final, T_w_c.matrix(), 1.0, 1.0);
    if (ok) {
        T_w_c = gtsam::Pose3(final);//转换为gtsam的pose3格式
    } else { //如果调用GICP匹配失败
        std::cout << "GICP fail to match! Use feature-based method!" << std::endl;
        pcl::KdTreeFLANN<PointT> kdtree_corn_submap;
        pcl::KdTreeFLANN<PointT> kdtree_surf_submap;
        //采用gtsam中的LM优化来进行求解
        feature_based_scan_matching(corn, surf, corns, surfs, kdtree_corn_submap, kdtree_surf_submap, T_w_c);
    }
}

//输入的是当前的角特征点云和面特征点云，以及原始点云；生成一个新的当前帧，并计算当前位姿T_w_c作为输出；生成的新的帧插入到N_scans容器中（最近N帧)
gtsam::Pose3 LidarSensor::scan2scan(const ros::Time &cloud_in_time, const pcl::PointCloud<PointT>::Ptr &corn,
                                    const pcl::PointCloud<PointT>::Ptr &surf, const pcl::PointCloud<PointT>::Ptr &raw) {

    //pose_normalize = return gtsam::Pose3(pose.rotation().normalized(), pose.translation());
    auto T_w_c = pose_normalize(T_wodom_curr * T_last_curr);//进行简单的初值计算，curr位姿 = last位姿*变换矩阵
    //异常处理，当输入的角特征点云/面特征点云为空时返回上一帧的简单预测
    if (!corn || !surf) {
        std::cerr << "Error nullptr corn or surf" << std::endl;
        return T_w_c;
    }

    TicToc t_gen_scans;//tictoc计时

    pcl::PointCloud<PointT>::Ptr corn_scans(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr surf_scans(new pcl::PointCloud<PointT>);

    nscans_gen.read(corn_scans, surf_scans); //从最近N帧中读到角特征的点云和面特征的点云

    //ds = downsample，ds_corn是一个voxel_grid体素珊格滤波器，对点云进行下采样，减少点的数量
    ds_corn.setInputCloud(corn_scans);
    ds_corn.filter(*corn_scans);
    ds_surf.setInputCloud(surf_scans);
    ds_surf.filter(*surf_scans);

//    printf("prepare %d scans: %.3f ms\n", n_scans, t_gen_scans.toc());
    //角特征点小于20个或者是面特征点小于100个，则认为特征点太小；返回粗略估计的位姿
    if (corn_scans->size() < 20 || surf_scans->size() < 100) {
        printf("too less features! corn = [%d], surf = [%d], exit!\n", int(corn_scans->size()),
               int(surf_scans->size()));
        return T_w_c;
    }

    TicToc t_odom;
    //基于gtsam的因子图优化。使用LM方法求解T_w_c
    feature_based_scan_matching(corn, surf, corn_scans, surf_scans,
                                kdtree_corn_nscans, kdtree_surf_nscans, T_w_c);
    //更新全局中存储位姿变量
    T_wodom_last = T_wodom_curr;
    T_wodom_curr = T_w_c;
    T_last_curr = pose_normalize(T_wodom_last.between(T_wodom_curr));
    //基于最新的点云信息构建最新的帧，并加入到最近N帧数据结构nscans_gen中
    nscans_gen.add(std::make_shared<Frame>(frameCount++, cloud_in_time, corn, surf, raw, T_wodom_curr));

//    printf("odom, scan to %d scans: %.3f ms\n", n_scans, t_odom.toc());
    return T_w_c;
}

//input current frame of corn&surf pointcloud, output is current frame to the submap's pose
//求解当前角面特征点云对应的帧到submap的变换矩阵
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
    // MapGenerator::generate_cloud generate pcd range from: [begin, end), so need to add 1
    int c_index = current_keyframe->index + 1;
    updateSubmap(c_index - window_size, c_index);
    printf("prepare submap: %.3f ms\n", t_gen_submap.toc());

    submap->mtx.lock(); //加锁保证submap读写一致性
    auto submapCorn_copy = submap->submap_corn->makeShared();
    auto submapSurf_copy = submap->submap_surf->makeShared();
    submap->mtx.unlock();

    TicToc t_mapping;
    //通过method值选择进行T_w_c求解的方法，0是基于特征匹配，1是使用GICP方法
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

//更新odom的位姿，频率为10hz，原始数据得到，无优化，可以加松耦合的IMU作为初值
gtsam::Pose3 LidarSensor::update_odom(const ros::Time &cloud_in_time, const pcl::PointCloud<PointT>::Ptr &corn,
                                      const pcl::PointCloud<PointT>::Ptr &surf, const pcl::PointCloud<PointT>::Ptr &raw,
                                      gtsam::Pose3 &pose_raw) {

    //当前没有帧
    if (frameCount == 0) {
        curr_frame = std::make_shared<Frame>(frameCount++, cloud_in_time, corn, surf, raw, gtsam::Pose3());
        frameChannel->push(curr_frame); //frame channel 是用来进行 odom to mapping 的帧的集合
        nscans_gen.add(curr_frame); //前n帧的集合（默认是6）
        return gtsam::Pose3(); //返回一个初始值的位姿
    }

    double a, b, c;
    TicToc tic;
    ds_corn.setInputCloud(corn); //输入的角特征点云进行下采样
    ds_corn.filter(*corn);
    a = tic.toc(); //用于下面输出降采样使用时间的计时，不是暂停
    tic.tic();
    ds_surf.setInputCloud(surf); //输入的面特征点云进行下采样
    ds_surf.filter(*surf);
    b = tic.toc();
    tic.tic();
    ds_raw.setInputCloud(raw); //输入的原始点云进行下采样
    ds_raw.filter(*raw);
    c = tic.toc();
    tic.tic();
//    printf("Downsampling corn: %.3f ms, surf: %.3f ms, raw: %.3f ms\n", a, b, c);

    pcl::PointCloud<PointT>::Ptr surf_ds2(new pcl::PointCloud<PointT>);
    ds_surf_2.setInputCloud(surf); //面特征点云进行第二次降采样
    ds_surf_2.filter(*surf_ds2);

    pose_raw = scan2scan(cloud_in_time, corn, surf_ds2, raw); //基于降采样的面特征、点特征和原始点云来求解最新的位姿
    pose_raw = pose_normalize(pose_raw); //正则化

    curr_frame = std::make_shared<Frame>(frameCount++, cloud_in_time, corn, surf, raw, pose_raw);//构造新帧
    frameChannel->push(curr_frame); //frame channel 是用来进行 odom to mapping 的帧的集合

    std::lock_guard<std::mutex> lg0(T_lock0), lg1(T_lock1); //构造两个mutex类，传入的mutex对象直接被当前线程锁住
    T_w_curr = pose_normalize(T_w_wmap * T_wmap_wodom * pose_raw); // 10Hz
    return T_w_curr;
}

//更新T_wmap_curr，频率小于10hz，由odom加balm得到，用于加了balm的mapping
gtsam::Pose3 LidarSensor::update_mapping(const Frame::Ptr &frame) {

    //判断下一帧（即将处理的帧）是否是关键帧，如果是的话调用update_keyframe，将当前关键帧插入到关键帧vector中
    if (is_keyframe_next) {
        update_keyframe(frame->cloud_in_time, frame->corn_features, frame->surf_features, frame->raw);
    }
    //没有初始化则进行初始化
    if (!is_init) {
        current_keyframe->set_init(gtsam::Pose3()); //初始位姿是默认0值
        current_keyframe->set_fixed(gtsam::Pose3());
        T_wmap_lastkey = gtsam::Pose3(); //T_wmap由odm的位姿+balm优化后得到，频率<10hz，mapping
//        updateSubmap(0, 1);
        BAKeyframeChannel->push(current_keyframe); //BAKeyframeChannel是用于mapping to BALM
//        factor_graph_opti();
        is_init = true; //初始化标志
        return gtsam::Pose3(); //返回默认值的位姿
    }

    T_wmap_last = T_wmap_curr; //更新T_wmap_last
//    gtsam::Pose3 T_pred = T_wmap_last * T_wmap_delta;
    gtsam::Pose3 T_pred = T_wmap_wodom * frame->pose_w_curr; //基于odom的变换矩阵预测一个变换T_pred

//    while (current_keyframe->index > submap->end + 2 && current_keyframe->index > window_base + 10) {
//        std::this_thread::sleep_for(std::chrono::milliseconds(1));
//    }
    //调用scan2submap来计算T_wmap_curr
    T_wmap_curr = scan2submap(frame->corn_features, frame->surf_features, T_pred, mapping_method);
    T_wmap_curr = pose_normalize(T_wmap_curr);

//    cout << "T_wmap_curr:\n" << T_wmap_curr.matrix() << endl;
    //如果current_keyframe没有初始化（判断依据是valid_frame的数量是否大于0）
    if (!current_keyframe->is_init()) {
        //set_increment函数调用move函数，将pose_的值赋值给pose_last_curr，pose_last_curr = std::move(pose_);
        current_keyframe->set_increment(T_wmap_lastkey.between(T_wmap_curr));//设置增量（变化量）？
        current_keyframe->set_init(T_wmap_curr);//设置初始值

//        cout << "keyframe!!!! " << current_keyframe->index << ", T_wmap_curr:\n" << T_wmap_curr.matrix() << endl;
        f_backend_timecost << T_wmap_curr.matrix() << endl; //f_backend_timecost是std::ofstream，文件输出
        T_wmap_lastkey = T_wmap_curr; //更新新的关键帧，lastkeyt是increment（增量）

        BAKeyframeChannel->push(current_keyframe);//BAKeyframeChannel插入新的关键帧

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

    T_wmap_delta = T_wmap_last.between(T_wmap_curr); //T_wmap_delta应该是变化量，求解last到curr的之间T_wamp（加了balm的odom）的变化量
    is_keyframe_next = nextFrameToBeKeyframe(); //判断下一帧是否是关键帧
    {
        T_lock1.lock();//加锁保证操作原子性
        T_wmap_wodom = pose_normalize(T_wmap_curr * frame->pose_w_curr.inverse()); //计算odom
        T_lock1.unlock();
    }

    return T_w_wmap * T_wmap_curr;//计算加了balm后的odom的位姿并返回
}

//BALM参数的初始化
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

//雷达大类初始化
void LidarSensor::initParam() {

    submap = std::make_shared<Submap>();//构建智能指针
    //降采样的体素过滤器设置参数setLeafSize，即分辨率，数值越小表示voxel的体积越小，密度越大，点的数量越多
    ds_corn.setLeafSize(corn_filter_length, corn_filter_length, corn_filter_length);
    ds_surf.setLeafSize(surf_filter_length, surf_filter_length, surf_filter_length);
    ds_surf_2.setLeafSize(surf_filter_length * 2, surf_filter_length * 2, surf_filter_length * 2);
    ds_raw.setLeafSize(0.1, 0.1, 0.1);

    ds_corn_submap.setLeafSize(corn_filter_length, corn_filter_length, corn_filter_length);
    ds_surf_submap.setLeafSize(surf_filter_length, surf_filter_length, surf_filter_length);

    opti_counter = 10; //优化的次数？

    keyframeVec = std::make_shared<KeyframeVec>(); //智能指针
    keyframeVec->keyframes.reserve(200);//增加vector的capacity而不是size
    status = std::make_shared<LidarStatus>();
    frameChannel = std::make_shared<Channel<Frame::Ptr>>(); //原始数据下odom
    BAKeyframeChannel = std::make_shared<Channel<Keyframe::Ptr>>();//加了BA的odom
    MargiKeyframeChannel = std::make_shared<Channel<Keyframe::Ptr>>();//BALM to loopfactor & gtsam 加了回环检测？
    //using FactorPtr = gtsam::NonlinearFactor::shared_ptr;//非线性优化因子的智能指针
    factorsChannel = std::make_shared<Channel<FactorPtr>>();//loopfactor to gtsam

    //边和平面的高斯模型
    auto edge_gaussian_model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(3) << 0.2, 0.2, 0.2).finished());
    auto surf_gaussian_model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(1) << 0.2).finished());
    //边和平面的噪声模型
    edge_noise_model = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(0.58),
                                                         edge_gaussian_model);
    surf_noise_model = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(0.02),
    //？？的噪声模型                                                     surf_gaussian_model);
    prior_noise_model = gtsam::noiseModel::Diagonal::Variances(
            (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished());

    //isam2优化参数设置
    gtsam::ISAM2Params isam2Params;
    isam2Params.relinearizeThreshold = 0.1;
    isam2Params.relinearizeSkip = 1;
    isam = std::make_shared<gtsam::ISAM2>(isam2Params);

    initBALMParam(); //初始化BALM的参数

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

//更新关键帧函数（假定输入的已经判断是关键帧）
//将当前的点面特征和原始点云构建新的帧，并插入到keyframeVec中，同时将is_keyframe_next
void LidarSensor::update_keyframe(const ros::Time &cloud_in_time,
                                  pcl::PointCloud<PointT>::Ptr currEdge,
                                  pcl::PointCloud<PointT>::Ptr currSurf,
                                  pcl::PointCloud<PointT>::Ptr currFull) {
    current_keyframe = std::make_shared<Keyframe>(keyframeVec->keyframes.size(), cloud_in_time, currEdge, currSurf,
                                                  currFull);
    keyframeVec->keyframes.emplace_back(current_keyframe);
    is_keyframe_next = false;
}
//在给定的两个index的frame之间插入Odom factor，同时更新BAGrpah和BAEstimate
void LidarSensor::addOdomFactor(int last_index, int curr_index) {

    using namespace gtsam;
    //为什么last_index会是0？因为第一帧？
    //可能是初始化
    if (last_index < 0) {
        //BAGraph类型为gtsam::NonlinearFactorGraph，插入Prior
        BAGraph.addPrior(X(0), Pose3::identity(), prior_noise_model);
        //BAEstimate类型为gtsam::Values
        BAEstimate.insert(X(0), Pose3::identity());
        f_pose_fixed << Eigen::Matrix4d::Identity() << endl; //文件输出
//        pcl::io::savePCDFile(file_save_path + "0.pcd", *keyframeVec->keyframes[curr_index]->raw);
        std::cout << "init Odom factor!" << std::endl;
        return;
    }

    // read lock
    gtsam::Pose3 pose_last = keyframeVec->read_pose(last_index, Type::Opti); //读取上一帧优化后的位姿（last）
    gtsam::Pose3 pose_delta = keyframeVec->read_pose(curr_index, Type::Delta);//读取位姿之间的变化量
    gtsam::Pose3 pose_curr = pose_last * pose_delta; //通过优化后的上一帧位姿和到当前位姿的变化量，求解当前位姿
    keyframeVec->at(curr_index)->set_fixed(pose_curr); //将pose_curr赋值给index为currindex的帧中的pose_fixed,并将该关键帧设置为is_fixed

    printf("add Odom factor between [%d] and [%d]\n", last_index, curr_index); //没看懂输出
//    cout << "last_pose:\n" << pose_last.matrix() << endl;
//    cout << "curr pose:\n" << pose_curr.matrix() << endl;

    //GetInformationMatrixFromPointClouds函数在registration.hpp中，作用应该是提取两帧点云中的信息矩阵
    //得到图优化中的信息矩阵，格式为Eigen::Matrix6d，作为参数输入到下面的BAGraph（gtsam中的非线性因子图）中
    auto information = GetInformationMatrixFromPointClouds(keyframeVec->keyframes[curr_index]->raw,
                                                           keyframeVec->keyframes[last_index]->raw,
                                                           2.0,
                                                           pose_delta.matrix());

    f_pose_fixed << pose_curr.matrix() << endl; //文件输出
//    pcl::io::savePCDFile(file_save_path + to_string(curr_index) + ".pcd", *keyframeVec->keyframes[curr_index]->raw);
//    f_pose_fixed << information << endl << endl;

    BAGraph.emplace_shared<BetweenFactor<Pose3>>(X(last_index), X(curr_index),
                                                 pose_delta, noiseModel::Gaussian::Information(information));
    BAEstimate.insert(X(curr_index), pose_curr);
//    printf("add Odom factor between [%d] and [%d]\n", last_index, curr_index);

}

//BALM的后端，应该是将keyframes_buf中的所有帧都优化，并输出为fixedKeyframes_buf中去
void LidarSensor::BALM_backend(const std::vector<Keyframe::Ptr> &keyframes_buf,
                               std::vector<Keyframe::Ptr> &fixedKeyframes_buf) {
    //遍历所有帧并处理
    for (const auto &keyframe : keyframes_buf) {
//        Timer t_BALM("BALM keyframe_" + std::to_string(keyframe->index));
//        TicToc tic;

        gtsam::Pose3 curr_pose = keyframe->pose_odom; //当前帧未优化的位姿
        q_odom = Eigen::Quaterniond(curr_pose.rotation().matrix()); //转换成四元式
        t_odom = Eigen::Vector3d(curr_pose.translation()); //变换矩阵格式？

        //变化量，并更新q和t
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
//回环检测，基于loop_detector.hpp函数进行回环检测，可优化？
void LidarSensor::loop_detect_thread() {

    int last_loop_found_index = -1;

    while (ros::ok()) {
        ////fixedKeyframes_buf是BALM to loopfactor & gtsam
        std::vector<Keyframe::Ptr> fixedKeyframes_buf = MargiKeyframeChannel->get_all();

        std::vector<FactorPtr> factors;
        //遍历所有帧找当前帧的回环
        for (const auto &keyframe: fixedKeyframes_buf) {
            //loopdetector找到回环则返回true,factors可能包含大于一个符合回环的，因此要遍历
            if (loopDetector.loop_detector(keyframeVec, keyframe, factors,
                                           last_loop_found_index)) {
                //遍历插入所有找到的回环因子到factorsChannel中
                for (const auto &factor: factors) {
                    factorsChannel->push(factor);
                }
                //更新status中的全局变量，上一次找到回环的index
                status->last_loop_found_index = max(status->last_loop_found_index, last_loop_found_index);
            }
        }

        //sleep 5 ms every time
        std::this_thread::sleep_for(std::chrono::milliseconds(backend_sleep));
    }

}

//BA优化
void LidarSensor::BA_optimization() {

    int last_index = -1;

    while (ros::ok()) {
        //提取所有的关键帧，keyframes_buf类型为vector；提取后Channel内的queue清空
        auto keyframes_buf = BAKeyframeChannel->get_all();
        //keyframe的vector不为空 或者 回环因子的channel不为空， 则继续处理
        if (!keyframes_buf.empty() || !factorsChannel->empty()) {
            std::vector<Keyframe::Ptr> margiKeyframes;
            // from balm backend
            //调用balm后端，将keyframes_buf中的所有帧都优化，并输出为margiKeyframes
            BALM_backend(keyframes_buf, margiKeyframes);

            // deal with fixed keyframes with factorGraph
            //BALM后的关键帧vector不为空，则遍历
            if (!margiKeyframes.empty()) {
                for (const auto &keyframe: margiKeyframes) {
                    int curr_index = keyframe->index;
                    addOdomFactor(last_index, curr_index); //将关键帧遍历插入到待优化的因子图中
                    last_index = curr_index;
                    if (need_loop) { //默认为true
                        MargiKeyframeChannel->push(keyframe); //将当前帧插入到经过baLM+gtsam因子图优化后的channel里面
                    }
                }
            }
            bool has_loop = false;
            if (!factorsChannel->empty()) { //回环因子图不为空
                auto vec = factorsChannel->get_all(); //提取所有factors，此时factorsChannel中的buff提取后为空
                //遍历插入到BAGraph中
                for (const auto &factor : vec) {
                    BAGraph.add(factor);
                }
                has_loop = true;
            }

            // isam update
            isam->update(BAGraph, BAEstimate);
            isam->update();
            //isamOptimize类型为gtsam::Values，初始值容器
            isamOptimize = isam->calculateEstimate();

            //重制BAGraph和BAEstimate
            BAGraph.resize(0);
            BAEstimate.clear();

            //上面通过factorsChannels来判断是否具有回环
            if (has_loop) {
                status->status_change = true;
            }

            {   // write lock
                std::unique_lock<std::shared_mutex> ul(keyframeVec->pose_mtx);
                for (size_t i = 0; i < isamOptimize.size(); i++) {
                    //将isamOptimize的结果赋值给keyframeVec对应序号的keyframe的位姿（同时会设置为fixed）
                    keyframeVec->at(i)->set_fixed(isamOptimize.at<gtsam::Pose3>(X(i)));
                }
            }
            //提取优化前后的位姿
            auto T_opti = keyframeVec->read_pose(isamOptimize.size() - 1, true);
            auto T_before = keyframeVec->read_pose(isamOptimize.size() - 1, false);
            T_lock0.lock();
            T_w_wmap = pose_normalize(T_opti * T_before.inverse()); //？？
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

//lidarSensor的初始化函数
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
