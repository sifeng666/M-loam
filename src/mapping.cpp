//
// Created by ziv on 2020/10/20.
//

#include "helper.h"




class Mapping {
public:

    // 用Mapping的位姿w_curr，将Lidar坐标系下的点变换到world坐标系下
    void pointAssociateToMap(PointT const *const pi, PointT *const po) {

        Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
        Eigen::Vector3d point_w = q_curr2map * point_curr + t_curr2map;
        po->x = point_w.x();
        po->y = point_w.y();
        po->z = point_w.z();
        po->intensity = pi->intensity;

    }

    void pointCloudAssociateToMap(const pcl::PointCloud<PointT>::Ptr& cloud_in, pcl::PointCloud<PointT>::Ptr& cloud_out) {

        cloud_out->resize(cloud_in->size());

        size_t j = 0;

        for (auto& pi : cloud_in->points) {
            Eigen::Vector3d point_curr(pi.x, pi.y, pi.z);
            Eigen::Vector3d point_w = q_curr2map * point_curr + t_curr2map;
            PointT po;
            po.x = point_w.x();
            po.y = point_w.y();
            po.z = point_w.z();
            po.intensity = pi.intensity;
            cloud_out->points[j++] = po;
        }

        cloud_out->is_dense = true;
        cloud_out->header = cloud_in->header;

    }

    void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudCornerLast2) {
        std::lock_guard lockGuard(mBuf);
        cornerLastBuf.push(laserCloudCornerLast2);
    }

    void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudSurfLast2) {
        std::lock_guard lockGuard(mBuf);
        surfLastBuf.push(laserCloudSurfLast2);
    }

    void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullRes2) {
        std::lock_guard lockGuard(mBuf);
        fullResBuf.push(laserCloudFullRes2);
    }

    // receive odomtry
    void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr &laserOdometry) {
        {
            std::lock_guard lockGuard(mBuf);
            odometryBuf.push(laserOdometry);
        }

        // high frequence publish
        Eigen::Quaterniond q_curr2odom;
        Eigen::Vector3d t_curr2odom;
        q_curr2odom.x() = laserOdometry->pose.pose.orientation.x;
        q_curr2odom.y() = laserOdometry->pose.pose.orientation.y;
        q_curr2odom.z() = laserOdometry->pose.pose.orientation.z;
        q_curr2odom.w() = laserOdometry->pose.pose.orientation.w;
        t_curr2odom.x() = laserOdometry->pose.pose.position.x;
        t_curr2odom.y() = laserOdometry->pose.pose.position.y;
        t_curr2odom.z() = laserOdometry->pose.pose.position.z;

        // 为了保证LOAM整体的实时性，防止Mapping线程耗时>100ms导致丢帧，用上一次的增量wmap_wodom来更新
        // Odometry的位姿，旨在用Mapping位姿的初始值（也可以理解为预测值）来实时输出，进而实现LOAM整体的实时性
        Eigen::Quaterniond q_w_curr = q_odom2map * q_curr2odom;
        Eigen::Vector3d t_w_curr = q_odom2map * t_curr2odom + t_odom2map;

        nav_msgs::Odometry odomAftMapped;
        odomAftMapped.header.frame_id = "/camera_init";
        odomAftMapped.child_frame_id = "/aft_mapped";
        odomAftMapped.header.stamp = laserOdometry->header.stamp;
        odomAftMapped.pose.pose.orientation.x = q_w_curr.x();
        odomAftMapped.pose.pose.orientation.y = q_w_curr.y();
        odomAftMapped.pose.pose.orientation.z = q_w_curr.z();
        odomAftMapped.pose.pose.orientation.w = q_w_curr.w();
        odomAftMapped.pose.pose.position.x = t_w_curr.x();
        odomAftMapped.pose.pose.position.y = t_w_curr.y();
        odomAftMapped.pose.pose.position.z = t_w_curr.z();
        pubOdomAftMappedHighFrec.publish(odomAftMapped);
    }

    Mapping(): nh("~") {

        laserCloudCornerLast.reset(new pcl::PointCloud<PointT>());
        laserCloudSurfLast.reset(new pcl::PointCloud<PointT>());

        q_curr2odom = Eigen::Quaterniond(1, 0, 0, 0);
        t_curr2odom = Eigen::Vector3d(0, 0, 0);

        q_odom2map = Eigen::Quaterniond(1, 0, 0, 0);
        t_odom2map = Eigen::Vector3d(0, 0, 0);

        q_curr2map = Eigen::Quaterniond(1, 0, 0, 0);
        t_curr2map = Eigen::Vector3d(0, 0, 0);

        subLaserCloudCornerLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 100, &Mapping::laserCloudCornerLastHandler, this);

        subLaserCloudSurfLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 100, &Mapping::laserCloudSurfLastHandler, this);

        subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/laser_odom_to_init", 100, &Mapping::laserOdometryHandler, this);

        subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_3", 100, &Mapping::laserCloudFullResHandler, this);

        pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 100);

        pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_map", 100);

        pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_registered", 100);

        pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 100);

        pubOdomAftMappedHighFrec = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init_high_frec", 100);

        pubLaserAfterMappedPath = nh.advertise<nav_msgs::Path>("/aft_mapped_path", 100);
    }

private:

    ros::NodeHandle nh;

    double timeLaserCloudCornerLast = 0;
    double timeLaserCloudSurfLast = 0;
    double timeLaserCloudFullRes = 0;
    double timeLaserOdometry = 0;

    // input: from odom
    pcl::PointCloud<PointT>::Ptr laserCloudCornerLast;
    pcl::PointCloud<PointT>::Ptr laserCloudSurfLast;

    // Mapping线程估计的frame在world坐标系的位姿P,因为Mapping的算法耗时很有可能会超过100ms，所以
    // 这个位姿P不是实时的，LOAM最终输出的实时位姿P_realtime,需要Mapping线程计算的相对低频位姿和
    // Odometry线程计算的相对高频位姿做整合，详见后面laserOdometryHandler函数分析。此外需要注意
    // 的是，不同于Odometry线程，这里的位姿P，即q_w_curr和t_w_curr，本身就是匹配时的优化变量。
//    Eigen::Map<Eigen::Quaterniond> q_w_curr(parameters);
//    Eigen::Map<Eigen::Vector3d> t_w_curr(parameters + 4);
    Eigen::Quaterniond q_curr2map;
    Eigen::Vector3d t_curr2map;

    // 下面的两个变量是world坐标系下的Odometry计算的位姿和Mapping计算的位姿之间的增量（也即变换，transformation）
    // wmap_odom * wodom_curr = wmap_curr(即前面的q/t_w_curr)
    // transformation between odom's world and map's world frame
//    Eigen::Quaterniond q_wmap_wodom(1, 0, 0, 0);
//    Eigen::Vector3d t_wmap_wodom(0, 0, 0);
    Eigen::Quaterniond q_odom2map;
    Eigen::Vector3d t_odom2map;

    // Odometry线程计算的frame在world坐标系的位姿
//    Eigen::Quaterniond q_wodom_curr(1, 0, 0, 0);
//    Eigen::Vector3d t_wodom_curr(0, 0, 0);
    Eigen::Quaterniond q_curr2odom;
    Eigen::Vector3d t_curr2odom;

    // T_o2m * T_c2o = T_c2m

    std::queue<sensor_msgs::PointCloud2ConstPtr> cornerLastBuf;
    std::queue<sensor_msgs::PointCloud2ConstPtr> surfLastBuf;
    std::queue<sensor_msgs::PointCloud2ConstPtr> fullResBuf;
    std::queue<nav_msgs::Odometry::ConstPtr> odometryBuf;
    std::mutex mBuf;

    ros::Publisher pubLaserCloudSurround, pubLaserCloudMap, pubLaserCloudFullRes, pubOdomAftMapped, pubOdomAftMappedHighFrec, pubLaserAfterMappedPath;
    nav_msgs::Path laserAfterMappedPath;

    ros::Subscriber subLaserCloudCornerLast;
    ros::Subscriber subLaserCloudSurfLast;
    ros::Subscriber subLaserOdometry;
    ros::Subscriber subLaserCloudFullRes;


};