//
// Created by ziv on 2020/10/12.
//

#include "helper.h"

#include "registrations.h"


ros::Rate rate(100);

class Odometry {
public:

    void undistortPointCloud(const pcl::PointCloud<PointT>::Ptr& cloud_in, pcl::PointCloud<PointT>::Ptr& cloud_out) {

        if (!need_undisortion) {
            cloud_out = cloud_in->makeShared();
            return;
        }

        cloud_out->resize(cloud_in->size());

        Eigen::Quaterniond q_last_curr(T_last.block<3, 3>(0, 0));
        Eigen::Vector3d t_last_curr(T_last.block<0, 3>(0, 3));

        Eigen::Quaterniond q_point_last;
        Eigen::Vector3d t_point_last, point_, un_point;

        double s;
        size_t j = 0;

        for (auto& point : cloud_in->points) {
            s = (point.intensity - int(point.intensity)) * SCAN_FREQUENCY;

            q_point_last = Eigen::Quaterniond::Identity().slerp(s, q_last_curr);
            t_point_last = s * t_last_curr;
            point_ = Eigen::Vector3d(point.x, point.y, point.z);
            un_point = q_point_last * point_ + t_point_last;

            PointT p;
            p.x = un_point.x();
            p.y = un_point.y();
            p.z = un_point.z();
            p.intensity = point.intensity;
            cloud_out->points[j++] = p;
        }

        cloud_out->is_dense = true;
        cloud_out->header = cloud_in->header;

    }

//    // undistort lidar point
//    void TransformToStart(PointT const *const point_in, PointT *const point_out) {
//        //interpolation ratio
//        double s;
//        if (need_undisortion)
//            s = (point_in->intensity - int(point_in->intensity)) * SCAN_FREQUENCY;
//        else
//            s = 1.0;
//        Eigen::Quaterniond q_last_curr(T_last.block<3, 3>(0, 0));
//        Eigen::Vector3d t_last_curr(T_last.block<0, 3>(0, 3));
//
//        Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, q_last_curr);
//        Eigen::Vector3d t_point_last = s * t_last_curr;
//        Eigen::Vector3d point(point_in->x, point_in->y, point_in->z);
//        Eigen::Vector3d un_point = q_point_last * point + t_point_last;
//
//        point_out->x = un_point.x();
//        point_out->y = un_point.y();
//        point_out->z = un_point.z();
//        point_out->intensity = point_in->intensity;
//    }

    //receive /laser_cloud_sharp
    void laserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsSharp2) {
        std::lock_guard lockGuard(mBuf);
        cornerSharpBuf.push(cornerPointsSharp2);
    }

    //receive /laser_cloud_less_sharp
    void laserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsLessSharp2) {
        std::lock_guard lockGuard(mBuf);
        cornerLessSharpBuf.push(cornerPointsLessSharp2);
    }

    //receive /laser_cloud_flat
    void laserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsFlat2) {
        std::lock_guard lockGuard(mBuf);
        surfFlatBuf.push(surfPointsFlat2);
    }

    //receive /laser_cloud_less_flat
    void laserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsLessFlat2) {
        std::lock_guard lockGuard(mBuf);
        surfLessFlatBuf.push(surfPointsLessFlat2);
    }

    //receive /velodyne_cloud_2
    void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullRes2) {
        std::lock_guard lockGuard(mBuf);
        fullPointsBuf.push(laserCloudFullRes2);
    }

    //receive /
    void laserCloudFilteredHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFiltered) {
        std::lock_guard lockGuard(mBuf);
        filteredPointsBuf.push(laserCloudFiltered);
    }

    void run() {

        while (ros::ok()) {

            ros::spinOnce();

            if (!cornerSharpBuf.empty() && !cornerLessSharpBuf.empty() &&
                !surfFlatBuf.empty() && !surfLessFlatBuf.empty() &&
                !fullPointsBuf.empty() && !filteredPointsBuf.empty()) {

                timeCornerPointsSharp = cornerSharpBuf.front()->header.stamp.toSec();
                timeCornerPointsLessSharp = cornerLessSharpBuf.front()->header.stamp.toSec();
                timeSurfPointsFlat = surfFlatBuf.front()->header.stamp.toSec();
                timeSurfPointsLessFlat = surfLessFlatBuf.front()->header.stamp.toSec();
                timeLaserCloudFullRes = fullPointsBuf.front()->header.stamp.toSec();
                timeLaserCloudFiltered = filteredPointsBuf.front()->header.stamp.toSec();

                if (timeCornerPointsSharp != timeLaserCloudFullRes ||
                    timeCornerPointsLessSharp != timeLaserCloudFullRes ||
                    timeSurfPointsFlat != timeLaserCloudFullRes ||
                    timeSurfPointsLessFlat != timeLaserCloudFullRes ||
                    timeLaserCloudFiltered != timeLaserCloudFullRes) {

                    printf("unsync messeage!");
                    ROS_BREAK();
                }

                {
                    std::lock_guard lockGuard(mBuf);
                    cornerPointsSharp->clear();
                    pcl::fromROSMsg(*cornerSharpBuf.front(), *cornerPointsSharp);
                    cornerSharpBuf.pop();

                    cornerPointsLessSharp->clear();
                    pcl::fromROSMsg(*cornerLessSharpBuf.front(), *cornerPointsLessSharp);
                    cornerLessSharpBuf.pop();

                    surfPointsFlat->clear();
                    pcl::fromROSMsg(*surfFlatBuf.front(), *surfPointsFlat);
                    surfFlatBuf.pop();

                    surfPointsLessFlat->clear();
                    pcl::fromROSMsg(*surfLessFlatBuf.front(), *surfPointsLessFlat);
                    surfLessFlatBuf.pop();

                    laserCloudFullRes->clear();
                    pcl::fromROSMsg(*fullPointsBuf.front(), *laserCloudFullRes);
                    fullPointsBuf.pop();

                    laserCloudFiltered->clear();
                    pcl::fromROSMsg(*filteredPointsBuf.front(), *laserCloudFiltered);
                    filteredPointsBuf.pop();
                }

                Timer t_whole("running");

                for (int reg_counter = 0; reg_counter < 2; reg_counter++) {



                }

            }

        }

    }


    Odometry() : nh("~") {

        // init all T to identity
        T_curr = T_last = T_sharp_curr = T_sharp_last = T_flat_curr = T_flat_last = T_filtered_curr = T_filtered_last = Eigen::Matrix4d::Identity();

        need_undisortion = true;

        cornerPointsSharp.reset(new pcl::PointCloud<PointT>());
        cornerPointsLessSharp.reset(new pcl::PointCloud<PointT>());
        surfPointsFlat.reset(new pcl::PointCloud<PointT>());
        surfPointsLessFlat.reset(new pcl::PointCloud<PointT>());
        laserCloudCornerLast.reset(new pcl::PointCloud<PointT>());
        laserCloudSurfLast.reset(new pcl::PointCloud<PointT>());
        laserCloudFullRes.reset(new pcl::PointCloud<PointT>());
        laserCloudFiltered.reset(new pcl::PointCloud<PointT>());

        subCornerPointsSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100, &Odometry::laserCloudSharpHandler, this);

        subCornerPointsLessSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100, &Odometry::laserCloudLessSharpHandler, this);

        subSurfPointsFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100, &Odometry::laserCloudFlatHandler, this);

        subSurfPointsLessFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100, &Odometry::laserCloudLessFlatHandler, this);

        subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100, &Odometry::laserCloudFullResHandler, this);

        subLaserCloudFiltered = nh.subscribe<sensor_msgs::PointCloud2>("/filtered_points", 100, &Odometry::laserCloudFilteredHandler, this);

        registration.set_ndt_omp_reg(); // set to pclomp's ndt
        reg = registration.getRegistration();



    }

private:

    ros::NodeHandle nh;

    Eigen::Matrix4d T_curr;
    Eigen::Matrix4d T_last;

    Eigen::Matrix4d T_sharp_curr;
    Eigen::Matrix4d T_sharp_last;

    Eigen::Matrix4d T_flat_curr;
    Eigen::Matrix4d T_flat_last;

    Eigen::Matrix4d T_filtered_curr;
    Eigen::Matrix4d T_filtered_last;

//    // Lidar Odometry线程估计的frame在world坐标系的位姿P，Transformation from current frame to world frame
//    Eigen::Quaterniond q_w_curr;
//    Eigen::Vector3d t_w_curr;
//    // 点云特征匹配时的优化变量
//    double para_q[4] = {0, 0, 0, 1};
//    double para_t[3] = {0, 0, 0};
//    // 下面的2个分别是优化变量para_q和para_t的映射：表示的是两个world坐标系下的位姿P之间的增量，例如△P = P0.inverse() * P1
//    Eigen::Map<Eigen::Quaterniond> q_last_curr;
//    Eigen::Map<Eigen::Vector3d> t_last_curr;

    std::queue<sensor_msgs::PointCloud2ConstPtr> cornerSharpBuf;
    std::queue<sensor_msgs::PointCloud2ConstPtr> cornerLessSharpBuf;
    std::queue<sensor_msgs::PointCloud2ConstPtr> surfFlatBuf;
    std::queue<sensor_msgs::PointCloud2ConstPtr> surfLessFlatBuf;
    std::queue<sensor_msgs::PointCloud2ConstPtr> fullPointsBuf;
    std::queue<sensor_msgs::PointCloud2ConstPtr> filteredPointsBuf;
    std::mutex mBuf;

    const int N_SCANS = 16;
    const int SCAN_FREQUENCY = 10;

    int frameCount = 0;
    bool need_undisortion = false;

    // subscriber topic from preprocessing
    ros::Subscriber subCornerPointsSharp;
    ros::Subscriber subCornerPointsLessSharp;
    ros::Subscriber subSurfPointsFlat;
    ros::Subscriber subSurfPointsLessFlat;
    ros::Subscriber subLaserCloudFullRes;
    ros::Subscriber subLaserCloudFiltered;

    double timeCornerPointsSharp = 0;
    double timeCornerPointsLessSharp = 0;
    double timeSurfPointsFlat = 0;
    double timeSurfPointsLessFlat = 0;
    double timeLaserCloudFullRes = 0;
    double timeLaserCloudFiltered = 0;

    pcl::PointCloud<PointT>::Ptr cornerPointsSharp;
    pcl::PointCloud<PointT>::Ptr cornerPointsLessSharp;
    pcl::PointCloud<PointT>::Ptr surfPointsFlat;
    pcl::PointCloud<PointT>::Ptr surfPointsLessFlat;

    pcl::PointCloud<PointT>::Ptr laserCloudCornerLast;
    pcl::PointCloud<PointT>::Ptr laserCloudSurfLast;
    pcl::PointCloud<PointT>::Ptr laserCloudFullRes;
    pcl::PointCloud<PointT>::Ptr laserCloudFiltered;

    Registration::pclRegPtr reg;
    Registration registration;


};

int main(int argc, char **argv) {

    ros::init(argc, argv, "laserOdometry");

    Odometry odometry;

}
