//
// Created by ziv on 2020/10/12.
//

#include "helper.h"
#include "pcl_utils.h"
#include "lidarFactor.hpp"


class Odometry {
public:

    void undistortPointCloud(const pcl::PointCloud<PointT>::Ptr& cloud_in, pcl::PointCloud<PointT>::Ptr& cloud_out) {

        if (!need_undisortion) {
            cloud_out = cloud_in->makeShared();
            return;
        }

        cloud_out->resize(cloud_in->size());

        Eigen::Quaterniond q_point_last;
        Eigen::Vector3d t_point_last, point_, un_point;

        double s;
        size_t j = 0;

        for (auto& point : cloud_in->points) {
            s = (point.intensity - int(point.intensity)) * SCAN_FREQUENCY;

            q_point_last = Eigen::Quaterniond::Identity().slerp(s, q_curr2last);
            t_point_last = s * t_curr2last;
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

//    //receive /filtered_points
//    void laserCloudFilteredHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFiltered) {
//        std::lock_guard lockGuard(mBuf);
//        filteredPointsBuf.push(laserCloudFiltered);
//    }

    void downsample(const pcl::PointCloud<PointT>::Ptr &cloud_in, pcl::PointCloud<PointT>::Ptr& cloud_out) const {
        if(!downsample_filter) {
            return;
        }

        downsample_filter->setInputCloud(cloud_in);
        downsample_filter->filter(*cloud_out);
        cloud_out->header = cloud_in->header;
    }

    void outlier_removal(const pcl::PointCloud<PointT>::Ptr &cloud_in, pcl::PointCloud<PointT>::Ptr& cloud_out) const {
        if(!outlier_removal_filter) {
            return;
        }

        outlier_removal_filter->setInputCloud(cloud_in);
        outlier_removal_filter->filter(*cloud_out);
        cloud_out->header = cloud_in->header;
    }

    bool matching(const pcl::PointCloud<PointT>::Ptr& filteredPoints, const pcl::PointCloud<PointT>::Ptr& filteredPointsLast, Eigen::Matrix4f& trans) {

        registration->setInputTarget(filteredPointsLast);
        registration->setInputSource(filteredPoints);

        pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
        registration->align(*aligned);

        if(!registration->hasConverged()) {
            ROS_INFO_STREAM("scan matching has not converged!!");
            return false;
        }

        trans = registration->getFinalTransformation();
        return true;
    }



//    Eigen::Matrix4f matching(const ros::Time& stamp, const pcl::PointCloud<PointT>::Ptr& cloud) {
//
//        if (!keyframe) {
//            T_curr2world.setIdentity();
//            T_curr2last.setIdentity();
//            keyframe_stamp = stamp;
//            keyframe.reset(new pcl::PointCloud<PointT>());
//            downsample(cloud, keyframe);
//            registration->setInputTarget(keyframe);
//            return Eigen::Matrix4f::Identity();
//        }
//
//        pcl::PointCloud<PointT>::Ptr downsample_cloud(new pcl::PointCloud<PointT>());
//        downsample(cloud, downsample_cloud);
//        registration->setInputSource(downsample_cloud); // cloud to keyframe
//
//        pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
//        registration->align(*aligned, T_curr2world);
//
//        if(!registration->hasConverged()) {
//            ROS_INFO_STREAM("scan matching has not converged!!");
//            ROS_INFO_STREAM("ignore this frame(" << stamp << ")");
//            return T_curr * T_last;
//        }
//
//        Eigen::Matrix4f trans = registration->getFinalTransformation();
//        Eigen::Matrix4f odom = keyframe_pose * trans;
//
//        if(transform_thresholding) {
//            Eigen::Matrix4f delta = prev_trans.inverse() * trans;
//            double dx = delta.block<3, 1>(0, 3).norm();
//            double da = std::acos(Eigen::Quaternionf(delta.block<3, 3>(0, 0)).w());
//
//            if(dx > max_acceptable_trans || da > max_acceptable_angle) {
//                NODELET_INFO_STREAM("too large transform!!  " << dx << "[m] " << da << "[rad]");
//                NODELET_INFO_STREAM("ignore this frame(" << stamp << ")");
//                return keyframe_pose * prev_trans;
//            }
//        }
//
//        prev_trans = trans;
//
//        auto keyframe_trans = matrix2transform(stamp, keyframe_pose, odom_frame_id, "keyframe");
//        keyframe_broadcaster.sendTransform(keyframe_trans);
//
//        double delta_trans = trans.block<3, 1>(0, 3).norm();
//        double delta_angle = std::acos(Eigen::Quaternionf(trans.block<3, 3>(0, 0)).w());
//        double delta_time = (stamp - keyframe_stamp).toSec();
//        if(delta_trans > keyframe_delta_trans || delta_angle > keyframe_delta_angle || delta_time > keyframe_delta_time) {
//            keyframe = filtered;
//            registration->setInputTarget(keyframe);
//
//            keyframe_pose = odom;
//            keyframe_stamp = stamp;
//            prev_trans.setIdentity();
//        }
//
//        return odom;
//
//    }

    void swapPointCloud(pcl::PointCloud<PointT>::Ptr& p1, pcl::PointCloud<PointT>::Ptr& p2) {
        pcl::PointCloud<PointT>::Ptr pTemp = p1;
        p1 = p2;
        p2 = pTemp;
    }

    void run() {
        ros::Rate rate(100);
        while (ros::ok()) {

            ros::spinOnce();

            if (!cornerSharpBuf.empty() && !cornerLessSharpBuf.empty() &&
                !surfFlatBuf.empty() && !surfLessFlatBuf.empty() && !fullPointsBuf.empty()) {
                //&& !filteredPointsBuf.empty()

                timeCornerPointsSharp = cornerSharpBuf.front()->header.stamp.toSec();
                timeCornerPointsLessSharp = cornerLessSharpBuf.front()->header.stamp.toSec();
                timeSurfPointsFlat = surfFlatBuf.front()->header.stamp.toSec();
                timeSurfPointsLessFlat = surfLessFlatBuf.front()->header.stamp.toSec();
                timeLaserCloudFullRes = fullPointsBuf.front()->header.stamp.toSec();
//                timeLaserCloudFiltered = filteredPointsBuf.front()->header.stamp.toSec();

                if (timeCornerPointsSharp != timeLaserCloudFullRes ||
                    timeCornerPointsLessSharp != timeLaserCloudFullRes ||
                    timeSurfPointsFlat != timeLaserCloudFullRes ||
                    timeSurfPointsLessFlat != timeLaserCloudFullRes ) {
                    //|| timeLaserCloudFiltered != timeLaserCloudFullRes

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
                }

                Timer t_whole("running");

                if (!is_init) {
                    is_init = true;
                    std::cout << "Initialization finished \n";
                } else {

                    downsample(laserCloudFullRes, filteredPoints);
                    outlier_removal(laserCloudFullRes, filteredPoints);

                    int edgePointSize = cornerPointsSharp->size();
                    int planePointSize = surfPointsFlat->size();
                    int fullPointSize = laserCloudFullRes->size();
                    int filteredPointSize = filteredPoints->size();
                    printf("size of {Edge}=%d, {Plane}=%d, {Full}=%d, {Filter}=%d\n", edgePointSize, planePointSize, fullPointSize, filteredPointSize);

//                    undistortPointCloud(cornerPointsSharp, cornerPointsSharp);
//                    undistortPointCloud(surfPointsFlat, surfPointsFlat);
//                    undistortPointCloud(filteredPoints, filteredPoints);
//                    t_whole.count_from_last("undisort pointcloud");

                    for (int reg_counter = 0; reg_counter < 2; reg_counter++) {

                        edge_correspondence = plane_correspondence;

                        ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                        ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();
                        ceres::Problem::Options problem_options;

                        ceres::Problem problem(problem_options);
                        problem.AddParameterBlock(para_q, 4, q_parameterization);
                        problem.AddParameterBlock(para_t, 3);

                        std::vector<int> pointSearchInd;
                        std::vector<float> pointSearchSqDis;

                        Timer t_gen_data("gen ceres data");
                        for (int i = 0; i < edgePointSize; ++i) {
                            kdtreeCornerLast->nearestKSearch(cornerPointsSharp->points[i], 1, pointSearchInd, pointSearchSqDis);
                            int closestPointInd = -1;
                            if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD) {
                                closestPointInd = pointSearchInd[0];
                            }
                            if (closestPointInd >= 0) {
                                Eigen::Vector3d curr_point(cornerPointsSharp->points[i].x,
                                                           cornerPointsSharp->points[i].y,
                                                           cornerPointsSharp->points[i].z);
                                Eigen::Vector3d cloest_point(laserCloudCornerLast->points[closestPointInd].x,
                                                             laserCloudCornerLast->points[closestPointInd].y,
                                                             laserCloudCornerLast->points[closestPointInd].z);
                                ceres::CostFunction *cost_function = LidarDistanceFactor::Create(curr_point, cloest_point);
                                problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                                edge_correspondence++;
                            }
                        }

                        for (int i = 0; i < planePointSize; ++i) {
                            kdtreeSurfLast->nearestKSearch(surfPointsFlat->points[i], 1, pointSearchInd, pointSearchSqDis);
                            int closestPointInd = -1;
                            if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD) {
                                closestPointInd = pointSearchInd[0];
                            }
                            if (closestPointInd >= 0) {
                                Eigen::Vector3d curr_point(surfPointsFlat->points[i].x,
                                                           surfPointsFlat->points[i].y,
                                                           surfPointsFlat->points[i].z);
                                Eigen::Vector3d cloest_point(laserCloudSurfLast->points[closestPointInd].x,
                                                             laserCloudSurfLast->points[closestPointInd].y,
                                                             laserCloudSurfLast->points[closestPointInd].z);
                                ceres::CostFunction *cost_function = LidarDistanceFactor::Create(curr_point, cloest_point);
                                problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                                plane_correspondence++;
                            }
                        }
//                        t_gen_data.count("gen data finished");

                        if ((edge_correspondence + plane_correspondence) < 20) {
                            printf("less correspondence! *************************************************\n");
                        }

                        Timer t_ceres;
                        ceres::Solver::Options options;
                        options.linear_solver_type = ceres::DENSE_QR;
                        options.max_num_iterations = 4;
                        options.minimizer_progress_to_stdout = false;
                        ceres::Solver::Summary summary;
                        // 基于构建的所有残差项，求解最优的当前帧位姿与上一帧位姿的位姿增量：para_q和para_t
                        ceres::Solve(options, &problem, &summary);
                        t_ceres.count("ceres solver");
                    }

                    t_whole.count("run 2 times");

                    Eigen::Matrix3f R = Eigen::Matrix3d(q_curr2last).cast<float>();
                    Eigen::Vector3f t = Eigen::Vector3d(t_curr2last).cast<float>();
                    Eigen::Matrix4f T_curr2last(Eigen::Matrix4f::Identity());
                    T_curr2last.block(0, 0, 3, 3) = R;
                    T_curr2last.block(0, 3, 3, 1) = t;
                    cout << "T_curr2last from ceres: \n" << T_curr2last << endl;

                    Eigen::Matrix4f registrationTrans;
                    Timer t_matching;
                    if (matching(filteredPoints, filteredPointsLast, registrationTrans)) {
                        cout << "registrationTrans from ndt: \n" << registrationTrans << endl;
                    }
                    t_matching.count("matching reg");

                    t_curr2world = t_curr2world + q_curr2world * t_curr2last;
                    q_curr2world = q_curr2world * q_curr2last;
                }

                Timer t_pub;
                // publish odometry
                nav_msgs::Odometry laserOdometry;
                laserOdometry.header.frame_id = "/camera_init";
                laserOdometry.child_frame_id = "/laser_odom";
                laserOdometry.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserOdometry.pose.pose.orientation.x = q_curr2world.x();
                laserOdometry.pose.pose.orientation.y = q_curr2world.y();
                laserOdometry.pose.pose.orientation.z = q_curr2world.z();
                laserOdometry.pose.pose.orientation.w = q_curr2world.w();
                laserOdometry.pose.pose.position.x = t_curr2world.x();
                laserOdometry.pose.pose.position.y = t_curr2world.y();
                laserOdometry.pose.pose.position.z = t_curr2world.z();
                pubLaserOdometry.publish(laserOdometry);

                geometry_msgs::PoseStamped laserPose;
                laserPose.header = laserOdometry.header;
                laserPose.pose = laserOdometry.pose.pose;
                laserPath.header.stamp = laserOdometry.header.stamp;
                laserPath.poses.push_back(laserPose);
                laserPath.header.frame_id = "/camera_init";
                pubLaserPath.publish(laserPath);

                swapPointCloud(cornerPointsSharp, laserCloudCornerLast);
                swapPointCloud(surfPointsFlat, laserCloudSurfLast);
                swapPointCloud(filteredPoints, filteredPointsLast);
//                std::cout << "the size of corner last is " << laserCloudCornerLast->points.size() << ", and the size of surf last is " << laserCloudSurfLast->points.size() << '\n';

                kdtreeCornerLast->setInputCloud(laserCloudCornerLast);// 更新kdtree的点云
                kdtreeSurfLast->setInputCloud(laserCloudSurfLast);

                if (frameCount % skipFrameNum == 0) {
                    frameCount = 0;

                    sensor_msgs::PointCloud2 laserCloudCornerLast2;
                    pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
                    laserCloudCornerLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                    laserCloudCornerLast2.header.frame_id = "/velodyne";
                    pubLaserCloudCornerLast.publish(laserCloudCornerLast2);

                    sensor_msgs::PointCloud2 laserCloudSurfLast2;
                    pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
                    laserCloudSurfLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                    laserCloudSurfLast2.header.frame_id = "/velodyne";
                    pubLaserCloudSurfLast.publish(laserCloudSurfLast2);

                    sensor_msgs::PointCloud2 laserCloudFullRes3;
                    pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
                    laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                    laserCloudFullRes3.header.frame_id = "/velodyne";
                    pubLaserCloudFullRes.publish(laserCloudFullRes3);
                }
                t_pub.count("publication time");
                t_whole.count("whole laserOdometry time");
                frameCount++;
            }
            rate.sleep();
        }

    }


    Odometry() : nh("~"), q_curr2last(para_q), t_curr2last(para_t) {

        // init all T to identity
        q_curr2world = Eigen::Quaterniond(1, 0, 0, 0);
        t_curr2world = Eigen::Vector3d(0, 0, 0);

        need_undisortion = true;

        cornerPointsSharp.reset(new pcl::PointCloud<PointT>());
        cornerPointsLessSharp.reset(new pcl::PointCloud<PointT>());
        surfPointsFlat.reset(new pcl::PointCloud<PointT>());
        surfPointsLessFlat.reset(new pcl::PointCloud<PointT>());
        laserCloudFullRes.reset(new pcl::PointCloud<PointT>());
        filteredPoints.reset(new pcl::PointCloud<PointT>());

        laserCloudCornerLast.reset(new pcl::PointCloud<PointT>());
        laserCloudSurfLast.reset(new pcl::PointCloud<PointT>());
        filteredPointsLast.reset(new pcl::PointCloud<PointT>());

        kdtreeCornerLast.reset(new pcl::KdTreeFLANN<PointT>());
        kdtreeSurfLast.reset(new pcl::KdTreeFLANN<PointT>());

        subCornerPointsSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100, &Odometry::laserCloudSharpHandler, this);

        subCornerPointsLessSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100, &Odometry::laserCloudLessSharpHandler, this);

        subSurfPointsFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100, &Odometry::laserCloudFlatHandler, this);

        subSurfPointsLessFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100, &Odometry::laserCloudLessFlatHandler, this);

        subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100, &Odometry::laserCloudFullResHandler, this);

        pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 100);

        pubLaserPath = nh.advertise<nav_msgs::Path>("/laser_odom_path", 100);

        pubLaserCloudCornerLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 100);

        pubLaserCloudSurfLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 100);

        pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_3", 100);

        registration = Registration::getNDTRegOMP("DIRECT7");

        downsample_filter = Filter::getDownsampleFilter("VOXELGRID");

    }

private:

    ros::NodeHandle nh;

    Eigen::Quaterniond q_curr2world;
    Eigen::Vector3d t_curr2world;

//    // 点云特征匹配时的优化变量
    double para_q[4] = {0, 0, 0, 1};
    double para_t[3] = {0, 0, 0};
//    // 下面的2个分别是优化变量para_q和para_t的映射：表示的是两个world坐标系下的位姿P之间的增量，例如△P = P0.inverse() * P1
    Eigen::Map<Eigen::Quaterniond> q_curr2last;
    Eigen::Map<Eigen::Vector3d> t_curr2last;

    std::queue<sensor_msgs::PointCloud2ConstPtr> cornerSharpBuf;
    std::queue<sensor_msgs::PointCloud2ConstPtr> cornerLessSharpBuf;
    std::queue<sensor_msgs::PointCloud2ConstPtr> surfFlatBuf;
    std::queue<sensor_msgs::PointCloud2ConstPtr> surfLessFlatBuf;
    std::queue<sensor_msgs::PointCloud2ConstPtr> fullPointsBuf;
    std::mutex mBuf;

    const int N_SCANS = 16;
    const int SCAN_FREQUENCY = 10;
    const int DISTANCE_SQ_THRESHOLD = 10;

    bool is_init = false;
    int skipFrameNum = 1;
    int frameCount = 0;
    bool need_undisortion = false;
    int edge_correspondence = 0;
    int plane_correspondence = 0;

    // subscriber topic from preprocessing
    ros::Subscriber subCornerPointsSharp;
    ros::Subscriber subCornerPointsLessSharp;
    ros::Subscriber subSurfPointsFlat;
    ros::Subscriber subSurfPointsLessFlat;
    ros::Subscriber subLaserCloudFullRes;
    ros::Subscriber subLaserCloudFiltered;

    ros::Publisher pubLaserOdometry;
    ros::Publisher pubLaserPath;
    ros::Publisher pubLaserCloudCornerLast;
    ros::Publisher pubLaserCloudSurfLast;
    ros::Publisher pubLaserCloudFullRes;

    nav_msgs::Path laserPath;

    double timeCornerPointsSharp = 0;
    double timeCornerPointsLessSharp = 0;
    double timeSurfPointsFlat = 0;
    double timeSurfPointsLessFlat = 0;
    double timeLaserCloudFullRes = 0;

    // point_cloud current
    pcl::PointCloud<PointT>::Ptr cornerPointsSharp;
    pcl::PointCloud<PointT>::Ptr cornerPointsLessSharp;
    pcl::PointCloud<PointT>::Ptr surfPointsFlat;
    pcl::PointCloud<PointT>::Ptr surfPointsLessFlat;
    pcl::PointCloud<PointT>::Ptr laserCloudFullRes;
    pcl::PointCloud<PointT>::Ptr filteredPoints;

    // point_cloud last
    pcl::PointCloud<PointT>::Ptr laserCloudCornerLast;
    pcl::PointCloud<PointT>::Ptr laserCloudSurfLast;
    pcl::PointCloud<PointT>::Ptr filteredPointsLast;

    // kdTree last
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeCornerLast;
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfLast;

    // pcl_utils
    pcl::Registration<PointT, PointT>::Ptr registration;
    pcl::Filter<PointT>::Ptr downsample_filter;
    pcl::Filter<PointT>::Ptr outlier_removal_filter;

};

int main(int argc, char **argv) {

    ros::init(argc, argv, "laserOdometry");

    Odometry odometry;
    odometry.run();

}
