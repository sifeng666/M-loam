//
// Created by ziv on 2020/10/26.
//

#include "helper.h"
#include "utils.h"
#include "lidar.h"
#include "calibration/calibration.h"

static const int odom_sleep = 2;    // 2ms

class LidarInfo {
public:
    using Ptr = boost::shared_ptr<LidarInfo>;
    using PointCloudPtr = pcl::PointCloud<PointT>::Ptr;
public:
    int i;                                  // Lidar i_th
    LidarMsgReader reader;                  // read ros lidar pointcloud msg
    gtsam::Pose3 odom;                      // latest odom of this lidar, not save history
    ros::Time ros_time;                     // latest ros timestamp of this lidar, not save history
    KeyframeVec::Ptr keyframeVec;           // including all history keyframes, optimized
    LidarStatus::Ptr status;                // lidar status, communication of lidar.cpp and this cpp
    MapGenerator mapGenerator;              // generate global map of this lidar
    FixedKeyframeChannel::Ptr fixedChannel; // channel that save map to be publish
    tf::TransformBroadcaster brMapToFrame;  // tf broadcaster, map_i => frame_i
    tf::TransformBroadcaster brMapToMap;    // tf broadcaster, map   => map_i

    gtsam::Pose3 T_0_i;                     // extrinsic param from L_i to L_base(L_0)
    bool is_base = false;                   // if i == 0, then is_base = true

    // ros msg
    ros::Subscriber sub_laser_cloud, sub_laser_cloud_edge, sub_laser_cloud_surf, sub_laser_cloud_less_edge, sub_laser_cloud_less_surf;
    ros::Publisher pub_map, pub_pointcloud_raw;
    ros::Publisher pub_odom_raw, pub_odom_opti;
    geometry_msgs::PoseArray parray;
    ros::Publisher pub_ros_odom;

public:
    LidarInfo(int i_) : i(i_) {
        T_0_i = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 50 * i, 0));
        if (i == 0) is_base = true;
    }
};

class LaserOdometry {
public:

    void pubRawOdom(LidarInfo::Ptr lidarInfo) {
        std::string child_frame_id = "frame" + std::to_string(lidarInfo->i);
        std::string frame_id = "map" + std::to_string(lidarInfo->i);
        gtsam::Pose3 odom = lidarInfo->odom;
        ros::Time cloud_in_time = lidarInfo->ros_time;
        auto odometry_odom = poseToNavOdometry(cloud_in_time, odom, frame_id, child_frame_id);

        tf::Transform transform;
        transform.setOrigin( tf::Vector3(odom.translation().x(), odom.translation().y(), odom.translation().z()) );
        Eigen::Quaterniond q_current(odom.rotation().matrix());
        tf::Quaternion q(q_current.x(), q_current.y(), q_current.z(), q_current.w());
        transform.setRotation(q);
        lidarInfo->brMapToFrame.sendTransform(tf::StampedTransform(transform, cloud_in_time, frame_id, child_frame_id));

//        geometry_msgs::PoseStamped laserPose;
//        laserPose.header = odometry_odom.header;
//        laserPose.pose = odometry_odom.pose.pose;
//        lidarInfo->path_odom.header.stamp = odometry_odom.header.stamp;
//        lidarInfo->path_odom.poses.push_back(laserPose);
//        lidarInfo->path_odom.header.frame_id = frame_id;
//        lidarInfo->pub_path_odom.publish(lidarInfo->path_odom);

        if (lidarInfo->i == 0) {
            lidarInfo->pub_ros_odom.publish(odometry_odom);
        }
    }


    void pubOptiOdom(LidarInfo::Ptr lidarInfo) {
        if (lidarInfo->keyframeVec->keyframes.empty()) return;
        geometry_msgs::PoseArray& parray = lidarInfo->parray;
        KeyframeVec::Ptr keyframeVec = lidarInfo->keyframeVec;
        // read lock
        std::vector<gtsam::Pose3> poseVec = keyframeVec->read_poses(0, keyframeVec->keyframes.size());

        parray.header.stamp = keyframeVec->keyframes[poseVec.size() - 1]->cloud_in_time;
        parray.header.frame_id = "map" + std::to_string(lidarInfo->i);

        for (size_t i = 0; i < poseVec.size(); i++) {
            Eigen::Quaterniond q(poseVec[i].rotation().matrix());
            if (i < parray.poses.size()) {
                parray.poses[i].orientation.w = q.w();
                parray.poses[i].orientation.x = q.x();
                parray.poses[i].orientation.y = q.y();
                parray.poses[i].orientation.z = q.z();
                parray.poses[i].position.x    = poseVec[i].translation().x();
                parray.poses[i].position.y    = poseVec[i].translation().y();
                parray.poses[i].position.z    = poseVec[i].translation().z();
            } else {
                geometry_msgs::Pose apose;
                apose.orientation.w = q.w();
                apose.orientation.x = q.x();
                apose.orientation.y = q.y();
                apose.orientation.z = q.z();
                apose.position.x = poseVec[i].translation().x();
                apose.position.y = poseVec[i].translation().y();
                apose.position.z = poseVec[i].translation().z();
                parray.poses.push_back(apose);
            }
        }
        lidarInfo->pub_odom_opti.publish(parray);
    }


    void pubRawPointCloud(LidarInfo::Ptr lidarInfo, pcl::PointCloud<PointT>::Ptr cloud_raw) {
        std::string child_frame_id = "frame" + std::to_string(lidarInfo->i);
        ros::Time cloud_in_time = lidarInfo->ros_time;
        sensor_msgs::PointCloud2Ptr raw_msg(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(*cloud_raw, *raw_msg);
        raw_msg->header.stamp = cloud_in_time;
        raw_msg->header.frame_id = child_frame_id;
        lidarInfo->pub_pointcloud_raw.publish(raw_msg);
    }

    void refresh_global_map(LidarInfo::Ptr lidarInfo, pcl::PointCloud<PointT>::Ptr& map) {
        KeyframeVec::Ptr keyframeVec = lidarInfo->keyframeVec;
        if (!keyframeVec) {
            map = nullptr;
            return;
        }
        int size = int(keyframeVec->keyframes.size());
        lidarInfo->mapGenerator.clear();
        lidarInfo->mapGenerator.insert(keyframeVec, 0, size);
        map = lidarInfo->mapGenerator.get(save_map_resolution);
        return;
    }

    void full_map_handler(LidarInfo::Ptr lidarInfo) {
        std::string map_id = "map" + std::to_string(lidarInfo->i);
        pcl::PointCloud<PointT>::Ptr map_full_pub(new pcl::PointCloud<PointT>());

        if (lidarInfo->status->map_refresh_signal) {
            refresh_global_map(lidarInfo, map_full_pub);
            if (map_full_pub) {
                lidarInfo->status->map_refresh_signal = false;
            }
            else return;
        } else {
            Keyframe::Ptr fixed_keyframe = lidarInfo->fixedChannel->get_front();
            if (!fixed_keyframe) return;
            pcl::transformPointCloud(*fixed_keyframe->raw, *map_full_pub, fixed_keyframe->pose_world_curr.matrix());
        }

        sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(*map_full_pub, *cloud_msg);
        cloud_msg->header.stamp = ros::Time::now();
        cloud_msg->header.frame_id = map_id;
        lidarInfo->pub_map.publish(cloud_msg);

        tf::Transform transform;
        transform.setOrigin( tf::Vector3(lidarInfo->T_0_i.x(), lidarInfo->T_0_i.y(), lidarInfo->T_0_i.z()) );
        Eigen::Quaterniond q(lidarInfo->T_0_i.rotation().matrix());
        transform.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
        lidarInfo->brMapToMap.sendTransform(tf::StampedTransform(transform, cloud_msg->header.stamp, "map", map_id));
    }

    void save_global_map(LidarInfo::Ptr lidarInfo, const std::string& filename) {
        if (lidarInfo->status->is_end) {
            pcl::PointCloud<PointT>::Ptr map_full_pub(new pcl::PointCloud<PointT>());
            refresh_global_map(lidarInfo, map_full_pub);
            pcl::io::savePCDFileASCII(file_save_path + filename, *map_full_pub);
            std::cout << "saved map: " << std::to_string(lidarInfo->i) + "!" << std::endl;
            lidarInfo->status->is_end = false;
        }
    }


    void pub_global_map() {

        while (ros::ok()) {


            full_map_handler(lidarInfo0);

            full_map_handler(lidarInfo1);

            save_global_map(lidarInfo0, "gmap0.pcd");

            save_global_map(lidarInfo1, "gmap1.pcd");

            //sleep 200 ms every time
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }

    }

    void laser_odometry_base() {

        while (ros::ok()) {

            if ( !lidarInfo0->reader.pointCloudFullBuf.empty() &&
//                 !lidarInfo0->reader.pointCloudEdgeBuf.empty() &&
//                 !lidarInfo0->reader.pointCloudSurfBuf.empty() &&
                 !lidarInfo0->reader.pointCloudLessEdgeBuf.empty() &&
                 !lidarInfo0->reader.pointCloudLessSurfBuf.empty() ) {

                Timer t_laser_odometry("laser_odometry: " + lidarSensor0.get_lidar_name());

                lidarInfo0->reader.lock();

                if ( lidarInfo0->reader.pointCloudFullBuf.front()->header.stamp.toSec() != lidarInfo0->reader.pointCloudLessEdgeBuf.front()->header.stamp.toSec() ) {
                    printf("unsync messeage!");
                    ROS_BREAK();
                }

                pcl::PointCloud<PointT>::Ptr cloud_raw(new pcl::PointCloud<PointT>());
//                pcl::PointCloud<PointT>::Ptr cloud_in_edge(new pcl::PointCloud<PointT>());
//                pcl::PointCloud<PointT>::Ptr cloud_in_surf(new pcl::PointCloud<PointT>());
                pcl::PointCloud<PointT>::Ptr cloud_in_less_edge(new pcl::PointCloud<PointT>());
                pcl::PointCloud<PointT>::Ptr cloud_in_less_surf(new pcl::PointCloud<PointT>());

                pcl::fromROSMsg(*lidarInfo0->reader.pointCloudFullBuf.front(), *cloud_raw);
//                pcl::fromROSMsg(*lidarInfo0->reader.pointCloudEdgeBuf.front(), *cloud_in_edge);
//                pcl::fromROSMsg(*lidarInfo0->reader.pointCloudSurfBuf.front(), *cloud_in_surf);
                pcl::fromROSMsg(*lidarInfo0->reader.pointCloudLessEdgeBuf.front(), *cloud_in_less_edge);
                pcl::fromROSMsg(*lidarInfo0->reader.pointCloudLessSurfBuf.front(), *cloud_in_less_surf);

                lidarInfo0->ros_time = lidarInfo0->reader.pointCloudFullBuf.front()->header.stamp;

                lidarInfo0->reader.pointCloudFullBuf.pop();
//                lidarInfo0->reader.pointCloudEdgeBuf.pop();
//                lidarInfo0->reader.pointCloudSurfBuf.pop();
                lidarInfo0->reader.pointCloudLessEdgeBuf.pop();
                lidarInfo0->reader.pointCloudLessSurfBuf.pop();

                lidarInfo0->reader.unlock();

                lidarInfo0->odom = lidarSensor0.update(lidarInfo0->ros_time, nullptr, nullptr, cloud_in_less_edge, cloud_in_less_surf, cloud_raw);
//                pubRawOdom(lidarInfo0);
                pubOptiOdom(lidarInfo0);
                pubRawPointCloud(lidarInfo0, cloud_raw);

                t_laser_odometry.count();

            }
            //sleep 2 ms every time
            std::this_thread::sleep_for(std::chrono::milliseconds(odom_sleep));
        }
    }

    void laser_odometry_auxiliary() {

        while (ros::ok()) {

            if ( !lidarInfo1->reader.pointCloudFullBuf.empty() &&
//                 !lidarInfo1->reader.pointCloudEdgeBuf.empty() &&
//                 !lidarInfo1->reader.pointCloudSurfBuf.empty() &&
                 !lidarInfo1->reader.pointCloudLessEdgeBuf.empty() &&
                 !lidarInfo1->reader.pointCloudLessSurfBuf.empty() ) {

                Timer t_laser_odometry("laser_odometry: " + lidarSensor1.get_lidar_name());

                lidarInfo1->reader.lock();

                if ( lidarInfo1->reader.pointCloudFullBuf.front()->header.stamp.toSec() != lidarInfo1->reader.pointCloudLessEdgeBuf.front()->header.stamp.toSec() ) {
                    printf("unsync messeage!");
                    ROS_BREAK();
                }

                pcl::PointCloud<PointT>::Ptr cloud_raw(new pcl::PointCloud<PointT>());
//                pcl::PointCloud<PointT>::Ptr cloud_in_edge(new pcl::PointCloud<PointT>());
//                pcl::PointCloud<PointT>::Ptr cloud_in_surf(new pcl::PointCloud<PointT>());
                pcl::PointCloud<PointT>::Ptr cloud_in_less_edge(new pcl::PointCloud<PointT>());
                pcl::PointCloud<PointT>::Ptr cloud_in_less_surf(new pcl::PointCloud<PointT>());

                pcl::fromROSMsg(*lidarInfo1->reader.pointCloudFullBuf.front(), *cloud_raw);
//                pcl::fromROSMsg(*lidarInfo1->reader.pointCloudEdgeBuf.front(), *cloud_in_edge);
//                pcl::fromROSMsg(*lidarInfo1->reader.pointCloudSurfBuf.front(), *cloud_in_surf);
                pcl::fromROSMsg(*lidarInfo1->reader.pointCloudLessEdgeBuf.front(), *cloud_in_less_edge);
                pcl::fromROSMsg(*lidarInfo1->reader.pointCloudLessSurfBuf.front(), *cloud_in_less_surf);

                lidarInfo1->ros_time = lidarInfo1->reader.pointCloudFullBuf.front()->header.stamp;

                lidarInfo1->reader.pointCloudFullBuf.pop();
//                lidarInfo1->reader.pointCloudEdgeBuf.pop();
//                lidarInfo1->reader.pointCloudSurfBuf.pop();
                lidarInfo1->reader.pointCloudLessEdgeBuf.pop();
                lidarInfo1->reader.pointCloudLessSurfBuf.pop();

                lidarInfo1->reader.unlock();

                lidarInfo1->odom = lidarSensor1.update(lidarInfo1->ros_time, nullptr, nullptr, cloud_in_less_edge, cloud_in_less_surf, cloud_raw);
//                pubRawOdom(lidarInfo1);
                pubOptiOdom(lidarInfo1);
                pubRawPointCloud(lidarInfo1, cloud_raw);

                t_laser_odometry.count();

            }
            //sleep 2 ms every time
            std::this_thread::sleep_for(std::chrono::milliseconds(odom_sleep));
        }
    }

    void laser_calibration() {

        double t_0_1_z_prior = nh.param<double>("t_0_1_z_prior", 0.0);
        gtsam::Pose3 last_T;

        while(ros::ok()) {

            //sleep 2000 ms every time
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));

            if (!convergence) {

                bool success;
                gtsam::Pose3 T_0_1;
                success = HandEyeCalibrator::calibrate(lidarInfo0->keyframeVec, lidarInfo1->keyframeVec, T_0_1);
                if (!success) continue;
                std::cout << "hand eye calibrate result: \n" << T_0_1.matrix() << std::endl;
                T_0_1 = gtsam::Pose3(T_0_1.rotation(), gtsam::Point3(T_0_1.x(), T_0_1.y(), t_0_1_z_prior));

//                auto map_0 = lidarInfo0->mapGenerator.get(0);
//                auto map_1 = lidarInfo1->mapGenerator.get(0);
//                success = LoopDetector::gicp_matching(map_0, map_1, T_0_1, T_0_1);
//                if (!success) continue;
//                std::cout << "gicp calibrate result: \n" << T_0_1.matrix() << std::endl;
                if (T_0_1.equals(last_T, 0.005)) {
                    convergence = true;
                    std::cout << "calibration convergence!" << std::endl;
                    lidarInfo1->T_0_i = T_0_1.inverse();
                }
                last_T = T_0_1;

            }

        }

    }


    LaserOdometry() {

        save_map_resolution = nh.param<double>("save_map_resolution", 0.1);
        file_save_path = nh.param<std::string>("file_save_path", "");

        // lidar 0 init
        lidarSensor0.set_name("Lidar0");
        lidarInfo0 = boost::make_shared<LidarInfo>(0);
        lidarInfo0->keyframeVec = lidarSensor0.get_keyframeVec();
        lidarInfo0->status = lidarSensor0.status;
        lidarInfo0->fixedChannel = lidarSensor0.fixedKeyframeChannel;

        // lidar 1 init
        lidarSensor1.set_name("Lidar1");
        lidarInfo1 = boost::make_shared<LidarInfo>(1);
        lidarInfo1->keyframeVec = lidarSensor1.get_keyframeVec();
        lidarInfo1->status = lidarSensor1.status;
        lidarInfo1->fixedChannel = lidarSensor1.fixedKeyframeChannel;

        // lidar 0 ros subscriber
        lidarInfo0->sub_laser_cloud           = nh.subscribe<sensor_msgs::PointCloud2>("/left/velodyne_points_2",  100, &LidarMsgReader::pointCloudFullHandler, &lidarInfo0->reader);
//        lidarInfo0->sub_laser_cloud_edge      = nh.subscribe<sensor_msgs::PointCloud2>("/left/laser_cloud_edge",  100, &LidarMsgReader::pointCloudEdgeHandler, &lidarInfo0->reader);
//        lidarInfo0->sub_laser_cloud_surf      = nh.subscribe<sensor_msgs::PointCloud2>("/left/laser_cloud_surf",  100, &LidarMsgReader::pointCloudSurfHandler, &lidarInfo0->reader);
        lidarInfo0->sub_laser_cloud_less_edge = nh.subscribe<sensor_msgs::PointCloud2>("/left/laser_cloud_less_edge",  100, &LidarMsgReader::pointCloudLessEdgeHandler, &lidarInfo0->reader);
        lidarInfo0->sub_laser_cloud_less_surf = nh.subscribe<sensor_msgs::PointCloud2>("/left/laser_cloud_less_surf",  100, &LidarMsgReader::pointCloudLessSurfHandler, &lidarInfo0->reader);
        // lidar 0 ros publisher
//        lidarInfo0->pub_odom_raw  = nh.advertise<nav_msgs::Path>("/left/odom_raw", 100);
        lidarInfo0->pub_odom_opti = nh.advertise<geometry_msgs::PoseArray>("/left/odom_opti", 100);
        lidarInfo0->pub_map       = nh.advertise<sensor_msgs::PointCloud2>("/left/global_map", 5);
        lidarInfo0->pub_pointcloud_raw  = nh.advertise<sensor_msgs::PointCloud2>("/left/raw_points", 10);

        // lidar 1 ros subscriber
        lidarInfo1->sub_laser_cloud           = nh.subscribe<sensor_msgs::PointCloud2>("/right/velodyne_points_2",  100, &LidarMsgReader::pointCloudFullHandler, &lidarInfo1->reader);
//        lidarInfo1->sub_laser_cloud_edge      = nh.subscribe<sensor_msgs::PointCloud2>("/right/laser_cloud_edge", 100, &LidarMsgReader::pointCloudEdgeHandler, &lidarInfo1->reader);
//        lidarInfo1->sub_laser_cloud_surf      = nh.subscribe<sensor_msgs::PointCloud2>("/right/laser_cloud_surf", 100, &LidarMsgReader::pointCloudSurfHandler, &lidarInfo1->reader);
        lidarInfo1->sub_laser_cloud_less_edge = nh.subscribe<sensor_msgs::PointCloud2>("/right/laser_cloud_less_edge", 100, &LidarMsgReader::pointCloudLessEdgeHandler, &lidarInfo1->reader);
        lidarInfo1->sub_laser_cloud_less_surf = nh.subscribe<sensor_msgs::PointCloud2>("/right/laser_cloud_less_surf", 100, &LidarMsgReader::pointCloudLessSurfHandler, &lidarInfo1->reader);
        // lidar 1 ros publisher
//        lidarInfo1->pub_odom_raw  = nh.advertise<nav_msgs::Path>("/right/odom_raw", 100);
        lidarInfo1->pub_odom_opti = nh.advertise<geometry_msgs::PoseArray>("/right/odom_opti", 100);
        lidarInfo1->pub_map       = nh.advertise<sensor_msgs::PointCloud2>("/right/global_map", 5);
        lidarInfo1->pub_pointcloud_raw  = nh.advertise<sensor_msgs::PointCloud2>("/right/raw_points", 10);;

        lidarInfo0->pub_ros_odom  = nh.advertise<nav_msgs::Odometry> ("/aft_mapped_to_init", 1);

    }

public:
    LidarSensor lidarSensor0;
    LidarSensor lidarSensor1;
private:

    /*********************************************************************
   ** Global Variables
   *********************************************************************/
    ros::NodeHandle nh;

    LidarInfo::Ptr lidarInfo0;
    LidarInfo::Ptr lidarInfo1;
    bool convergence = false;

    double save_map_resolution;
    std::string file_save_path;

    /*********************************************************************
   ** GTSAM Optimizer
   *********************************************************************/
    gtsam::NonlinearFactorGraph globalGraph;
    gtsam::Values globalEstimate;
    std::shared_ptr<gtsam::ISAM2> isam;
    gtsam::Values isamOptimize;

};

int main(int argc, char **argv) {

    ros::init(argc, argv, "laserOdometry");

    LaserOdometry laserOdometry;

    std::thread laser_odometry_thread_0{&LaserOdometry::laser_odometry_base, &laserOdometry};
    std::thread laser_odometry_thread_1{&LaserOdometry::laser_odometry_auxiliary, &laserOdometry};
    std::thread backend_ba_optimization_0{&LidarSensor::BA_optimization, &laserOdometry.lidarSensor0};
    std::thread backend_ba_optimization_1{&LidarSensor::BA_optimization, &laserOdometry.lidarSensor1};
    std::thread laser_calibration_thread{&LaserOdometry::laser_calibration, &laserOdometry};

    std::thread global_map_publisher{&LaserOdometry::pub_global_map, &laserOdometry};

    ros::spin();

}