//
// Created by ziv on 2020/10/26.
//

#include "helper.h"
#include "utils.h"
#include "lidar.h"
#include "tools/calibration.hpp"

using namespace tools;

static const int odom_sleep = 2;    // 2ms

class LidarInfo {
public:
    using Ptr = std::shared_ptr<LidarInfo>;
public:
    int i;                                  // Lidar i_th
    LidarMsgReader reader;                  // read ros lidar pointcloud msg
    gtsam::Pose3 odom;                      // latest odom of this lidar, not save history
    ros::Time ros_time;                     // latest ros timestamp of this lidar, not save history
    KeyframeVec::Ptr keyframeVec;           // including all history keyframes, optimized
    LidarStatus::Ptr status;                // lidar status, communication of lidar.cpp and this cpp
//    MapGenerator mapGenerator;              // generate global map of this lidar
    FixedKeyframeChannel::Ptr fixedChannel; // channel that save map to be publish
    tf::TransformBroadcaster brMapToFrame;  // tf broadcaster, map_i => frame_i
    tf::TransformBroadcaster brMapToMap;    // tf broadcaster, map   => map_i

    gtsam::Pose3 T_0_i;                     // extrinsic param from L_i to L_base(L_0)
    bool is_base = false;                   // if i == 0, then is_base = true

    // ros msg
    ros::Subscriber sub_laser_cloud;
    ros::Subscriber sub_laser_cloud_edge;
    ros::Subscriber sub_laser_cloud_surf;
    ros::Subscriber sub_laser_cloud_less_edge;
    ros::Subscriber sub_laser_cloud_less_surf;
    ros::Publisher pub_map;
    ros::Publisher pub_pointcloud_raw;
    ros::Publisher pub_odom_opti;
    nav_msgs::Path parray_path;

public:
    LidarInfo(int i_) : i(i_) {
        T_0_i = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 50 * i, 0));
        if (i == 0) is_base = true;
    }
};

class LaserOdometry {
public:
    void pubOptiOdom(LidarInfo::Ptr lidarInfo) {
        if (lidarInfo->keyframeVec->keyframes.empty()) return;
        auto& parray_path = lidarInfo->parray_path;
        KeyframeVec::Ptr keyframeVec = lidarInfo->keyframeVec;
        // read lock
        std::vector<gtsam::Pose3> poseVec = keyframeVec->read_poses(0, keyframeVec->keyframes.size(), true);

        parray_path.header.stamp = ros::Time::now();
        parray_path.header.frame_id = "map" + std::to_string(lidarInfo->i);

        for (size_t i = 0; i < poseVec.size(); i++) {
            Eigen::Quaterniond q(poseVec[i].rotation().matrix());
            if (i < parray_path.poses.size()) {
                parray_path.poses[i].pose.orientation.w = q.w(); parray_path.poses[i].pose.orientation.x = q.x();
                parray_path.poses[i].pose.orientation.y = q.y(); parray_path.poses[i].pose.orientation.z = q.z();
                parray_path.poses[i].pose.position.x = poseVec[i].translation().x();
                parray_path.poses[i].pose.position.y = poseVec[i].translation().y();
                parray_path.poses[i].pose.position.z = poseVec[i].translation().z();
            } else {
                geometry_msgs::PoseStamped apose;
                apose.pose.orientation.w = q.w(); apose.pose.orientation.x = q.x();
                apose.pose.orientation.y = q.y(); apose.pose.orientation.z = q.z();
                apose.pose.position.x = poseVec[i].translation().x();
                apose.pose.position.y = poseVec[i].translation().y();
                apose.pose.position.z = poseVec[i].translation().z();
                parray_path.poses.push_back(apose);
            }
        }

        lidarInfo->pub_odom_opti.publish(parray_path);

        tf::Transform transform;
        gtsam::Pose3 T(lidarInfo->T_0_i.inverse());
        transform.setOrigin( tf::Vector3(T.x(), T.y(), T.z()) );
        Eigen::Quaterniond q(T.rotation().matrix());
        transform.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
        lidarInfo->brMapToMap.sendTransform(tf::StampedTransform(transform, parray_path.header.stamp, "map", parray_path.header.frame_id));
    }


    void pubRawPointCloud(LidarInfo::Ptr lidarInfo, pcl::PointCloud<PointT>::Ptr cloud_raw) {
        std::string child_frame_id = "frame" + std::to_string(lidarInfo->i);
        std::string frame_id = "map" + std::to_string(lidarInfo->i);
        gtsam::Pose3 odom = lidarInfo->odom;
        ros::Time cloud_in_time = ros::Time::now();

        sensor_msgs::PointCloud2Ptr raw_msg(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(*cloud_raw, *raw_msg);
        raw_msg->header.stamp = cloud_in_time;
        raw_msg->header.frame_id = child_frame_id;
        lidarInfo->pub_pointcloud_raw.publish(raw_msg);

        tf::Transform transform;
        transform.setOrigin( tf::Vector3(odom.translation().x(), odom.translation().y(), odom.translation().z()) );
        Eigen::Quaterniond q_current(odom.rotation().matrix());
        tf::Quaternion q(q_current.x(), q_current.y(), q_current.z(), q_current.w());
        transform.setRotation(q);
        lidarInfo->brMapToFrame.sendTransform(tf::StampedTransform(transform, cloud_in_time, frame_id, child_frame_id));

    }

    void full_map_handler(LidarInfo::Ptr lidarInfo) {
        std::string map_id = "map" + std::to_string(lidarInfo->i);

        auto map_full_pub = MapGenerator::generate_cloud(lidarInfo->keyframeVec, 0, lidarInfo->keyframeVec->size(), FeatureType::Full, true);
        if (!map_full_pub) return;
        pcl::VoxelGrid<PointT> v;
        v.setLeafSize(show_map_resolution, show_map_resolution, show_map_resolution);
        v.setInputCloud(map_full_pub);
        v.filter(*map_full_pub);

        sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(*map_full_pub, *cloud_msg);
        cloud_msg->header.stamp = ros::Time::now();
        cloud_msg->header.frame_id = map_id;
        lidarInfo->pub_map.publish(cloud_msg);
    }

    void save_global_map(LidarInfo::Ptr lidarInfo, const std::string& filename) {
        if (lidarInfo->status->is_end) {
            auto map_full_pub = MapGenerator::generate_cloud(lidarInfo->keyframeVec, 0, lidarInfo->keyframeVec->size(), FeatureType::Full, true);
            if (!map_full_pub) return;
            down_sampling_voxel(*map_full_pub, save_map_resolution);

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

    void laser_odometry() {

        while (ros::ok()) {

            if ( !lidarInfo->reader.pointCloudFullBuf.empty() &&
                 !lidarInfo->reader.pointCloudEdgeBuf.empty() &&
                 !lidarInfo->reader.pointCloudSurfBuf.empty() &&
                 !lidarInfo->reader.pointCloudLessEdgeBuf.empty() &&
                 !lidarInfo->reader.pointCloudLessSurfBuf.empty() ) {

                TicToc tic;

                lidarInfo->reader.lock();

                if ( lidarInfo->reader.pointCloudFullBuf.front()->header.stamp.toSec() != lidarInfo->reader.pointCloudLessEdgeBuf.front()->header.stamp.toSec() ) {
                    printf("unsync messeage!");
                    ROS_BREAK();
                }

                pcl::PointCloud<PointT>::Ptr cloud_raw(new pcl::PointCloud<PointT>());
                pcl::PointCloud<PointT>::Ptr cloud_in_edge(new pcl::PointCloud<PointT>());
                pcl::PointCloud<PointT>::Ptr cloud_in_surf(new pcl::PointCloud<PointT>());
                pcl::PointCloud<PointT>::Ptr cloud_in_less_edge(new pcl::PointCloud<PointT>());
                pcl::PointCloud<PointT>::Ptr cloud_in_less_surf(new pcl::PointCloud<PointT>());

                pcl::fromROSMsg(*lidarInfo->reader.pointCloudFullBuf.front(), *cloud_raw);
                pcl::fromROSMsg(*lidarInfo->reader.pointCloudEdgeBuf.front(), *cloud_in_edge);
                pcl::fromROSMsg(*lidarInfo->reader.pointCloudSurfBuf.front(), *cloud_in_surf);
                pcl::fromROSMsg(*lidarInfo->reader.pointCloudLessEdgeBuf.front(), *cloud_in_less_edge);
                pcl::fromROSMsg(*lidarInfo->reader.pointCloudLessSurfBuf.front(), *cloud_in_less_surf);

                lidarInfo->ros_time = lidarInfo->reader.pointCloudFullBuf.front()->header.stamp;

                lidarInfo->reader.pointCloudFullBuf.pop();
                lidarInfo->reader.pointCloudEdgeBuf.pop();
                lidarInfo->reader.pointCloudSurfBuf.pop();
                lidarInfo->reader.pointCloudLessEdgeBuf.pop();
                lidarInfo->reader.pointCloudLessSurfBuf.pop();

                lidarInfo->reader.unlock();

                lidarInfo->odom = lidarSensor.update(lidarInfo->ros_time, cloud_in_edge, cloud_in_surf, cloud_raw);
                pubOptiOdom(lidarInfo);
                pubRawPointCloud(lidarInfo, cloud_raw);

                printf("laser odometry cost: %.3f ms", tic.toc());

            }
            //sleep 2 ms every time
            std::this_thread::sleep_for(std::chrono::milliseconds(odom_sleep));
        }
    }

    LaserOdometry() {

        save_map_resolution = nh.param<double>("save_map_resolution", 0.1);
        show_map_resolution = nh.param<double>("show_map_resolution", 0.2);
        file_save_path = nh.param<std::string>("file_save_path", "");

        // lidar 0 init
        lidarSensor.set_name("Lidar");
        lidarInfo = std::make_shared<LidarInfo>(0);
        lidarInfo->keyframeVec = lidarSensor.get_keyframeVec();
        lidarInfo->status = lidarSensor.status;

        lidarInfo->sub_laser_cloud           = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points_2",  100, &LidarMsgReader::pointCloudFullHandler, &lidarInfo->reader);
        lidarInfo->sub_laser_cloud_edge      = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_edge",  100, &LidarMsgReader::pointCloudEdgeHandler, &lidarInfo->reader);
        lidarInfo->sub_laser_cloud_surf      = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_edge",  100, &LidarMsgReader::pointCloudSurfHandler, &lidarInfo->reader);
        lidarInfo->sub_laser_cloud_less_edge = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_edge",  100, &LidarMsgReader::pointCloudLessEdgeHandler, &lidarInfo->reader);
        lidarInfo->sub_laser_cloud_less_surf = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_surf",  100, &LidarMsgReader::pointCloudLessSurfHandler, &lidarInfo->reader);
        lidarInfo->pub_odom_opti             = nh.advertise<nav_msgs::Path>(          "/odom_opti", 100);
        lidarInfo->pub_map                   = nh.advertise<sensor_msgs::PointCloud2>("/global_map", 5);
        lidarInfo->pub_pointcloud_raw        = nh.advertise<sensor_msgs::PointCloud2>("/raw_points", 10);

    }

public:
    LidarSensor lidarSensor;
private:

    /*********************************************************************
   ** Global Variables
   *********************************************************************/
    ros::NodeHandle nh;

    LidarInfo::Ptr lidarInfo;

    bool calibra_simple = false;
    bool calibra_with_loop = false;
    bool calibra_fixed = false;

    double save_map_resolution;
    double show_map_resolution;
    std::string file_save_path;

};

int main(int argc, char **argv) {

    ros::init(argc, argv, "laserOdometry");

    LaserOdometry laserOdometry;

    std::thread laser_odometry_thread_0{&LaserOdometry::laser_odometry_base, &laserOdometry};
    std::thread backend_ba_optimization_0{&LidarSensor::BA_optimization, &laserOdometry.lidarSensor};
    std::thread backend_loop_detector_0{&LidarSensor::loop_detect_thread, &laserOdometry.lidarSensor};

    std::thread global_map_publisher{&LaserOdometry::pub_global_map, &laserOdometry};

    ros::spin();

}