//
// Created by ziv on 2020/10/26.
//

#include "helper.h"
#include "utils.h"
#include "lidar.h"
#include "tools/calibration.hpp"

using namespace tools;

static const int odom_sleep = 1;    // 2ms

class LatencyBuf {
private:
    mutable std::mutex mtx;
    std::queue<int> buf;
public:
    inline void push(int a) {
        std::lock_guard lg(mtx);
        buf.push(a);
    }
    inline bool empty() const {
        std::lock_guard lg(mtx);
        return buf.empty();
    }
    inline int get() {
        std::lock_guard lg(mtx);
        if (buf.empty())
            return -1;
        int ret = buf.front();
        buf.pop();
        return ret;
    }
};

class LidarInfo {
public:
    using Ptr = std::shared_ptr<LidarInfo>;
public:
    LidarMsgReader reader;                  // read ros lidar pointcloud msg
    gtsam::Pose3 odom;                      // odom, localization result of the system
    gtsam::Pose3 odom_raw;                  // raw odom of scan to scan method
    ros::Time ros_time;                     // latest ros timestamp of this lidar, not save history
    KeyframeVec::Ptr keyframeVec;           // including all history keyframes, optimized
    LidarStatus::Ptr status;                // lidar status, communication of lidar.cpp and this cpp
    FrameChannel::Ptr frameChannel;
    tf::TransformBroadcaster brMapToFrame;  // tf broadcaster, map_i => frame_i

    // ros msg
    ros::Subscriber sub_laser_cloud;
    ros::Subscriber sub_laser_cloud_edge;
    ros::Subscriber sub_laser_cloud_surf;
    ros::Subscriber sub_laser_cloud_less_edge;
    ros::Subscriber sub_laser_cloud_less_surf;

    ros::Subscriber sub_latency;

    ros::Publisher pub_map;

    ros::Publisher pub_pointcloud;
    ros::Publisher pub_odom_opti;
    nav_msgs::Path parray_path_opti;

    ros::Publisher pub_odom_raw;
    nav_msgs::Path parray_path_raw;
    ros::Publisher pub_odom;
    nav_msgs::Path parray_path_odom;

    ros::Subscriber sub_save_map;
};

class LaserOdometry {
public:

    void pubOdomAndRawOdom(LidarInfo::Ptr lidarInfo) {

        // odom
        auto& parray_path_odom = lidarInfo->parray_path_odom;
        parray_path_odom.header.stamp = ros::Time::now();
        parray_path_odom.header.frame_id = "map";

        geometry_msgs::PoseStamped apose;
        Eigen::Quaterniond q(lidarInfo->odom.rotation().matrix());
        apose.pose.orientation.w = q.w(); apose.pose.orientation.x = q.x();
        apose.pose.orientation.y = q.y(); apose.pose.orientation.z = q.z();
        apose.pose.position.x = lidarInfo->odom.translation().x();
        apose.pose.position.y = lidarInfo->odom.translation().y();
        apose.pose.position.z = lidarInfo->odom.translation().z();
        parray_path_odom.poses.push_back(apose);

        // write to file
        f_odom << setprecision(20) << double(lidarInfo->ros_time.toNSec())/1e9 << " "
               << lidarInfo->odom.translation().x() << " " << lidarInfo->odom.translation().y()
               << " " << lidarInfo->odom.translation().z() << " " << q.x()
               << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        //odom raw
        auto& parray_path_raw = lidarInfo->parray_path_raw;
        parray_path_raw.header.stamp = ros::Time::now();
        parray_path_raw.header.frame_id = "map";

        geometry_msgs::PoseStamped apose_raw;
        Eigen::Quaterniond q_raw(lidarInfo->odom_raw.rotation().matrix());
        apose_raw.pose.orientation.w = q.w(); apose_raw.pose.orientation.x = q.x();
        apose_raw.pose.orientation.y = q.y(); apose_raw.pose.orientation.z = q.z();
        apose_raw.pose.position.x = lidarInfo->odom_raw.translation().x();
        apose_raw.pose.position.y = lidarInfo->odom_raw.translation().y();
        apose_raw.pose.position.z = lidarInfo->odom_raw.translation().z();
        parray_path_raw.poses.push_back(apose_raw);

        // write to file
        f_odom_raw << setprecision(20) << double(lidarInfo->ros_time.toNSec())/1e9 << " "
               << lidarInfo->odom_raw.translation().x() << " " << lidarInfo->odom_raw.translation().y()
               << " " << lidarInfo->odom_raw.translation().z() << " " << q_raw.x()
               << " " << q_raw.y() << " " << q_raw.z() << " " << q_raw.w() << endl;

        lidarInfo->pub_odom.publish(parray_path_odom);
        lidarInfo->pub_odom_raw.publish(parray_path_raw);
    }

    void pubOptiOdom(LidarInfo::Ptr lidarInfo) {
        if (lidarInfo->keyframeVec->keyframes.empty()) return;
        auto& parray_path = lidarInfo->parray_path_opti;
        KeyframeVec::Ptr keyframeVec = lidarInfo->keyframeVec;
        // read lock
        std::vector<gtsam::Pose3> poseVec = keyframeVec->read_poses(0, keyframeVec->keyframes.size(), true);

        parray_path.header.stamp = ros::Time::now();
        parray_path.header.frame_id = "map";
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
    }


    void pubRawPointCloud(LidarInfo::Ptr lidarInfo, gtsam::Pose3 odom, pcl::PointCloud<PointT>::Ptr cloud_raw) {
        std::string child_frame_id = "frame";
        std::string frame_id = "map";
        ros::Time cloud_in_time = ros::Time::now();

        sensor_msgs::PointCloud2Ptr raw_msg(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(*cloud_raw, *raw_msg);
        raw_msg->header.stamp = cloud_in_time;
        raw_msg->header.frame_id = child_frame_id;
        lidarInfo->pub_pointcloud.publish(raw_msg);

        tf::Transform transform;
        transform.setOrigin( tf::Vector3(odom.translation().x(), odom.translation().y(), odom.translation().z()) );
        Eigen::Quaterniond q_current(odom.rotation().matrix());
        tf::Quaternion q(q_current.x(), q_current.y(), q_current.z(), q_current.w());
        transform.setRotation(q);
        lidarInfo->brMapToFrame.sendTransform(tf::StampedTransform(transform, cloud_in_time, frame_id, child_frame_id));

    }

    void full_map_handler(LidarInfo::Ptr lidarInfo) {
        std::string map_id = "map";

        global_map = MapGenerator::generate_cloud(lidarInfo->keyframeVec, 0, lidarInfo->keyframeVec->size(), FeatureType::Full, true);
        if (!global_map) return;
        pcl::VoxelGrid<PointT> v;
        v.setLeafSize(show_map_resolution, show_map_resolution, show_map_resolution);
        v.setInputCloud(global_map);
        v.filter(*global_map);

        sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(*global_map, *cloud_msg);
        cloud_msg->header.stamp = ros::Time::now();
        cloud_msg->header.frame_id = map_id;
        lidarInfo->pub_map.publish(cloud_msg);
    }

    void save_map_handler(const std_msgs::StringConstPtr& str) {
//        TicToc t_save;
//        string data = str->data;
//        cout << "std_msgs string: " << data << endl;
//        if (data.size() - data.find(".pcd") == 4) {
//            data = str->data;
//        } else {
//            data += ".pcd";
//        }
//        cout << "save path: " << file_save_path + data << endl;
//        _mkdir(file_save_path + data);
//        auto map = MapGenerator::generate_cloud(lidarInfo->keyframeVec, 0, lidarInfo->keyframeVec->size(), FeatureType::Full, true);
//        if (map) {
//            pcl::io::savePCDFileASCII(file_save_path + data, *map);
//            printf("global map save! cost: %.3f ms\n", t_save.toc());
//        }

        _mkdir(file_save_path + "opti_pose.txt");
        std::ofstream f_mapping_pose(file_save_path + "opti_pose.txt");
        for (int i = 0; i < lidarInfo->keyframeVec->size(); i++) {
//            f_mapping_pose << 1 << ": " << lidarInfo->keyframeVec->at(i)->cloud_in_time.toNSec() << endl << lidarInfo->keyframeVec->at(i)->pose_fixed.matrix() << endl;
            Eigen::Quaterniond q(lidarInfo->keyframeVec->at(i)->pose_fixed.rotation().matrix());
            auto& p = lidarInfo->keyframeVec->at(i)->pose_fixed;
            f_mapping_pose << setprecision(20) << double(lidarInfo->keyframeVec->at(i)->cloud_in_time.toNSec())/1e9 << " "
                   << p.translation().x() << " " << p.translation().y() << " " << p.translation().z() << " " << q.x()
                   << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }
        f_mapping_pose.close();

    }

    void pub_global_map() {
        while (ros::ok()) {
            full_map_handler(lidarInfo);
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

                lidarInfo->odom = lidarSensor.update_odom(lidarInfo->ros_time, cloud_in_less_edge, cloud_in_less_surf, cloud_raw, lidarInfo->odom_raw);
                f_odom_time << tic.toc() << endl;
                pubOdomAndRawOdom(lidarInfo);
                printf("[Odometry]: %.3f ms\n", tic.toc());

            }
            //sleep 2 ms every time
            std::this_thread::sleep_for(std::chrono::milliseconds(odom_sleep));
        }
    }

    void laser_mapping() {

        auto& frameChannel = lidarInfo->frameChannel;

        while (ros::ok()) {

            auto frame = frameChannel->get_front();
            if (!frame) continue;
//            frameChannel->clear();

            TicToc t_mapping;

            auto odom = lidarSensor.update_mapping(frame);
            f_mapping_time << t_mapping.toc() << endl;
            pubOptiOdom(lidarInfo);
            pubRawPointCloud(lidarInfo, odom, frame->raw);

            double time_cost = t_mapping.toc();
            printf("[Mapping]: %.3f ms\n", time_cost);

            if (set_latency && !latencyBuf.empty()) {
                int latency = latencyBuf.get();
                cout << "############## bad conection, stop mapping ############" << endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(latency));
                cout << "############## good conection, go mapping #############" << endl;
            } else {
                //sleep 1 ms every time
                std::this_thread::sleep_for(std::chrono::milliseconds(odom_sleep));
            }
        }

    }

    void latency_handler(const std_msgs::Int32ConstPtr &latency) {
        latencyBuf.push(int(latency->data));
    }

    LaserOdometry() {

        save_map_resolution = nh.param<double>("save_map_resolution", 0.1);
        show_map_resolution = nh.param<double>("show_map_resolution", 0.2);
        file_save_path = nh.param<std::string>("file_save_path", "");
        set_latency = nh.param<bool>("set_latency", false);

        f_mapping_time.open(file_save_path + "time_ours.txt");
        f_odom_time.open(file_save_path + "time_odom_ours.txt");

        global_map = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);

        // lidar 0 init
        lidarSensor.set_name("Lidar");
        lidarInfo = std::make_shared<LidarInfo>();
        lidarInfo->keyframeVec = lidarSensor.get_keyframeVec();
        lidarInfo->status = lidarSensor.status;
        lidarInfo->frameChannel = lidarSensor.get_frameChannel();

        lidarInfo->sub_laser_cloud           = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points_2",  100, &LidarMsgReader::pointCloudFullHandler, &lidarInfo->reader);
        lidarInfo->sub_laser_cloud_edge      = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_edge",  100, &LidarMsgReader::pointCloudEdgeHandler, &lidarInfo->reader);
        lidarInfo->sub_laser_cloud_surf      = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_edge",  100, &LidarMsgReader::pointCloudSurfHandler, &lidarInfo->reader);
        lidarInfo->sub_laser_cloud_less_edge = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_edge",  100, &LidarMsgReader::pointCloudLessEdgeHandler, &lidarInfo->reader);
        lidarInfo->sub_laser_cloud_less_surf = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_surf",  100, &LidarMsgReader::pointCloudLessSurfHandler, &lidarInfo->reader);
        lidarInfo->pub_odom_opti             = nh.advertise<nav_msgs::Path>(          "/odom_opti", 100);
        lidarInfo->pub_map                   = nh.advertise<sensor_msgs::PointCloud2>("/global_map", 5);
        lidarInfo->pub_pointcloud            = nh.advertise<sensor_msgs::PointCloud2>("/raw_points", 10);
        lidarInfo->pub_odom_raw              = nh.advertise<nav_msgs::Path>(          "/odom_raw", 100);
        lidarInfo->pub_odom                  = nh.advertise<nav_msgs::Path>(          "/odom", 100);

        lidarInfo->sub_save_map = nh.subscribe<std_msgs::String>("/save_map", 1, &LaserOdometry::save_map_handler, this);
        lidarInfo->sub_latency = nh.subscribe<std_msgs::Int32>  ("/latency", 1, &LaserOdometry::latency_handler, this);


        int interval, sustain;
        interval = nh.param<int>("interval", 5);
        sustain = nh.param<int>("sustain", 2);

        string file_name = file_save_path + to_string(interval) + "-" + to_string(sustain) + ".txt";
        _mkdir(file_name);
        f_odom.open(file_name);
        f_odom_raw.open(file_save_path + "raw.txt");

    }

public:
    LidarSensor lidarSensor;
    pcl::PointCloud<PointT>::Ptr global_map;
private:

    /*********************************************************************
   ** Global Variables
   *********************************************************************/
    ros::NodeHandle nh;

    LidarInfo::Ptr lidarInfo;

    double save_map_resolution;
    double show_map_resolution;
    std::string file_save_path;
    bool set_latency;
    LatencyBuf latencyBuf;
    std::ofstream f_odom, f_odom_raw;

    std::ofstream f_mapping_time;
    std::ofstream f_odom_time;

};

int main(int argc, char **argv) {

    ros::init(argc, argv, "laserOdometry");

    LaserOdometry laserOdometry;

    std::thread laser_odometry_thread_0{&LaserOdometry::laser_odometry, &laserOdometry};
    std::thread laser_mapping_thread_0{&LaserOdometry::laser_mapping, &laserOdometry};
    std::thread backend_loop_detector_0{&LidarSensor::loop_detect_thread, &laserOdometry.lidarSensor};
    std::thread global_map_publisher{&LaserOdometry::pub_global_map, &laserOdometry};

    ros::spin();

}