//
// Created by ziv on 2020/10/26.
//

#include "helper.h"
#include "lidar.h"
#include "utils.h"

class LaserOdometry {
public:

    void pubOdomAndPath(const ros::Time& cloud_in_time, const gtsam::Pose3& odom, KeyframeVec::Ptr keyframeVec, int n_sensor) {

        std::string child_frame_id = "frame" + std::to_string(n_sensor);
        auto odometry_odom = poseToNavOdometry(cloud_in_time, odom, "map", child_frame_id);
        auto odometry_opti = poseToNavOdometry(cloud_in_time, odom, "map", child_frame_id);

        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(odom.translation().x(), odom.translation().y(), odom.translation().z()) );
        Eigen::Quaterniond q_current(odom.rotation().matrix());
        tf::Quaternion q(q_current.x(),q_current.y(),q_current.z(),q_current.w());
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, cloud_in_time, "map", child_frame_id));

        geometry_msgs::PoseStamped laserPose;
        laserPose.header = odometry_opti.header;
        laserPose.pose = odometry_opti.pose.pose;
        path_opti[n_sensor].header.stamp = odometry_opti.header.stamp;
        path_opti[n_sensor].poses.clear();

        // read lock
        std::vector<gtsam::Pose3> poseVec = keyframeVec->read_poses(0, keyframeVec->keyframes.size());

        for (size_t i = 0; i < poseVec.size(); i++) {
            laserPose.header = odometry_opti.header;
            Eigen::Quaterniond q(poseVec[i].rotation().matrix());
            laserPose.pose.orientation.x    = q.x();
            laserPose.pose.orientation.y    = q.y();
            laserPose.pose.orientation.z    = q.z();
            laserPose.pose.orientation.w    = q.w();
            laserPose.pose.position.x       = poseVec[i].translation().x();
            laserPose.pose.position.y       = poseVec[i].translation().y();
            laserPose.pose.position.z       = poseVec[i].translation().z();
            path_opti[n_sensor].poses.push_back(laserPose);
        }
        path_opti[n_sensor].header.frame_id = "map";
        pub_path_opti[n_sensor].publish(path_opti[n_sensor]);

        laserPose.header = odometry_odom.header;
        laserPose.pose = odometry_odom.pose.pose;
        path_odom[n_sensor].header.stamp = odometry_odom.header.stamp;
        path_odom[n_sensor].poses.push_back(laserPose);
        path_odom[n_sensor].header.frame_id = "map";
        pub_path_odom[n_sensor].publish(path_odom[n_sensor]);

    }

    void pubRawPointCloud(const ros::Time& cloud_in_time, pcl::PointCloud<PointT>::Ptr cloud_in_edge, pcl::PointCloud<PointT>::Ptr cloud_in_surf, int n_sensor) {
        std::string child_frame_id = "frame" + std::to_string(n_sensor);
        sensor_msgs::PointCloud2Ptr edge_msg(new sensor_msgs::PointCloud2());
        sensor_msgs::PointCloud2Ptr surf_msg(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(*cloud_in_edge, *edge_msg);
        pcl::toROSMsg(*cloud_in_surf, *surf_msg);
        edge_msg->header.stamp = cloud_in_time;
        edge_msg->header.frame_id = child_frame_id;
        surf_msg->header.stamp = cloud_in_time;
        surf_msg->header.frame_id = child_frame_id;
        pub_curr_edge[n_sensor].publish(edge_msg);
        pub_curr_surf[n_sensor].publish(surf_msg);
    }


    void pub_global_map() {

        while (ros::ok()) {

            if (!keyframeVec_0)
                continue;

            size_t size = keyframeVec_0->keyframes.size();

            if (size > 0 && keyframeVec_0->keyframes[size - 1]->is_init()) {

//                if (regenerate_map) {
                    mapGenerator.clear();
                    mapGenerator.insert(keyframeVec_0, 0, size);
//                    regenerate_map = false;
//                } else {
//                    mapGenerator.insert(keyframeVec, last_map_generate_index, size);
//                }

                auto globalMap = mapGenerator.get(save_map_resolution);
                if (!globalMap) {
                    std::cout << "empty global map!" << std::endl;
                    continue;
                }
                sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2());
                pcl::toROSMsg(*globalMap, *cloud_msg);
                cloud_msg->header.stamp = time_0;
                cloud_msg->header.frame_id = "map";

                pub_map[0].publish(cloud_msg);

                if (last_map_generate_index == size) {
                    save_latency++;
                } else {
                    save_latency = 0;
                }
                if (save_latency == 20) {
                    pcl::io::savePCDFileASCII("/home/ziv/mloam/global_map_opti.pcd", *globalMap);
                    cout << "saved map!" << endl;
                }

                last_map_generate_index = size;
            }


            //sleep 200 ms every time
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }

    }

    void laser_odometry_base() {

        while (ros::ok()) {

            if ( !lidarMsgReader_0.pointCloudEdgeBuf.empty() && !lidarMsgReader_0.pointCloudSurfBuf.empty() ) {

                Timer t_laser_odometry("laser_odometry: " + lidarSensor0.get_lidar_name());

                lidarMsgReader_0.lock();

                if ( lidarMsgReader_0.pointCloudEdgeBuf.front()->header.stamp.toSec() != lidarMsgReader_0.pointCloudSurfBuf.front()->header.stamp.toSec() ) {
                    printf("unsync messeage!");
                    ROS_BREAK();
                }

                pcl::PointCloud<PointT>::Ptr cloud_in_edge(new pcl::PointCloud<PointT>());
                pcl::PointCloud<PointT>::Ptr cloud_in_surf(new pcl::PointCloud<PointT>());

                pcl::fromROSMsg(*lidarMsgReader_0.pointCloudEdgeBuf.front(), *cloud_in_edge);
                pcl::fromROSMsg(*lidarMsgReader_0.pointCloudSurfBuf.front(), *cloud_in_surf);

                time_0 = lidarMsgReader_0.pointCloudEdgeBuf.front()->header.stamp;

                lidarMsgReader_0.pointCloudEdgeBuf.pop();
                lidarMsgReader_0.pointCloudSurfBuf.pop();

                lidarMsgReader_0.unlock();

                gtsam::Pose3 odom = lidarSensor0.update(cloud_in_edge, cloud_in_surf);
                pubOdomAndPath(time_0, odom, keyframeVec_0, 0);
                pubRawPointCloud(time_0, cloud_in_edge, cloud_in_surf, 0);

                t_laser_odometry.count();

            }

            //sleep 2 ms every time
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    void laser_odometry_auxiliary() {

        while (ros::ok()) {

            if ( !lidarMsgReader_1.pointCloudEdgeBuf.empty() && !lidarMsgReader_1.pointCloudSurfBuf.empty() ) {

                Timer t_laser_odometry("laser_odometry: " + lidarSensor1.get_lidar_name());

                lidarMsgReader_1.lock();

                if ( lidarMsgReader_1.pointCloudEdgeBuf.front()->header.stamp.toSec() != lidarMsgReader_1.pointCloudSurfBuf.front()->header.stamp.toSec() ) {
                    printf("unsync messeage!");
                    ROS_BREAK();
                }

                pcl::PointCloud<PointT>::Ptr cloud_in_edge(new pcl::PointCloud<PointT>());
                pcl::PointCloud<PointT>::Ptr cloud_in_surf(new pcl::PointCloud<PointT>());

                pcl::fromROSMsg(*lidarMsgReader_1.pointCloudEdgeBuf.front(), *cloud_in_edge);
                pcl::fromROSMsg(*lidarMsgReader_1.pointCloudSurfBuf.front(), *cloud_in_surf);

                time_1 = lidarMsgReader_1.pointCloudEdgeBuf.front()->header.stamp;

                lidarMsgReader_1.pointCloudEdgeBuf.pop();
                lidarMsgReader_1.pointCloudSurfBuf.pop();

                lidarMsgReader_1.unlock();

                gtsam::Pose3 odom = lidarSensor1.update(cloud_in_edge, cloud_in_surf);
                pubOdomAndPath(time_1, odom, keyframeVec_1, 1);
                pubRawPointCloud(time_1, cloud_in_edge, cloud_in_surf, 1);

                t_laser_odometry.count();

            }

            //sleep 2 ms every time
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }


    LaserOdometry() {

        save_map_resolution = nh.param<double>("save_map_resolution", 0.1);

        lidarSensor0.setName("Lidar0");
        lidarSensor1.setName("Lidar1");

        int n_sensor = 2;
        keyframeVec_0 = lidarSensor0.get_keyframeVec();
        keyframeVec_1 = lidarSensor1.get_keyframeVec();

        sub_laser_cloud.resize(n_sensor);
        sub_laser_cloud_edge.resize(n_sensor);
        sub_laser_cloud_surf.resize(n_sensor);
        pub_path_odom.resize(n_sensor);
        pub_path_opti.resize(n_sensor);
        pub_map.resize(n_sensor);
        pub_curr_edge.resize(n_sensor);
        pub_curr_surf.resize(n_sensor);
        path_odom.resize(n_sensor);
        path_opti.resize(n_sensor);

        sub_laser_cloud_edge[0] = nh.subscribe<sensor_msgs::PointCloud2>("/left/laser_cloud_edge",  100, &LidarMsgReader::pointCloudEdgeHandler, &lidarMsgReader_0);
        sub_laser_cloud_surf[0] = nh.subscribe<sensor_msgs::PointCloud2>("/left/laser_cloud_surf",  100, &LidarMsgReader::pointCloudSurfHandler, &lidarMsgReader_0);
        sub_laser_cloud_edge[1] = nh.subscribe<sensor_msgs::PointCloud2>("/right/laser_cloud_edge", 100, &LidarMsgReader::pointCloudEdgeHandler, &lidarMsgReader_1);
        sub_laser_cloud_surf[1] = nh.subscribe<sensor_msgs::PointCloud2>("/right/laser_cloud_surf", 100, &LidarMsgReader::pointCloudSurfHandler, &lidarMsgReader_1);

        pub_path_odom[0] = nh.advertise<nav_msgs::Path>("/left/path_odom", 100);
        pub_path_opti[0] = nh.advertise<nav_msgs::Path>("/left/path_opti", 100);
        pub_map[0]       = nh.advertise<sensor_msgs::PointCloud2>("/left/global_map", 5);
        pub_curr_edge[0] = nh.advertise<sensor_msgs::PointCloud2>("/left/curr_edge", 10);
        pub_curr_surf[0] = nh.advertise<sensor_msgs::PointCloud2>("/left/curr_surf", 10);

        pub_path_odom[1] = nh.advertise<nav_msgs::Path>("/right/path_odom", 100);
        pub_path_opti[1] = nh.advertise<nav_msgs::Path>("/right/path_opti", 100);
        pub_map[1]       = nh.advertise<sensor_msgs::PointCloud2>("/right/global_map", 5);
        pub_curr_edge[1] = nh.advertise<sensor_msgs::PointCloud2>("/right/curr_edge", 10);
        pub_curr_surf[1] = nh.advertise<sensor_msgs::PointCloud2>("/right/curr_surf", 10);

    }

public:
    LidarSensor lidarSensor0;
    LidarSensor lidarSensor1;
private:

    /*********************************************************************
   ** Global Variables
   *********************************************************************/
    ros::NodeHandle nh;
    MapGenerator mapGenerator;

    double save_map_resolution;

    LidarMsgReader lidarMsgReader_0, lidarMsgReader_1;
    KeyframeVec::Ptr keyframeVec_0, keyframeVec_1;
    ros::Time time_0, time_1;

    // ros msg
    std::vector<ros::Subscriber> sub_laser_cloud, sub_laser_cloud_edge, sub_laser_cloud_surf;
    std::vector<ros::Publisher> pub_path_odom, pub_path_opti, pub_map, pub_curr_edge, pub_curr_surf;
    std::vector<nav_msgs::Path> path_odom, path_opti;

    /*********************************************************************
   ** Map Generator
   *********************************************************************/
    int save_latency = 0;
    int last_map_generate_index = 0;

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
//    std::thread laser_odometry_thread_1{&LaserOdometry::laser_odometry_auxiliary, &laserOdometry};
    std::thread backend_ba_optimization_0{&LidarSensor::BA_optimization, &laserOdometry.lidarSensor0};
//    std::thread backend_ba_optimization_1{&LidarSensor::BA_optimization, &laserOdometry.lidarSensor1};

    std::thread global_map_publisher{&LaserOdometry::pub_global_map, &laserOdometry};

    ros::spin();

}