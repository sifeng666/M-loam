//
// Created by ziv on 2020/10/26.
//

#include "helper.h"
#include "lidar.h"
#include "utils.h"

class LidarInfo {
public:
    using Ptr = boost::shared_ptr<LidarInfo>;
public:
    int i;
    LidarMsgReader reader;
    gtsam::Pose3 odom;
    ros::Time ros_time;
    KeyframeVec::Ptr keyframeVec;
    LidarStatus::Ptr status;
    MapGenerator mapGenerator;
    int last_map_generate_index = 0;
    gtsam::Pose3 T_0_i; // extrinsic param from L_i to L_base(L_0)
    bool is_base = false;
    tf::TransformBroadcaster brMapToFrame;

    // ros msg
    ros::Subscriber sub_laser_cloud, sub_laser_cloud_edge, sub_laser_cloud_surf, sub_laser_cloud_less_edge, sub_laser_cloud_less_surf;
    ros::Publisher pub_path_odom, pub_path_opti, pub_map, pub_curr_edge, pub_curr_surf;
    nav_msgs::Path path_odom, path_opti;

public:
    LidarInfo(int i_) : i(i_) {
        T_0_i = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(10 * i, 20 * i, 10 * i));
        if (i == 0) is_base = true;
    }
};

class LaserOdometry {
public:

    void pubRawOdom(LidarInfo::Ptr lidarInfo) {
        std::string child_frame_id = "frame" + std::to_string(lidarInfo->i);
        std::string frame_id = "map" + std::to_string(lidarInfo->i);
        gtsam::Pose3 odom;

        ros::Time cloud_in_time = lidarInfo->ros_time;
        auto odometry_odom = poseToNavOdometry(cloud_in_time, odom, frame_id, child_frame_id);

        tf::Transform transform;
        transform.setOrigin( tf::Vector3(odom.translation().x(), odom.translation().y(), odom.translation().z()) );
        Eigen::Quaterniond q_current(odom.rotation().matrix());
        tf::Quaternion q(q_current.x(), q_current.y(), q_current.z(), q_current.w());
        transform.setRotation(q);
        lidarInfo->brMapToFrame.sendTransform(tf::StampedTransform(transform, cloud_in_time, frame_id, child_frame_id));

        geometry_msgs::PoseStamped laserPose;
        laserPose.header = odometry_odom.header;
        laserPose.pose = odometry_odom.pose.pose;
        lidarInfo->path_odom.header.stamp = odometry_odom.header.stamp;
        lidarInfo->path_odom.poses.push_back(laserPose);
        lidarInfo->path_odom.header.frame_id = frame_id;
        lidarInfo->pub_path_odom.publish(lidarInfo->path_odom);
    }


    void pubOptiOdom(LidarInfo::Ptr lidarInfo) {
        geometry_msgs::PoseStamped laserPose;
        KeyframeVec::Ptr keyframeVec = lidarInfo->keyframeVec;
        std::string frame_id = "map" + std::to_string(lidarInfo->i);
        // read lock
        std::vector<gtsam::Pose3> poseVec = keyframeVec->read_poses(0, keyframeVec->keyframes.size());
        lidarInfo->path_opti.poses.clear();
        for (size_t i = 0; i < poseVec.size(); i++) {

            laserPose.header.frame_id = frame_id;
            Eigen::Quaterniond q(poseVec[i].rotation().matrix());
            laserPose.pose.orientation.x    = q.x();
            laserPose.pose.orientation.y    = q.y();
            laserPose.pose.orientation.z    = q.z();
            laserPose.pose.orientation.w    = q.w();
            laserPose.pose.position.x       = poseVec[i].translation().x();
            laserPose.pose.position.y       = poseVec[i].translation().y();
            laserPose.pose.position.z       = poseVec[i].translation().z();
            lidarInfo->path_opti.poses.push_back(laserPose);
        }
        lidarInfo->path_opti.header.frame_id = frame_id;
        lidarInfo->pub_path_opti.publish(lidarInfo->path_opti);
    }


    void pubRawPointCloud(LidarInfo::Ptr lidarInfo, pcl::PointCloud<PointT>::Ptr cloud_in_edge, pcl::PointCloud<PointT>::Ptr cloud_in_surf) {
        std::string child_frame_id = "frame" + std::to_string(lidarInfo->i);
        ros::Time cloud_in_time = lidarInfo->ros_time;
        sensor_msgs::PointCloud2Ptr edge_msg(new sensor_msgs::PointCloud2());
        sensor_msgs::PointCloud2Ptr surf_msg(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(*cloud_in_edge, *edge_msg);
        pcl::toROSMsg(*cloud_in_surf, *surf_msg);
        edge_msg->header.stamp = cloud_in_time;
        edge_msg->header.frame_id = child_frame_id;
        surf_msg->header.stamp = cloud_in_time;
        surf_msg->header.frame_id = child_frame_id;
        lidarInfo->pub_curr_edge.publish(edge_msg);
        lidarInfo->pub_curr_surf.publish(surf_msg);
    }


    int gen_map(LidarInfo::Ptr lidarInfo, pcl::PointCloud<PointT>::Ptr& map) {
        KeyframeVec::Ptr keyframeVec = lidarInfo->keyframeVec;
        if (!keyframeVec) {
            map = nullptr;
            return -1;
        }
        int size = int(keyframeVec->keyframes.size());
        if (size > 0 && keyframeVec->keyframes[size - 1]->is_init()) {
            lidarInfo->mapGenerator.clear();
            lidarInfo->mapGenerator.insert(keyframeVec, 0, size);
            map = lidarInfo->mapGenerator.get(save_map_resolution);
            return size;
        }
        map = nullptr;
        return -1;
    }


    void pub_global_map() {

        while (ros::ok()) {

            // deal with map 0
            {
                std::string frame_id = "map0";
                pcl::PointCloud<PointT>::Ptr map;
                int size = gen_map(lidarInfo0, map);
                if (size < 0 || map == nullptr) {
                    continue;
                }
                sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2());
                pcl::toROSMsg(*map, *cloud_msg);
                cloud_msg->header.stamp = lidarInfo0->ros_time;
                cloud_msg->header.frame_id = frame_id;
                lidarInfo0->pub_map.publish(cloud_msg);

                // save map 0
                {
                    if (lidarInfo0->last_map_generate_index == size) {
                        save_latency_0++;
                    } else {
                        save_latency_0 = 0;
                    }
                    if (save_latency_0 == 20) {
                        pcl::io::savePCDFileASCII("/home/ziv/mloam/global_map_0.pcd", *map);
                        cout << "saved map_0!" << endl;
                    }

                    lidarInfo0->last_map_generate_index = size;
                }
            }

            // deal with map 1
            {
                std::string frame_id = "map1";
                pcl::PointCloud<PointT>::Ptr map;
                int size = gen_map(lidarInfo1, map);
                if (size < 0 || map == nullptr) {
                    continue;
                }
                // transform to T_0_1
//                pcl::transformPointCloud(*map, *map, lidarInfo1->T_0_i.matrix());
                sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2());
                pcl::toROSMsg(*map, *cloud_msg);
                cloud_msg->header.stamp = lidarInfo1->ros_time;
                cloud_msg->header.frame_id = frame_id;
                lidarInfo1->pub_map.publish(cloud_msg);

                // save map 1
                {
                    if (lidarInfo1->last_map_generate_index == size) {
                        save_latency_1++;
                    } else {
                        save_latency_1 = 0;
                    }
                    if (save_latency_1 == 20) {
                        pcl::io::savePCDFileASCII("/home/ziv/mloam/global_map_1.pcd", *map);
                        cout << "saved map_1!" << endl;
                    }

                    lidarInfo1->last_map_generate_index = size;
                }
            }


            //sleep 200 ms every time
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }

    }

    void laser_odometry_base() {

        while (ros::ok()) {

            if ( !lidarInfo0->reader.pointCloudEdgeBuf.empty() &&
                 !lidarInfo0->reader.pointCloudSurfBuf.empty() &&
                 !lidarInfo0->reader.pointCloudLessEdgeBuf.empty() &&
                 !lidarInfo0->reader.pointCloudLessSurfBuf.empty() ) {

                Timer t_laser_odometry("laser_odometry: " + lidarSensor0.get_lidar_name());

                lidarInfo0->reader.lock();

                if ( lidarInfo0->reader.pointCloudEdgeBuf.front()->header.stamp.toSec() != lidarInfo0->reader.pointCloudSurfBuf.front()->header.stamp.toSec() ) {
                    printf("unsync messeage!");
                    ROS_BREAK();
                }

                pcl::PointCloud<PointT>::Ptr cloud_in_edge(new pcl::PointCloud<PointT>());
                pcl::PointCloud<PointT>::Ptr cloud_in_surf(new pcl::PointCloud<PointT>());
                pcl::PointCloud<PointT>::Ptr cloud_in_less_edge(new pcl::PointCloud<PointT>());
                pcl::PointCloud<PointT>::Ptr cloud_in_less_surf(new pcl::PointCloud<PointT>());

                pcl::fromROSMsg(*lidarInfo0->reader.pointCloudEdgeBuf.front(), *cloud_in_edge);
                pcl::fromROSMsg(*lidarInfo0->reader.pointCloudSurfBuf.front(), *cloud_in_surf);
                pcl::fromROSMsg(*lidarInfo0->reader.pointCloudLessEdgeBuf.front(), *cloud_in_less_edge);
                pcl::fromROSMsg(*lidarInfo0->reader.pointCloudLessSurfBuf.front(), *cloud_in_less_surf);

                lidarInfo0->ros_time = lidarInfo0->reader.pointCloudEdgeBuf.front()->header.stamp;

                lidarInfo0->reader.pointCloudEdgeBuf.pop();
                lidarInfo0->reader.pointCloudSurfBuf.pop();
                lidarInfo0->reader.pointCloudLessEdgeBuf.pop();
                lidarInfo0->reader.pointCloudLessSurfBuf.pop();

                lidarInfo0->reader.unlock();

                lidarInfo0->odom = lidarSensor0.update(lidarInfo0->ros_time, cloud_in_edge, cloud_in_surf, cloud_in_less_edge, cloud_in_less_surf);
                pubRawOdom(lidarInfo0);
                pubOptiOdom(lidarInfo0);
                pubRawPointCloud(lidarInfo0, cloud_in_less_edge, cloud_in_less_surf);

                static tf::TransformBroadcaster br0;
                tf::Transform transform;
                transform.setOrigin( tf::Vector3(0, 0, 0) );
                tf::Quaternion q(tf::Quaternion::getIdentity());
                transform.setRotation(q);
                br0.sendTransform(tf::StampedTransform(transform, lidarInfo0->ros_time, "map", "map0"));

                t_laser_odometry.count();

            } else if (lidarInfo0->status->status_change) {
                lidarSensor0.updatePoses();
                pubOptiOdom(lidarInfo0);
            }

            //sleep 2 ms every time
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    void laser_odometry_auxiliary() {

        while (ros::ok()) {

            if ( !lidarInfo1->reader.pointCloudEdgeBuf.empty() &&
                 !lidarInfo1->reader.pointCloudSurfBuf.empty() &&
                 !lidarInfo1->reader.pointCloudLessEdgeBuf.empty() &&
                 !lidarInfo1->reader.pointCloudLessSurfBuf.empty() ) {

                Timer t_laser_odometry("laser_odometry: " + lidarSensor1.get_lidar_name());

                lidarInfo1->reader.lock();

                if ( lidarInfo1->reader.pointCloudEdgeBuf.front()->header.stamp.toSec() != lidarInfo1->reader.pointCloudSurfBuf.front()->header.stamp.toSec() ) {
                    printf("unsync messeage!");
                    ROS_BREAK();
                }

                pcl::PointCloud<PointT>::Ptr cloud_in_edge(new pcl::PointCloud<PointT>());
                pcl::PointCloud<PointT>::Ptr cloud_in_surf(new pcl::PointCloud<PointT>());
                pcl::PointCloud<PointT>::Ptr cloud_in_less_edge(new pcl::PointCloud<PointT>());
                pcl::PointCloud<PointT>::Ptr cloud_in_less_surf(new pcl::PointCloud<PointT>());

                pcl::fromROSMsg(*lidarInfo1->reader.pointCloudEdgeBuf.front(), *cloud_in_edge);
                pcl::fromROSMsg(*lidarInfo1->reader.pointCloudSurfBuf.front(), *cloud_in_surf);
                pcl::fromROSMsg(*lidarInfo1->reader.pointCloudLessEdgeBuf.front(), *cloud_in_less_edge);
                pcl::fromROSMsg(*lidarInfo1->reader.pointCloudLessSurfBuf.front(), *cloud_in_less_surf);

                lidarInfo1->ros_time = lidarInfo1->reader.pointCloudEdgeBuf.front()->header.stamp;

                lidarInfo1->reader.pointCloudEdgeBuf.pop();
                lidarInfo1->reader.pointCloudSurfBuf.pop();
                lidarInfo1->reader.pointCloudLessEdgeBuf.pop();
                lidarInfo1->reader.pointCloudLessSurfBuf.pop();

                lidarInfo1->reader.unlock();

                lidarInfo1->odom = lidarSensor1.update(lidarInfo1->ros_time, cloud_in_edge, cloud_in_surf, cloud_in_less_edge, cloud_in_less_surf);
                pubRawOdom(lidarInfo1);
                pubOptiOdom(lidarInfo1);
                pubRawPointCloud(lidarInfo1, cloud_in_less_edge, cloud_in_less_surf);

                static tf::TransformBroadcaster br1;
                tf::Transform transform;
                transform.setOrigin( tf::Vector3(lidarInfo1->T_0_i.translation().x(), lidarInfo1->T_0_i.translation().y(), lidarInfo1->T_0_i.translation().z()) );
                Eigen::Quaterniond q_current(lidarInfo1->T_0_i.rotation().matrix());
                tf::Quaternion q(q_current.x(), q_current.y(), q_current.z(), q_current.w());
                transform.setRotation(q);
                br1.sendTransform(tf::StampedTransform(transform, lidarInfo1->ros_time, "map", "map1"));

                t_laser_odometry.count();

            } else if (lidarInfo1->status->status_change) {
                lidarSensor1.updatePoses();
                pubOptiOdom(lidarInfo1);
            }

            //sleep 2 ms every time
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }


    LaserOdometry() {

        save_map_resolution = nh.param<double>("save_map_resolution", 0.1);

        lidarSensor0.setName("Lidar0");
        lidarSensor1.setName("Lidar1");
        lidarInfo0 = boost::make_shared<LidarInfo>(0);
        lidarInfo1 = boost::make_shared<LidarInfo>(1);

        lidarInfo0->keyframeVec = lidarSensor0.get_keyframeVec();
        lidarInfo1->keyframeVec = lidarSensor1.get_keyframeVec();
        lidarInfo0->status = lidarSensor0.get_status();
        lidarInfo1->status = lidarSensor1.get_status();

        lidarInfo0->sub_laser_cloud_edge      = nh.subscribe<sensor_msgs::PointCloud2>("/left/laser_cloud_edge",  100, &LidarMsgReader::pointCloudEdgeHandler, &lidarInfo0->reader);
        lidarInfo0->sub_laser_cloud_surf      = nh.subscribe<sensor_msgs::PointCloud2>("/left/laser_cloud_surf",  100, &LidarMsgReader::pointCloudSurfHandler, &lidarInfo0->reader);
        lidarInfo0->sub_laser_cloud_less_edge = nh.subscribe<sensor_msgs::PointCloud2>("/left/laser_cloud_less_edge",  100, &LidarMsgReader::pointCloudLessEdgeHandler, &lidarInfo0->reader);
        lidarInfo0->sub_laser_cloud_less_surf = nh.subscribe<sensor_msgs::PointCloud2>("/left/laser_cloud_less_surf",  100, &LidarMsgReader::pointCloudLessSurfHandler, &lidarInfo0->reader);
        lidarInfo1->sub_laser_cloud_edge      = nh.subscribe<sensor_msgs::PointCloud2>("/right/laser_cloud_edge", 100, &LidarMsgReader::pointCloudEdgeHandler, &lidarInfo1->reader);
        lidarInfo1->sub_laser_cloud_surf      = nh.subscribe<sensor_msgs::PointCloud2>("/right/laser_cloud_surf", 100, &LidarMsgReader::pointCloudSurfHandler, &lidarInfo1->reader);
        lidarInfo1->sub_laser_cloud_less_edge = nh.subscribe<sensor_msgs::PointCloud2>("/right/laser_cloud_less_edge", 100, &LidarMsgReader::pointCloudLessEdgeHandler, &lidarInfo1->reader);
        lidarInfo1->sub_laser_cloud_less_surf = nh.subscribe<sensor_msgs::PointCloud2>("/right/laser_cloud_less_surf", 100, &LidarMsgReader::pointCloudLessSurfHandler, &lidarInfo1->reader);

        lidarInfo0->pub_path_odom = nh.advertise<nav_msgs::Path>("/left/path_odom", 100);
        lidarInfo0->pub_path_opti = nh.advertise<nav_msgs::Path>("/left/path_opti", 100);
        lidarInfo0->pub_map       = nh.advertise<sensor_msgs::PointCloud2>("/left/global_map", 5);
        lidarInfo0->pub_curr_edge = nh.advertise<sensor_msgs::PointCloud2>("/left/curr_edge", 10);
        lidarInfo0->pub_curr_surf = nh.advertise<sensor_msgs::PointCloud2>("/left/curr_surf", 10);

        lidarInfo1->pub_path_odom = nh.advertise<nav_msgs::Path>("/right/path_odom", 100);
        lidarInfo1->pub_path_opti = nh.advertise<nav_msgs::Path>("/right/path_opti", 100);
        lidarInfo1->pub_map       = nh.advertise<sensor_msgs::PointCloud2>("/right/global_map", 5);
        lidarInfo1->pub_curr_edge = nh.advertise<sensor_msgs::PointCloud2>("/right/curr_edge", 10);
        lidarInfo1->pub_curr_surf = nh.advertise<sensor_msgs::PointCloud2>("/right/curr_surf", 10);

    }

public:
    LidarSensor lidarSensor0;
    LidarSensor lidarSensor1;
private:

    /*********************************************************************
   ** Global Variables
   *********************************************************************/
    ros::NodeHandle nh;

    double save_map_resolution;

    LidarInfo::Ptr lidarInfo0, lidarInfo1;

    int save_latency_0 = 0, save_latency_1 = 0;

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

    std::thread global_map_publisher{&LaserOdometry::pub_global_map, &laserOdometry};

    ros::spin();

}