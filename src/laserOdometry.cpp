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

    void laser_odometry_base() {

        while (ros::ok()) {

            if ( !lidarInfo0->reader.pointCloudFullBuf.empty() &&
                 !lidarInfo0->reader.pointCloudLessEdgeBuf.empty() &&
                 !lidarInfo0->reader.pointCloudLessSurfBuf.empty() ) {

                Timer t_laser_odometry("laser_odometry: " + lidarSensor0.get_lidar_name());

                lidarInfo0->reader.lock();

                if ( lidarInfo0->reader.pointCloudFullBuf.front()->header.stamp.toSec() != lidarInfo0->reader.pointCloudLessEdgeBuf.front()->header.stamp.toSec() ) {
                    printf("unsync messeage!");
                    ROS_BREAK();
                }

                pcl::PointCloud<PointT>::Ptr cloud_raw(new pcl::PointCloud<PointT>());
                pcl::PointCloud<PointT>::Ptr cloud_in_edge(new pcl::PointCloud<PointT>());
                pcl::PointCloud<PointT>::Ptr cloud_in_surf(new pcl::PointCloud<PointT>());

                pcl::fromROSMsg(*lidarInfo0->reader.pointCloudFullBuf.front(), *cloud_raw);
                pcl::fromROSMsg(*lidarInfo0->reader.pointCloudLessEdgeBuf.front(), *cloud_in_edge);
                pcl::fromROSMsg(*lidarInfo0->reader.pointCloudLessSurfBuf.front(), *cloud_in_surf);

                lidarInfo0->ros_time = lidarInfo0->reader.pointCloudFullBuf.front()->header.stamp;

                lidarInfo0->reader.pointCloudFullBuf.pop();
                lidarInfo0->reader.pointCloudLessEdgeBuf.pop();
                lidarInfo0->reader.pointCloudLessSurfBuf.pop();

                lidarInfo0->reader.unlock();

                lidarInfo0->odom = lidarSensor0.update(lidarInfo0->ros_time, cloud_in_edge, cloud_in_surf, cloud_raw);
                pubOptiOdom(lidarInfo0);
                pubRawPointCloud(lidarInfo0, cloud_raw);

//                t_laser_odometry.count();

            } else {
                pubOptiOdom(lidarInfo0);
            }
            //sleep 2 ms every time
            std::this_thread::sleep_for(std::chrono::milliseconds(odom_sleep));
        }
    }

    void laser_odometry_auxiliary() {

        while (ros::ok()) {

            if ( !lidarInfo1->reader.pointCloudFullBuf.empty() &&
                 !lidarInfo1->reader.pointCloudLessEdgeBuf.empty() &&
                 !lidarInfo1->reader.pointCloudLessSurfBuf.empty() ) {

                Timer t_laser_odometry("laser_odometry: " + lidarSensor1.get_lidar_name());

                lidarInfo1->reader.lock();

                if ( lidarInfo1->reader.pointCloudFullBuf.front()->header.stamp.toSec() != lidarInfo1->reader.pointCloudLessEdgeBuf.front()->header.stamp.toSec() ) {
                    printf("unsync messeage!");
                    ROS_BREAK();
                }

                pcl::PointCloud<PointT>::Ptr cloud_raw(new pcl::PointCloud<PointT>());
                pcl::PointCloud<PointT>::Ptr cloud_in_edge(new pcl::PointCloud<PointT>());
                pcl::PointCloud<PointT>::Ptr cloud_in_surf(new pcl::PointCloud<PointT>());

                pcl::fromROSMsg(*lidarInfo1->reader.pointCloudFullBuf.front(), *cloud_raw);
                pcl::fromROSMsg(*lidarInfo1->reader.pointCloudLessEdgeBuf.front(), *cloud_in_edge);
                pcl::fromROSMsg(*lidarInfo1->reader.pointCloudLessSurfBuf.front(), *cloud_in_surf);

                lidarInfo1->ros_time = lidarInfo1->reader.pointCloudFullBuf.front()->header.stamp;

                lidarInfo1->reader.pointCloudFullBuf.pop();
                lidarInfo1->reader.pointCloudLessEdgeBuf.pop();
                lidarInfo1->reader.pointCloudLessSurfBuf.pop();

                lidarInfo1->reader.unlock();

                lidarInfo1->odom = lidarSensor1.update(lidarInfo1->ros_time, cloud_in_edge, cloud_in_surf, cloud_raw);
                pubOptiOdom(lidarInfo1);
                pubRawPointCloud(lidarInfo1, cloud_raw);

//                t_laser_odometry.count();

            } else {
                pubOptiOdom(lidarInfo1);
            }
            //sleep 2 ms every time
            std::this_thread::sleep_for(std::chrono::milliseconds(odom_sleep));
        }
    }

    void laser_calibration() {

        bool manual_tz = nh.param<bool>("manual_tz", true);
        double tz_0_1  = nh.param<double>("tz_0_1", 0.0);

        auto set_tz = [&](gtsam::Pose3& T_0_1) {
            if (manual_tz) {
                T_0_1 = gtsam::Pose3(T_0_1.rotation(), gtsam::Point3(T_0_1.x(), T_0_1.y(), tz_0_1));
            } else {
                auto gmap0 = MapGenerator::generate_cloud(lidarInfo0->keyframeVec, 0,
                                                          lidarInfo0->keyframeVec->keyframes.size(),
                                                          FeatureType::Full, true);
                auto gmap1 = MapGenerator::generate_cloud(lidarInfo1->keyframeVec, 0,
                                                          lidarInfo1->keyframeVec->keyframes.size(),
                                                          FeatureType::Full, true);
                Eigen::Matrix4d T;
                tools::FastGeneralizedRegistration(gmap0, gmap1, T, T_0_1.matrix());
                T_0_1 = gtsam::Pose3(T_0_1.rotation(), gtsam::Point3(T_0_1.x(), T_0_1.y(), T(12)));
            }
        };

        gtsam::Pose3 last_T;

        while(ros::ok()) {

            //sleep 2000 ms every time
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));

            bool encounter_loop = lidarInfo0->status->status_change &&  lidarInfo1->status->status_change &&
                    std::abs(lidarInfo0->status->last_loop_found_index - lidarInfo1->status->last_loop_found_index) < 5;
            bool calibra_simple_done = !calibra_with_loop && calibra_simple;
            bool start_calibra_with_loop = encounter_loop && calibra_simple_done;

            if (!calibra_simple) {
                gtsam::Pose3 T_0_1;
                bool success = HandEyeCalibrator::calibrate(lidarInfo0->keyframeVec, lidarInfo1->keyframeVec, T_0_1);
                if (!success) continue;
                std::cout << "hand eye calibrate convergence result: \n" << T_0_1.matrix() << std::endl;
                set_tz(T_0_1);

                if (T_0_1.equals(last_T, 0.01)) {
                    calibra_simple = true;
                    std::cout << "simple calibration done!" << std::endl;
                    lidarInfo1->T_0_i = T_0_1;
                }
                last_T = T_0_1;
            }

            if (start_calibra_with_loop) {
                gtsam::Pose3 T_0_1;
                bool success = HandEyeCalibrator::calibrate(lidarInfo0->keyframeVec, lidarInfo1->keyframeVec, T_0_1);
                if (!success) continue;
                std::cout << "hand eye calibrate fixed result: \n" << T_0_1.matrix() << std::endl;
                set_tz(T_0_1);

                calibra_with_loop = true;
                std::cout << "calibration with loop done!" << std::endl;
                lidarInfo1->T_0_i = T_0_1;
            }

        }

    }

    void global_calibration() {

        bool require_loop         = nh.param<bool>("require_loop", true);
        double surf_filter_length = nh.param<double>("surf_filter_length",  0.4);
        double corn_filter_length = nh.param<double>("corn_filter_length",  0.2);

        KeyframeVec::Ptr keyframeVec0 = lidarInfo0->keyframeVec;
        KeyframeVec::Ptr keyframeVec1 = lidarInfo1->keyframeVec;

        // submap and kdtree from Lidar0
        pcl::PointCloud<PointT>::Ptr submap_corn, submap_surf;
        pcl::KdTreeFLANN<PointT> kdtree_corn, kdtree_surf;

        gtsam::Pose3 pose_w_c;

        while (ros::ok()) {

            //sleep 20 ms every time
            std::this_thread::sleep_for(std::chrono::milliseconds(20));

            if (require_loop && calibra_with_loop) {

                int l0_end_cursor = lidarInfo0->status->last_loop_found_index;
                int l1_end_cursor = lidarInfo1->status->last_loop_found_index;

                auto poseVec0 = keyframeVec0->read_poses(0, l0_end_cursor + 1);
                auto poseVec1 = keyframeVec1->read_poses(0, l1_end_cursor + 1);

                if (std::abs(l0_end_cursor - l1_end_cursor) < 5) {

                    assert(keyframeVec0->at(l0_end_cursor)->is_fixed() && keyframeVec1->at(l1_end_cursor)->is_fixed());

                    gtsam::Pose3 T_0_1 = lidarInfo1->T_0_i;

                    TicToc t1;
                    submap_corn = MapGenerator::generate_cloud(keyframeVec0, 0, l0_end_cursor + 1, FeatureType::Edge,
                                                               true);
                    submap_surf = MapGenerator::generate_cloud(keyframeVec0, 0, l0_end_cursor + 1, FeatureType::Surf,
                                                               true);
                    cout << "generate submap takes: " << t1.toc() << "ms" << endl;
                    t1.tic();

                    down_sampling_voxel(*submap_corn, corn_filter_length);
                    down_sampling_voxel(*submap_surf, surf_filter_length);
                    cout << "down_sampling_voxel takes: " << t1.toc() << "ms" << endl;

                    kdtree_corn.setInputCloud(submap_corn);
                    kdtree_surf.setInputCloud(submap_surf);

                    gtsam::NonlinearFactorGraph factors;
                    gtsam::Values init_values;
                    auto state_key = X(0);
                    init_values.insert(state_key, T_0_1);
                    t1.tic();
                    for (int i = 0; i <= l1_end_cursor; i++) {

                        // 'pose_w_c' to be optimize
                        pose_w_c = poseVec0[i] * T_0_1;

                        //  corn and surf features have already downsample when push to keyframeVec
                        LidarSensor::addCornCostFactor(keyframeVec1->at(i)->corn_features, submap_corn, kdtree_corn,
                                                       pose_w_c, factors, poseVec0[i]);
                        LidarSensor::addSurfCostFactor(keyframeVec1->at(i)->surf_features, submap_surf, kdtree_surf,
                                                       pose_w_c, factors, poseVec0[i]);

                    }
                    printf("add factor takes %f ms\n", t1.toc());
                    t1.tic();
                    // gtsam
                    gtsam::LevenbergMarquardtParams params;
                    params.setLinearSolverType("MULTIFRONTAL_CHOLESKY");
                    params.setRelativeErrorTol(5e-3);
                    params.maxIterations = 10;

                    auto result = gtsam::LevenbergMarquardtOptimizer(factors, init_values, params).optimize();
                    pose_w_c = result.at<gtsam::Pose3>(state_key);
                    cout << "before T_0_1: \n" << T_0_1.matrix() << endl;
                    cout << "after opti: \n" << pose_w_c.matrix() << endl;
                    printf("gtsam takes %f ms\n", t1.toc());
                }

            }
        }

    }


    LaserOdometry() {

        save_map_resolution = nh.param<double>("save_map_resolution", 0.1);
        show_map_resolution = nh.param<double>("show_map_resolution", 0.2);
        file_save_path = nh.param<std::string>("file_save_path", "");

        // lidar 0 init
        lidarSensor0.set_name("Lidar0");
        lidarInfo0 = std::make_shared<LidarInfo>(0);
        lidarInfo0->keyframeVec = lidarSensor0.get_keyframeVec();
        lidarInfo0->status = lidarSensor0.status;
        lidarInfo0->fixedChannel = lidarSensor0.fixedKeyframeChannel;

        // lidar 1 init
        lidarSensor1.set_name("Lidar1");
        lidarInfo1 = std::make_shared<LidarInfo>(1);
        lidarInfo1->keyframeVec = lidarSensor1.get_keyframeVec();
        lidarInfo1->status = lidarSensor1.status;
        lidarInfo1->fixedChannel = lidarSensor1.fixedKeyframeChannel;

        lidarInfo0->sub_laser_cloud           = nh.subscribe<sensor_msgs::PointCloud2>("/left/velodyne_points_2",  100, &LidarMsgReader::pointCloudFullHandler, &lidarInfo0->reader);
        lidarInfo0->sub_laser_cloud_less_edge = nh.subscribe<sensor_msgs::PointCloud2>("/left/laser_cloud_less_edge",  100, &LidarMsgReader::pointCloudLessEdgeHandler, &lidarInfo0->reader);
        lidarInfo0->sub_laser_cloud_less_surf = nh.subscribe<sensor_msgs::PointCloud2>("/left/laser_cloud_less_surf",  100, &LidarMsgReader::pointCloudLessSurfHandler, &lidarInfo0->reader);
        lidarInfo0->pub_odom_opti             = nh.advertise<nav_msgs::Path>(          "/left/odom_opti", 100);
        lidarInfo0->pub_map                   = nh.advertise<sensor_msgs::PointCloud2>("/left/global_map", 5);
        lidarInfo0->pub_pointcloud_raw        = nh.advertise<sensor_msgs::PointCloud2>("/left/raw_points", 10);

        lidarInfo1->sub_laser_cloud           = nh.subscribe<sensor_msgs::PointCloud2>("/right/velodyne_points_2",  100, &LidarMsgReader::pointCloudFullHandler, &lidarInfo1->reader);
        lidarInfo1->sub_laser_cloud_less_edge = nh.subscribe<sensor_msgs::PointCloud2>("/right/laser_cloud_less_edge", 100, &LidarMsgReader::pointCloudLessEdgeHandler, &lidarInfo1->reader);
        lidarInfo1->sub_laser_cloud_less_surf = nh.subscribe<sensor_msgs::PointCloud2>("/right/laser_cloud_less_surf", 100, &LidarMsgReader::pointCloudLessSurfHandler, &lidarInfo1->reader);
        lidarInfo1->pub_odom_opti             = nh.advertise<nav_msgs::Path>(          "/right/odom_opti", 100);
        lidarInfo1->pub_map                   = nh.advertise<sensor_msgs::PointCloud2>("/right/global_map", 5);
        lidarInfo1->pub_pointcloud_raw        = nh.advertise<sensor_msgs::PointCloud2>("/right/raw_points", 10);;

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
    std::thread backend_ba_optimization_0{&LidarSensor::BA_optimization, &laserOdometry.lidarSensor0};
    std::thread backend_loop_detector_0{&LidarSensor::loop_detect_thread, &laserOdometry.lidarSensor0};

    std::thread laser_odometry_thread_1{&LaserOdometry::laser_odometry_auxiliary, &laserOdometry};
    std::thread backend_ba_optimization_1{&LidarSensor::BA_optimization, &laserOdometry.lidarSensor1};
    std::thread backend_loop_detector_1{&LidarSensor::loop_detect_thread, &laserOdometry.lidarSensor1};

    std::thread laser_calibration_thread{&LaserOdometry::laser_calibration, &laserOdometry};
    std::thread global_calibration_thread{&LaserOdometry::global_calibration, &laserOdometry};

    std::thread global_map_publisher{&LaserOdometry::pub_global_map, &laserOdometry};

    ros::spin();

}