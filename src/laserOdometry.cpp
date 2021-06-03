//
// Created by ziv on 2020/10/26.
//

#include "helper.h"
#include "utils.h"
#include "lidar.h"
#include "imu.h"
#include "tools/calibration.hpp"

using namespace tools;

static const int odom_sleep = 2;    // 2ms
static const int lidar_count = 2;

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
    Channel<Frame::Ptr>::Ptr frameChannel;
    tf::TransformBroadcaster brMapToFrame;  // tf broadcaster, map_i => frame_i
    tf::TransformBroadcaster brMapToMap;    // tf broadcaster, map   => map_i

    gtsam::Pose3 T_0_i;                     // extrinsic param from L_0 to L_base(L_i)

    // ros msg
    ros::Subscriber sub_laser_cloud;
    ros::Subscriber sub_laser_cloud_less_edge;
    ros::Subscriber sub_laser_cloud_less_surf;

    ros::Subscriber sub_laser_cloud_edge;
    ros::Subscriber sub_laser_cloud_surf;

    ros::Publisher pub_map;
    ros::Publisher pub_pointcloud_raw;
    ros::Publisher pub_odom_opti;
    nav_msgs::Path parray_path_opti;
    ros::Publisher pub_odom_mapping;
    nav_msgs::Path parray_path_mapping;

    //add for imu
    ros::Publisher pub_odometry_type;

public:
    LidarInfo(int i_) : i(i_) {
        T_0_i = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 50 * i, 0));
    }
};

class LaserOdometry {
public:
    //
    void pubOptiOdom(LidarInfo::Ptr lidarInfo) {
        if (lidarInfo->keyframeVec->keyframes.empty()) return;
        auto &parray_path = lidarInfo->parray_path_opti;
        KeyframeVec::Ptr keyframeVec = lidarInfo->keyframeVec;
        // read lock
        std::vector<gtsam::Pose3> poseVec = keyframeVec->read_poses(0, keyframeVec->keyframes.size(), true);

        parray_path.header.stamp = ros::Time::now();
        parray_path.header.frame_id = "map" + std::to_string(lidarInfo->i);

        //add all the pose before into parray_path including the current pose, then publish the newest parray_path
        for (size_t i = 0; i < poseVec.size(); i++) {
            Eigen::Quaterniond q(poseVec[i].rotation().matrix());
            if (i < parray_path.poses.size()) { //parray_path with the size
                parray_path.poses[i].pose.orientation.w = q.w();
                parray_path.poses[i].pose.orientation.x = q.x();
                parray_path.poses[i].pose.orientation.y = q.y();
                parray_path.poses[i].pose.orientation.z = q.z();
                parray_path.poses[i].pose.position.x = poseVec[i].translation().x();
                parray_path.poses[i].pose.position.y = poseVec[i].translation().y();
                parray_path.poses[i].pose.position.z = poseVec[i].translation().z();
            } else {
                geometry_msgs::PoseStamped apose;
                apose.pose.orientation.w = q.w();
                apose.pose.orientation.x = q.x();
                apose.pose.orientation.y = q.y();
                apose.pose.orientation.z = q.z();
                apose.pose.position.x = poseVec[i].translation().x();
                apose.pose.position.y = poseVec[i].translation().y();
                apose.pose.position.z = poseVec[i].translation().z();
                parray_path.poses.push_back(apose);  //if oversize, new a apose and push it to the back of parray_path
            }
        }

        lidarInfo->pub_odom_opti.publish(parray_path);

        tf::Transform transform;
        gtsam::Pose3 T(lidarInfo->T_0_i);
        transform.setOrigin(tf::Vector3(T.x(), T.y(), T.z()));
        Eigen::Quaterniond q(T.rotation().matrix());
        transform.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
        lidarInfo->brMapToMap.sendTransform(
                tf::StampedTransform(transform, parray_path.header.stamp, "map", parray_path.header.frame_id));
    }

    void pubRawPointCloud(LidarInfo::Ptr lidarInfo, gtsam::Pose3 odom, pcl::PointCloud<PointT>::Ptr cloud_raw) {

        std::string frame_id = "map" + std::to_string(lidarInfo->i);
        ros::Time cloud_in_time = ros::Time::now();

        sensor_msgs::PointCloud2Ptr raw_msg(new sensor_msgs::PointCloud2());
        pcl::PointCloud<PointT> cloud_raw_transformed;
        pcl::transformPointCloud(*cloud_raw, cloud_raw_transformed, odom.matrix());
        pcl::toROSMsg(cloud_raw_transformed, *raw_msg);
        raw_msg->header.stamp = cloud_in_time;
        raw_msg->header.frame_id = frame_id;
        lidarInfo->pub_pointcloud_raw.publish(raw_msg);

        auto &parray_path = lidarInfo->parray_path_mapping;
        parray_path.header.stamp = cloud_in_time;
        parray_path.header.frame_id = frame_id;

        geometry_msgs::PoseStamped apose;
        auto q = odom.rotation().quaternion();
        apose.pose.orientation.w = q.w();
        apose.pose.orientation.x = q.x();
        apose.pose.orientation.y = q.y();
        apose.pose.orientation.z = q.z();
        apose.pose.position.x = odom.translation().x();
        apose.pose.position.y = odom.translation().y();
        apose.pose.position.z = odom.translation().z();
        parray_path.poses.push_back(apose);

        lidarInfo->pub_odom_mapping.publish(parray_path);

        //pub raw odomtry for imu
    }

    void full_map_handler(LidarInfo::Ptr lidarInfo) {
        std::string map_id = "map" + std::to_string(lidarInfo->i);

        auto map_full_pub = MapGenerator::generate_cloud(lidarInfo->keyframeVec, 0, lidarInfo->keyframeVec->size(),
                                                         FeatureType::Full, true);
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

    void save_map_handler(const std_msgs::StringConstPtr &str) {
        TicToc t_save;
        string filename = str->data;
        cout << "std_msgs string: " << filename << endl;

        cout << "save path: " << file_save_path + "*.pcd" << endl;
        _mkdir(file_save_path + "1.txt");

        std::ofstream f_mapping_pose;

        auto map = MapGenerator::generate_cloud(lidarInfo0->keyframeVec, 0, lidarInfo0->keyframeVec->size(),
                                                FeatureType::Full, true);
        if (map) {
            pcl::io::savePCDFileASCII(file_save_path + filename + "_0.pcd", *map);
            printf("global map 0 save! cost: %.3f ms\n", t_save.toc());
        }

        _mkdir(file_save_path + "pose/opti_pose_0.txt");
        f_mapping_pose.open(file_save_path + "pose/opti_pose_0.txt");
        for (int i = 0; i < lidarInfo0->keyframeVec->size(); i++) {
            Eigen::Quaterniond q(lidarInfo0->keyframeVec->at(i)->pose_fixed.rotation().matrix());
            auto &p = lidarInfo0->keyframeVec->at(i)->pose_fixed;
            f_mapping_pose << setprecision(20) << double(lidarInfo0->keyframeVec->at(i)->cloud_in_time.toNSec()) / 1e9
                           << " "
                           << p.translation().x() << " " << p.translation().y() << " " << p.translation().z() << " "
                           << q.x()
                           << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }
        f_mapping_pose.close();

        map = MapGenerator::generate_cloud(lidarInfo1->keyframeVec, 0, lidarInfo1->keyframeVec->size(),
                                           FeatureType::Full, true);
        if (map) {
            pcl::io::savePCDFileASCII(file_save_path + filename + "_1.pcd", *map);
            printf("global map 1 save! cost: %.3f ms\n", t_save.toc());
        }

        _mkdir(file_save_path + "pose/opti_pose_1.txt");
        f_mapping_pose.open(file_save_path + "pose/opti_pose_1.txt");
        for (int i = 0; i < lidarInfo1->keyframeVec->size(); i++) {
            Eigen::Quaterniond q(lidarInfo1->keyframeVec->at(i)->pose_fixed.rotation().matrix());
            auto &p = lidarInfo1->keyframeVec->at(i)->pose_fixed;
            f_mapping_pose << setprecision(20) << double(lidarInfo1->keyframeVec->at(i)->cloud_in_time.toNSec()) / 1e9
                           << " "
                           << p.translation().x() << " " << p.translation().y() << " " << p.translation().z() << " "
                           << q.x()
                           << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }
        f_mapping_pose.close();

    }

    void pub_global_map() {

        while (ros::ok()) {

            full_map_handler(lidarInfo0);

            full_map_handler(lidarInfo1);

            //sleep 200 ms every time
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }

    }

    void laser_odometry_fusion() {

        ros::Time sys_time;
        bool use_lidar0 = true;
        bool use_lidar1 = false;

        bool is_first = true;

        while (ros::ok()) {

            if (!go_fusion) {
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                continue;
            }

            if ((use_lidar0 &&
                 !lidarInfo0->reader.pointCloudFullBuf.empty() &&
                 !lidarInfo0->reader.pointCloudLessEdgeBuf.empty() &&
                 !lidarInfo0->reader.pointCloudLessSurfBuf.empty())
                ||
                (use_lidar1 &&
                 !lidarInfo1->reader.pointCloudFullBuf.empty() &&
                 !lidarInfo1->reader.pointCloudLessEdgeBuf.empty() &&
                 !lidarInfo1->reader.pointCloudLessSurfBuf.empty())) {

                TicToc tic;

                pcl::PointCloud<PointT>::Ptr cloud_raw(new pcl::PointCloud<PointT>());
                pcl::PointCloud<PointT>::Ptr cloud_in_edge(new pcl::PointCloud<PointT>());
                pcl::PointCloud<PointT>::Ptr cloud_in_surf(new pcl::PointCloud<PointT>());

                if (use_lidar0) {
                    lidarInfo0->reader.lock();
                    if (lidarInfo0->reader.pointCloudFullBuf.front()->header.stamp.toSec() !=
                        lidarInfo0->reader.pointCloudLessEdgeBuf.front()->header.stamp.toSec()) {
                        printf("unsync messeage!");
                        ROS_BREAK();
                    }

                    pcl::fromROSMsg(*lidarInfo0->reader.pointCloudFullBuf.front(), *cloud_raw);
                    pcl::fromROSMsg(*lidarInfo0->reader.pointCloudLessEdgeBuf.front(), *cloud_in_edge);
                    pcl::fromROSMsg(*lidarInfo0->reader.pointCloudLessSurfBuf.front(), *cloud_in_surf);

                    lidarInfo0->ros_time = lidarInfo0->reader.pointCloudFullBuf.front()->header.stamp;

                    lidarInfo0->reader.pointCloudFullBuf.pop();
                    lidarInfo0->reader.pointCloudLessEdgeBuf.pop();
                    lidarInfo0->reader.pointCloudLessSurfBuf.pop();

                    lidarInfo0->reader.unlock();

                    if (is_first) {
                        sys_time = lidarInfo0->ros_time;
                        is_first = false;
                    }

                    if (lidarInfo0->ros_time < sys_time) {
                        continue;
                    } else {
                        sys_time = lidarInfo0->ros_time;
                        use_lidar0 = false;
                        use_lidar1 = true;
                    }

                } else {
                    if (lidarInfo1->reader.pointCloudFullBuf.front()->header.stamp.toSec() !=
                        lidarInfo1->reader.pointCloudLessEdgeBuf.front()->header.stamp.toSec()) {
                        printf("unsync messeage!");
                        ROS_BREAK();
                    }

                    pcl::fromROSMsg(*lidarInfo1->reader.pointCloudFullBuf.front(), *cloud_raw);
                    pcl::fromROSMsg(*lidarInfo1->reader.pointCloudLessEdgeBuf.front(), *cloud_in_edge);
                    pcl::fromROSMsg(*lidarInfo1->reader.pointCloudLessSurfBuf.front(), *cloud_in_surf);

                    lidarInfo1->ros_time = lidarInfo1->reader.pointCloudFullBuf.front()->header.stamp;

                    lidarInfo1->reader.pointCloudFullBuf.pop();
                    lidarInfo1->reader.pointCloudLessEdgeBuf.pop();
                    lidarInfo1->reader.pointCloudLessSurfBuf.pop();

                    lidarInfo1->reader.unlock();

                    if (lidarInfo1->ros_time < sys_time) {
                        continue;
                    } else {
                        sys_time = lidarInfo1->ros_time;
                        use_lidar0 = true;
                        use_lidar1 = false;
                    }
                }

                gtsam::Pose3 unuse;
                lidarInfo0->odom = lidarSensor0.update_odom(lidarInfo0->ros_time, cloud_in_edge,
                                                            cloud_in_surf, cloud_raw, unuse);
                printf("[Odometry fusion]: %.3f ms\n", tic.toc());

            }
        }
    }

    void laser_mapping_fusion() {

        auto &frameChannel = lidarInfo0->frameChannel;

        while (ros::ok()) {

            auto frame = frameChannel->get_front();
            if (!frame) continue;
//            frameChannel->clear();

            TicToc t_mapping;

            auto odom = lidarSensor0.update_mapping(frame);
            pubOptiOdom(lidarInfo0);
            pubRawPointCloud(lidarInfo0, odom, frame->raw);

            double time_cost = t_mapping.toc();
            printf("[Mapping 0]: %.3f ms\n", time_cost);

            //sleep 1 ms every time
            std::this_thread::sleep_for(std::chrono::milliseconds(odom_sleep));
        }

    }

    void laser_odometry_base() {

        while (ros::ok()) {
            if (go_fusion) {
                return;
            }

            if (!lidarInfo0->reader.pointCloudFullBuf.empty() &&
                !lidarInfo0->reader.pointCloudLessEdgeBuf.empty() &&
                !lidarInfo0->reader.pointCloudLessSurfBuf.empty() &&
                !lidarInfo0->reader.pointCloudEdgeBuf.empty() &&
                !lidarInfo0->reader.pointCloudSurfBuf.empty()) {

                TicToc tic;

                lidarInfo0->reader.lock();

                if (lidarInfo0->reader.pointCloudFullBuf.front()->header.stamp.toSec() !=
                    lidarInfo0->reader.pointCloudLessEdgeBuf.front()->header.stamp.toSec()) {
                    printf("unsync messeage!");
                    ROS_BREAK();
                }

                pcl::PointCloud<PointT>::Ptr cloud_raw(new pcl::PointCloud<PointT>());
                pcl::PointCloud<PointT>::Ptr cloud_in_edge(new pcl::PointCloud<PointT>());
                pcl::PointCloud<PointT>::Ptr cloud_in_surf(new pcl::PointCloud<PointT>());
                pcl::PointCloud<PointT>::Ptr cloud_in_less_edge(new pcl::PointCloud<PointT>());
                pcl::PointCloud<PointT>::Ptr cloud_in_less_surf(new pcl::PointCloud<PointT>());

                pcl::fromROSMsg(*lidarInfo0->reader.pointCloudFullBuf.front(), *cloud_raw);
                pcl::fromROSMsg(*lidarInfo0->reader.pointCloudLessEdgeBuf.front(), *cloud_in_less_edge);
                pcl::fromROSMsg(*lidarInfo0->reader.pointCloudLessSurfBuf.front(), *cloud_in_less_surf);
                pcl::fromROSMsg(*lidarInfo0->reader.pointCloudEdgeBuf.front(), *cloud_in_edge);
                pcl::fromROSMsg(*lidarInfo0->reader.pointCloudSurfBuf.front(), *cloud_in_surf);

                lidarInfo0->ros_time = lidarInfo0->reader.pointCloudFullBuf.front()->header.stamp;

                lidarInfo0->reader.pointCloudFullBuf.pop();
                lidarInfo0->reader.pointCloudLessEdgeBuf.pop();
                lidarInfo0->reader.pointCloudLessSurfBuf.pop();
                lidarInfo0->reader.pointCloudEdgeBuf.pop();
                lidarInfo0->reader.pointCloudSurfBuf.pop();

                lidarInfo0->reader.unlock();

                gtsam::Pose3 unuse;
                lidarInfo0->odom = lidarSensor0.update_odom(lidarInfo0->ros_time, cloud_in_less_edge,
                                                            cloud_in_less_surf, cloud_raw, unuse);
                printf("[Odometry 0]: %.3f ms\n", tic.toc());
            }
            //sleep 2 ms every time
            std::this_thread::sleep_for(std::chrono::milliseconds(odom_sleep));
        }
    }

    void laser_odometry_sub() {

        while (ros::ok()) {
            if (go_fusion) {
                return;
            }

            if (!lidarInfo1->reader.pointCloudFullBuf.empty() &&
                !lidarInfo1->reader.pointCloudLessEdgeBuf.empty() &&
                !lidarInfo1->reader.pointCloudLessSurfBuf.empty() &&
                !lidarInfo1->reader.pointCloudEdgeBuf.empty() &&
                !lidarInfo1->reader.pointCloudSurfBuf.empty()) {

                TicToc tic;

                lidarInfo1->reader.lock();

                if (lidarInfo1->reader.pointCloudFullBuf.front()->header.stamp.toSec() !=
                    lidarInfo1->reader.pointCloudLessEdgeBuf.front()->header.stamp.toSec()) {
                    printf("unsync messeage!");
                    ROS_BREAK();
                }

                pcl::PointCloud<PointT>::Ptr cloud_raw(new pcl::PointCloud<PointT>());
                pcl::PointCloud<PointT>::Ptr cloud_in_edge(new pcl::PointCloud<PointT>());
                pcl::PointCloud<PointT>::Ptr cloud_in_surf(new pcl::PointCloud<PointT>());
                pcl::PointCloud<PointT>::Ptr cloud_in_less_edge(new pcl::PointCloud<PointT>());
                pcl::PointCloud<PointT>::Ptr cloud_in_less_surf(new pcl::PointCloud<PointT>());

                pcl::fromROSMsg(*lidarInfo1->reader.pointCloudFullBuf.front(), *cloud_raw);
                pcl::fromROSMsg(*lidarInfo1->reader.pointCloudLessEdgeBuf.front(), *cloud_in_less_edge);
                pcl::fromROSMsg(*lidarInfo1->reader.pointCloudLessSurfBuf.front(), *cloud_in_less_surf);
                pcl::fromROSMsg(*lidarInfo1->reader.pointCloudEdgeBuf.front(), *cloud_in_edge);
                pcl::fromROSMsg(*lidarInfo1->reader.pointCloudSurfBuf.front(), *cloud_in_surf);

                lidarInfo1->ros_time = lidarInfo1->reader.pointCloudFullBuf.front()->header.stamp;

                lidarInfo1->reader.pointCloudFullBuf.pop();
                lidarInfo1->reader.pointCloudLessEdgeBuf.pop();
                lidarInfo1->reader.pointCloudLessSurfBuf.pop();
                lidarInfo1->reader.pointCloudEdgeBuf.pop();
                lidarInfo1->reader.pointCloudSurfBuf.pop();

                lidarInfo1->reader.unlock();

                gtsam::Pose3 unuse;
                lidarInfo1->odom = lidarSensor1.update_odom(lidarInfo1->ros_time, cloud_in_less_edge,
                                                            cloud_in_less_surf, cloud_raw, unuse);
                printf("[Odometry 1]: %.3f ms\n", tic.toc());
            }
            //sleep 2 ms every time
            std::this_thread::sleep_for(std::chrono::milliseconds(odom_sleep));
        }
    }

    void laser_mapping_base() {

        auto &frameChannel = lidarInfo0->frameChannel;

        while (ros::ok()) {

            auto frame = frameChannel->get_front();
            if (!frame) continue;
//            frameChannel->clear();

            TicToc t_mapping;

            auto odom = lidarSensor0.update_mapping(frame);
            pubOptiOdom(lidarInfo0);
            pubRawPointCloud(lidarInfo0, odom, frame->raw);

            double time_cost = t_mapping.toc();
            printf("[Mapping 0]: %.3f ms\n", time_cost);

            //sleep 1 ms every time
            std::this_thread::sleep_for(std::chrono::milliseconds(odom_sleep));
        }

    }

    void laser_mapping_sub() {

        auto &frameChannel = lidarInfo1->frameChannel;

        while (ros::ok()) {

            auto frame = frameChannel->get_front();
            if (!frame) continue;
//            frameChannel->clear();

            TicToc t_mapping;

            auto odom = lidarSensor1.update_mapping(frame);
            pubOptiOdom(lidarInfo1);
            pubRawPointCloud(lidarInfo1, odom, frame->raw);

            double time_cost = t_mapping.toc();
            printf("[Mapping 1]: %.3f ms\n", time_cost);

            //sleep 1 ms every time
            std::this_thread::sleep_for(std::chrono::milliseconds(odom_sleep));
        }

    }


    void laser_calibration() {

        string hand_eye_convergence = file_save_path + "hand_eye_convergence.txt";
        std::ofstream f_hand_eye_convergence(hand_eye_convergence);

        string hand_eye_loop = file_save_path + "hand_eye_loop.txt";
        std::ofstream f_hand_eye_loop(hand_eye_loop);

        if (!need_calibra) return;

        bool manual_tz = nh.param<bool>("manual_tz", true);
        double tz_0_1 = nh.param<double>("tz_0_1", 0.0);

        auto set_tz = [&](gtsam::Pose3 &T_0_1) {
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
                tools::FastGeneralizedRegistration(gmap1, gmap0, T, T_0_1.matrix());
                T_0_1 = gtsam::Pose3(T_0_1.rotation(), gtsam::Point3(T_0_1.x(), T_0_1.y(), T(12)));
            }
        };

        gtsam::Pose3 last_T;

        int conv_time = 0;

        while (ros::ok()) {

            //sleep 2000 ms every time
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            bool encounter_loop = (lidarInfo0->status->status_change && lidarInfo1->status->status_change);
            bool calibra_simple_done = !calibra_with_loop && calibra_simple;
            bool start_calibra_with_loop = encounter_loop && calibra_simple_done;

            if (start_calibra_with_loop) {
                lidarInfo0->status->status_change = false;
                lidarInfo1->status->status_change = false;
            }
            cout << "calibra_simple: " << calibra_simple << ", start_calibra_with_loop" << start_calibra_with_loop
                 << endl;

            if (!calibra_simple) {
                gtsam::Pose3 T_0_1;
                bool success = HandEyeCalibrator::calibrate(lidarInfo1->keyframeVec, lidarInfo0->keyframeVec, T_0_1);
                if (!success) continue;
                std::cout << "hand eye calibrate convergence result: \n" << T_0_1.matrix() << std::endl;

                f_hand_eye_convergence << T_0_1.matrix() << std::endl << std::endl;

                set_tz(T_0_1);

                if (T_0_1.equals(last_T, 0.01)) {
                    calibra_simple = true;
                    std::cout << "simple calibration done!" << std::endl;

                    lidarInfo1->T_0_i = T_0_1;
                }
                last_T = T_0_1;
            }

            if (start_calibra_with_loop) {

                cout << "start start_calibra_with_loop!!!!" << endl;
                gtsam::Pose3 T_0_1;
                bool success = HandEyeCalibrator::calibrate(lidarInfo1->keyframeVec, lidarInfo0->keyframeVec, T_0_1);
                if (!success) continue;
                std::cout << "hand eye calibrate fixed result: \n" << T_0_1.matrix() << std::endl;
                f_hand_eye_loop << T_0_1.matrix() << std::endl << std::endl;

                set_tz(T_0_1);

                calibra_with_loop = true;
                std::cout << "calibration with loop done!" << std::endl;
                lidarInfo1->T_0_i = T_0_1;
            }

        }

    }

    void global_calibration() {

//        return;

        bool require_loop = nh.param<bool>("require_loop", true);
        double surf_filter_length = nh.param<double>("surf_filter_length", 0.4);
        double corn_filter_length = nh.param<double>("corn_filter_length", 0.2);

        KeyframeVec::Ptr keyframeVec0 = lidarInfo0->keyframeVec;
        KeyframeVec::Ptr keyframeVec1 = lidarInfo1->keyframeVec;

        // submap and kdtree from Lidar0
        pcl::PointCloud<PointT>::Ptr submap_corn, submap_surf;
        pcl::KdTreeFLANN<PointT> kdtree_corn, kdtree_surf;

        gtsam::Pose3 pose_w0_w1, pose_w0_curr;

        string calibration_opti = file_save_path + "calibration_opti.txt";
        std::ofstream f_calibration_opti(calibration_opti);

        while (ros::ok()) {

            //sleep 20 ms every time
            std::this_thread::sleep_for(std::chrono::milliseconds(20));

            if (!need_calibra) return;

            if (require_loop && calibra_with_loop) {

                int l0_end_cursor = lidarInfo0->status->last_loop_found_index;
                int l1_end_cursor = lidarInfo1->status->last_loop_found_index;

                auto poseVec_w0_curr = keyframeVec0->read_poses(0, l0_end_cursor + 1);
                auto poseVec_w1_curr = keyframeVec1->read_poses(0, l1_end_cursor + 1);

                if (std::abs(l0_end_cursor - l1_end_cursor) < 5) {

                    assert(keyframeVec0->at(l0_end_cursor)->is_fixed() && keyframeVec1->at(l1_end_cursor)->is_fixed());

                    TicToc t1;
                    submap_corn = MapGenerator::generate_cloud(keyframeVec0, 0, l0_end_cursor + 1, FeatureType::Corn,
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
                    gtsam::Pose3 pose_w0_w1 = lidarInfo1->T_0_i;

                    init_values.insert(state_key, pose_w0_w1);

                    for (int k = 0; k < 3; k++) {

                        t1.tic();

                        for (int i = 0; i <= l1_end_cursor; i++) {

                            pose_w0_curr = pose_w0_w1 * poseVec_w1_curr[i];

                            //  corn and surf features have already downsample when push to keyframeVec
                            LidarSensor::addCornCostFactor(keyframeVec1->at(i)->corn_features, submap_corn, kdtree_corn,
                                                           pose_w0_curr, factors, poseVec_w1_curr[i]);
                            LidarSensor::addSurfCostFactor(keyframeVec1->at(i)->surf_features, submap_surf, kdtree_surf,
                                                           pose_w0_curr, factors, poseVec_w1_curr[i]);

                        }

                        printf("add factor takes %f ms\n", t1.toc());
                        t1.tic();
                        // gtsam
                        gtsam::LevenbergMarquardtParams params;
                        params.setLinearSolverType("MULTIFRONTAL_CHOLESKY");
                        params.setRelativeErrorTol(5e-3);
                        params.maxIterations = 6;

                        auto result = gtsam::LevenbergMarquardtOptimizer(factors, init_values, params).optimize();
                        pose_w0_w1 = result.at<gtsam::Pose3>(state_key);

                    }

                    cout << "before T_0_1: \n" << lidarInfo1->T_0_i.matrix() << endl;
                    cout << "after opti: \n" << pose_w0_w1.matrix() << endl;
                    f_calibration_opti << pose_w0_w1.matrix() << endl;

                    printf("gtsam takes %f ms\n", t1.toc());

                    calibra_fixed = true;
                    lidarInfo1->T_0_i = pose_w0_w1;
                    return;
                }

            }
        }

    }


    LaserOdometry() : lidarSensor0(0), lidarSensor1(1) {

        need_calibra = nh.param<bool>("need_calibra", true);
        go_fusion = nh.param<bool>("go_fusion", false);

        save_map_resolution = nh.param<double>("save_map_resolution", 0.1);
        show_map_resolution = nh.param<double>("show_map_resolution", 0.2);
        file_save_path = nh.param<std::string>("file_save_path", "");
//        _mkdir(file_save_path + "test.txt");

//        f_lidar0_timecost.open(file_save_path + "lidar0_timecost.txt");
//        f_lidar1_timecost.open(file_save_path + "lidar1_timecost.txt");
//        cout << "#######################################" << endl;
//        cout << "f_lidar0_timecost path: " << file_save_path + "lidar0_timecost.txt" << endl;
//        cout << "f_lidar0_timecost isopen? " << f_lidar0_timecost.is_open() << endl;
//        cout << "f_lidar1_timecost path: " << file_save_path + "lidar1_timecost.txt" << endl;
//        cout << "f_lidar1_timecost isopen? " << f_lidar1_timecost.is_open() << endl;
//        cout << "#######################################" << endl;


        // LiDAR 0 init
        lidarSensor0.lidar_name = "LiDAR 0";
        lidarInfo0 = std::make_shared<LidarInfo>(0);
        lidarInfo0->status = lidarSensor0.status;
        lidarInfo0->keyframeVec = lidarSensor0.keyframeVec;
        lidarInfo0->frameChannel = lidarSensor0.frameChannel;

        // LiDAR 1 init
        lidarSensor1.lidar_name = "LiDAR 1";
        lidarInfo1 = std::make_shared<LidarInfo>(1);
        lidarInfo1->status = lidarSensor1.status;
        lidarInfo1->keyframeVec = lidarSensor1.keyframeVec;
        lidarInfo1->frameChannel = lidarSensor1.frameChannel;

        lidarInfo0->sub_laser_cloud = nh.subscribe<sensor_msgs::PointCloud2>("/left/velodyne_points_2", 100,
                                                                             &LidarMsgReader::pointCloudFullHandler,
                                                                             &lidarInfo0->reader);
        lidarInfo0->sub_laser_cloud_edge = nh.subscribe<sensor_msgs::PointCloud2>("/left/laser_cloud_edge", 100,
                                                                                  &LidarMsgReader::pointCloudEdgeHandler,
                                                                                  &lidarInfo0->reader);
        lidarInfo0->sub_laser_cloud_surf = nh.subscribe<sensor_msgs::PointCloud2>("/left/laser_cloud_surf", 100,
                                                                                  &LidarMsgReader::pointCloudSurfHandler,
                                                                                  &lidarInfo0->reader);
        lidarInfo0->sub_laser_cloud_less_edge = nh.subscribe<sensor_msgs::PointCloud2>("/left/laser_cloud_less_edge",
                                                                                       100,
                                                                                       &LidarMsgReader::pointCloudLessEdgeHandler,
                                                                                       &lidarInfo0->reader);
        lidarInfo0->sub_laser_cloud_less_surf = nh.subscribe<sensor_msgs::PointCloud2>("/left/laser_cloud_less_surf",
                                                                                       100,
                                                                                       &LidarMsgReader::pointCloudLessSurfHandler,
                                                                                       &lidarInfo0->reader);
        lidarInfo0->pub_odom_opti = nh.advertise<nav_msgs::Path>("/left/odom_opti", 100);
        lidarInfo0->pub_odom_mapping = nh.advertise<nav_msgs::Path>("/left/odom_mapping", 100);
        lidarInfo0->pub_map = nh.advertise<sensor_msgs::PointCloud2>("/left/global_map", 5);
        lidarInfo0->pub_pointcloud_raw = nh.advertise<sensor_msgs::PointCloud2>("/left/raw_points", 10);

        lidarInfo1->sub_laser_cloud = nh.subscribe<sensor_msgs::PointCloud2>("/right/velodyne_points_2", 100,
                                                                             &LidarMsgReader::pointCloudFullHandler,
                                                                             &lidarInfo1->reader);
        lidarInfo1->sub_laser_cloud_edge = nh.subscribe<sensor_msgs::PointCloud2>("/right/laser_cloud_edge", 100,
                                                                                  &LidarMsgReader::pointCloudEdgeHandler,
                                                                                  &lidarInfo1->reader);
        lidarInfo1->sub_laser_cloud_surf = nh.subscribe<sensor_msgs::PointCloud2>("/right/laser_cloud_surf", 100,
                                                                                  &LidarMsgReader::pointCloudSurfHandler,
                                                                                  &lidarInfo1->reader);
        lidarInfo1->sub_laser_cloud_less_edge = nh.subscribe<sensor_msgs::PointCloud2>("/right/laser_cloud_less_edge",
                                                                                       100,
                                                                                       &LidarMsgReader::pointCloudLessEdgeHandler,
                                                                                       &lidarInfo1->reader);
        lidarInfo1->sub_laser_cloud_less_surf = nh.subscribe<sensor_msgs::PointCloud2>("/right/laser_cloud_less_surf",
                                                                                       100,
                                                                                       &LidarMsgReader::pointCloudLessSurfHandler,
                                                                                       &lidarInfo1->reader);
        lidarInfo1->pub_odom_opti = nh.advertise<nav_msgs::Path>("/right/odom_opti", 100);
        lidarInfo1->pub_odom_mapping = nh.advertise<nav_msgs::Path>("/right/odom_mapping", 100);
        lidarInfo1->pub_map = nh.advertise<sensor_msgs::PointCloud2>("/right/global_map", 5);
        lidarInfo1->pub_pointcloud_raw = nh.advertise<sensor_msgs::PointCloud2>("/right/raw_points", 10);;

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

    bool need_calibra;
    bool go_fusion;

    bool calibra_simple = false;
    bool calibra_with_loop = false;
    bool calibra_fixed = false;

    double save_map_resolution;
    double show_map_resolution;
    std::string file_save_path;
    ofstream f_lidar0_timecost;
    ofstream f_lidar1_timecost;

};

int main(int argc, char **argv) {

    ros::init(argc, argv, "laserOdometry");

    LaserOdometry laserOdometry;

    std::thread laser_odometry_thread_0{&LaserOdometry::laser_odometry_base, &laserOdometry};
    std::thread laser_mapping_thread_0{&LaserOdometry::laser_mapping_base, &laserOdometry};
    std::thread backend_ba_optimization_0{&LidarSensor::BA_optimization, &laserOdometry.lidarSensor0};
    std::thread backend_loop_detector_0{&LidarSensor::loop_detect_thread, &laserOdometry.lidarSensor0};

//    std::thread laser_odometry_thread_1{&LaserOdometry::laser_odometry_sub, &laserOdometry};
//    std::thread laser_mapping_thread_1{&LaserOdometry::laser_mapping_sub, &laserOdometry};
//    std::thread backend_ba_optimization_1{&LidarSensor::BA_optimization, &laserOdometry.lidarSensor1};
//    std::thread backend_loop_detector_1{&LidarSensor::loop_detect_thread, &laserOdometry.lidarSensor1};

//    std::thread laser_odometry_thread_fusion{&LaserOdometry::laser_odometry_fusion, &laserOdometry};
//    std::thread laser_mapping_thread_fusion{&LaserOdometry::laser_mapping_fusion, &laserOdometry};
//    std::thread backend_ba_optimization_fusion{&LidarSensor::BA_optimization, &laserOdometry.lidarSensor0};
//    std::thread backend_loop_detector_fusion{&LidarSensor::loop_detect_thread, &laserOdometry.lidarSensor0};

//    std::thread laser_calibration_thread{&LaserOdometry::laser_calibration, &laserOdometry};
//    std::thread global_calibration_thread{&LaserOdometry::global_calibration, &laserOdometry};

    std::thread global_map_publisher{&LaserOdometry::pub_global_map, &laserOdometry};

    ros::spin();

}