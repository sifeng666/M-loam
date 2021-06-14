//
// Created by ziv on 2020/12/16.
//

#ifndef MLOAM_LOOP_DETECTOR_H
#define MLOAM_LOOP_DETECTOR_H

#include "helper.h"
#include "tools/keyframe.hpp"
#include "tools/map_generator.hpp"
#include "tools/registration.hpp"

static int debug_count = 0;

namespace tools
{
    using FactorPtr = gtsam::NonlinearFactor::shared_ptr;//非线性优化因子的智能指针
//    using FactorPtr = gtsam::GaussianFactor::shared_ptr;
    //回环检测类
    class LoopDetector {
    public:
        ros::NodeHandle nh;

        int last_loop_found_index = 0;

        // loop hyper parameter
        double fitness_thres_; //判断是否配准成功的阈值，越小越好
        int loop_detect_distance_; //回环检测距离，小于这个值则有可能产生回环
        int loop_found_sleep_; //距离上次回环检测的间隔，比如10和2中找到了回环，10后面的10帧都跳过回环检测
        int loop_submap_length_;//规定子图的大小，用于跳过靠近的帧之间的回环检测
        //skip last是一个trick
        //就是30不跟10:30内的找回环，最近k个不找，否则10和9也有回环
        int loop_skip_last_;

        std::ofstream f_loop;

    public:
        LoopDetector(int i) {
            string file_save_path = nh.param<std::string>("file_save_path", "");
//            boost::filesystem::path save_path(file_save_path+"lidar"+to_string(i)+"f_loop.txt");
//            auto folder_path = save_path.parent_path();
//            if (!boost::filesystem::exists(folder_path)) {
//                boost::filesystem::create_directories(folder_path);
//            }
            // f_loop.open(file_save_path+"lidar"+to_string(i)+"f_loop.txt");
            // cout << "#######################################" << endl;
            // cout << "current_lidar: " << i << endl;
            // cout << "f_loop path: " << file_save_path+"lidar"+to_string(i)+"f_loop.txt" << endl;
            // cout << "f_loop isopen? " << f_loop.is_open() << endl;
            // cout << "#######################################" << endl;
            loop_detect_distance_ = nh.param<int>("loop_detect_distance", 15);
            loop_found_sleep_     = nh.param<int>("loop_found_sleep", 20);
            loop_submap_length_   = nh.param<int>("loop_submap_length", 10);
            loop_skip_last_       = nh.param<int>("loop_skip_last", loop_detect_distance_ * 2);
            fitness_thres_        = nh.param<double>("gicp_fitness_socre", 0.8);
        }

//    void submap_finetune(KeyframeVec::Ptr keyframeVec,
//                         Keyframe::Ptr latestKeyframe,
//                         std::vector<FactorPtr>& loopFactors);

        bool loop_detector(KeyframeVec::Ptr keyframeVec, //关键帧的vector
                           Keyframe::Ptr curr_keyframe, //当前关键帧
                           vector<FactorPtr>& loopFactors, //输出，找到的回环因子（可能不止一个）
                           int& last_found_index) //最后找到回环的index
        {
            int curr_index = curr_keyframe->index;  //当前的index
            if (curr_index < loop_skip_last_ + 1)  //帧数还没有到达找回环检测的数量
                return false;
            if (last_loop_found_index > 0 && curr_index <= last_loop_found_index + loop_found_sleep_) //当前帧与上一次找到回环的帧过于接近
                return false;

            size_t buffer_size = curr_index - loop_skip_last_;
            // <frameCount, distance>
            std::vector<pair<int, double>> candidates; //候选帧
            candidates.reserve(buffer_size); //重制长度

            // read lock
            std::vector<gtsam::Pose3> poseVec = keyframeVec->read_poses(0, buffer_size, true); //读取【0，buffer_size】里面的所有位姿
            gtsam::Pose3 curr_pose = keyframeVec->read_pose(curr_index, true); //读取curr_index对应的位姿

            //遍历范围内的所有帧，判断可能有与当前帧判断可以产生回环的，插入到候选帧中
            for (int i = 0; i < buffer_size; i++) {
                gtsam::Pose3 pose_between = curr_pose.between(poseVec[i]); //计算两帧位姿之间的变化量
//                double distance = pose_between.translation().norm();
                double distance = std::sqrt(pose_between.translation().x() * pose_between.translation().x() + pose_between.translation().y() * pose_between.translation().y());
                // too far
                if (distance > loop_detect_distance_)
                    continue;
                candidates.emplace_back(i, distance); //距离小于给定阈值，则插入到候选帧中
            }

            if (candidates.empty()) //候选帧为空，直接返回false
                return false;

            //按照距离从小到大排序
            std::sort(candidates.begin(), candidates.end(), [](const auto& p1, const auto& p2) {
                return p1.second < p2.second;
            });

            // select 2 closest key
            int success_loop_count = 0;
            //min的作用是当candidates矩阵为空时跳过for循环?否则也是只遍历一次，即只取最前面（距离最小）的帧
            for (int i = 0; i < min(1, int(candidates.size())); i++) {
                int closestKeyIdx = candidates[i].first; //候选帧的序号
                cout << "####################################" << endl;
                cout << "[loop] detecting: " << curr_index << " and " << closestKeyIdx << ", distance is " << candidates[i].second << endl;
                cout << "####################################" << endl;
                int submap_start = max(0, closestKeyIdx - loop_submap_length_); //跳过过于靠近当前帧的帧再开始匹配
                int submap_end   = min(closestKeyIdx + loop_submap_length_, curr_index - loop_skip_last_); //跳过过于靠近当前帧的帧或者是刚找到回环的再开始匹配
                //生成给定区间内所有原始点云集合成的submap
                auto submap_pcd = MapGenerator::generate_cloud(keyframeVec, submap_start, submap_end + 1, FeatureType::Full);
                //基于候选帧的位姿的刚体变换，将submap_pcd点云进行旋转+平移的刚体变换；三个参数分别为（输入点云，输出点云，变换矩阵）
                pcl::transformPointCloud(*submap_pcd, *submap_pcd, poseVec[closestKeyIdx].inverse().matrix());
                //当前帧的点云
                auto curr_pcd = keyframeVec->keyframes[curr_index]->raw->makeShared();

                //对两帧用于比较的点云降采样
                pcl::VoxelGrid<PointT> f;
                f.setLeafSize(0.2, 0.2, 0.2);
                f.setInputCloud(submap_pcd);
                f.filter(*submap_pcd);
                f.setInputCloud(curr_pcd);
                f.filter(*curr_pcd);

                //求解候选帧id对应的位姿和当前位姿之间的变换矩阵？
                Eigen::Matrix4d pose_closest_curr(poseVec[closestKeyIdx].between(curr_pose).matrix());
                pose_closest_curr(14) = 0.0; // planar assumption
                Eigen::Matrix4d pose_closest_curr_final;
                // fast gicp, refer to "Voxelized GICP for fast and accurate 3D point cloud registration, ICRA2021", this version is faster by using multiple CPU, but result is not as good as normal one
                //fast GICP配准，参数分别为（点云1，点云2，变换矩阵，Identity()?，对应点对的最大距离，配准成功阈值）；配准成功返回true
                bool ok = tools::FastGeneralizedRegistration(curr_pcd, submap_pcd, pose_closest_curr_final, pose_closest_curr, 2.0, fitness_thres_);
                // // gicp from PCL standard library, using one CPU, not that slow but resulting better
                // bool ok = tools::GeneralizedRegistration(curr_pcd, submap_pcd, pose_closest_curr_final, pose_closest_curr, 2.0, fitness_thres_);

                // // debug, save all probable loop pcds
                // {
                //     static string path0 = "/home/ziv/mloam/debug_loop/";
                //     auto path = path0 + to_string(debug_count) + "/";

                //     boost::filesystem::path save_path(path + "1.txt");
                //     auto folder_path = save_path.parent_path();
                //     if (!boost::filesystem::exists(folder_path)) {
                //         boost::filesystem::create_directories(folder_path);
                //     }

                //     pcl::io::savePCDFileASCII(path + "submap.pcd", *submap_pcd);
                //     pcl::io::savePCDFileASCII(path + "curr.pcd", *curr_pcd);
                //     ofstream f(path + "guess.txt");
                //     f << init_guess << endl;
                //     f << final << endl;
                //     if (!ok) {
                //         f << "false" << endl;
                //     } else {
                //         f << "true" << endl;
                //     }
                //     f.close();
                //     debug_count++;
                // }

                 if (!ok) { //没有配准成功，即没有找到回环
                     continue;
                 }
                //信息矩阵
                auto information = tools::GetInformationMatrixFromPointClouds(curr_pcd, submap_pcd, 2.0, pose_closest_curr_final);
                // auto loop_model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-2, 1e-2, 1e-1).finished());
                // cout << "loop information:\n" << information << endl;

                // f_loop << closestKeyIdx << " " << curr_index << endl;
                // f_loop << pose_closest_curr_final << endl;
                // f_loop << information << endl;
                //找到回环，则插入到回环vector loopFactors中
                loopFactors.emplace_back(new gtsam::BetweenFactor<gtsam::Pose3>(
                        X(closestKeyIdx),
                        X(curr_index),
                        gtsam::Pose3(pose_closest_curr_final),
//                        loop_model));
                        gtsam::noiseModel::Gaussian::Information(information)));

                printf("add loop factor: [%d] and [%d]\n", curr_index, closestKeyIdx);
                success_loop_count++;
            }
            //找到回环，更新回环相关的全局参数
            if (success_loop_count > 0) {
                last_loop_found_index = curr_index;
                last_found_index = last_loop_found_index;
                return true;
            }
            return false;
        }
    };

//void LoopDetector::submap_finetune(KeyframeVec::Ptr keyframeVec, Keyframe::Ptr latestKeyframe,
//                                   vector<FactorPtr> &loopFactors) {
//
//    int latest_index = latestKeyframe->index;
//
//    const int LEN = 6;
//
//    if (latest_index < LEN)
//        return;
//
//    gtsam::Pose3 submap_pose = keyframeVec->read_pose(latest_index - LEN);
//    gtsam::Pose3 latest_pose = keyframeVec->read_pose(latest_index);
//
//    auto submap = MapGenerator::generate_cloud(keyframeVec, latest_index - LEN, latest_index, FeatureType::Full);
//    pcl::transformPointCloud(*submap, *submap, submap_pose.inverse().matrix());
//    auto latest = MapGenerator::generate_cloud(keyframeVec, latest_index, latest_index + 1, FeatureType::Full);
//    pcl::transformPointCloud(*latest, *latest, latest_pose.inverse().matrix());
//
//    gtsam::Pose3 pose_crop_latest_opti;
//
//    auto pose_crop_latest_coarse = submap_pose.between(latest_pose);
//    bool can_match = gicp_matching(submap, latest, pose_crop_latest_coarse, pose_crop_latest_opti);
//
//    if (!can_match) {
//        return;
//    }
//
//    loopFactors.emplace_back(new gtsam::BetweenFactor<gtsam::Pose3>(X(latest_index - LEN), X(latest_index), pose_crop_latest_opti, submap_noise_model));
//
//    std::cout << "add submap_finetune factor!" << std::endl;
//
//}
}




#endif //MLOAM_LOOP_DETECTOR_H
