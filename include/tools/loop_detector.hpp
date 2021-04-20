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
    using FactorPtr = gtsam::NonlinearFactor::shared_ptr;
//    using FactorPtr = gtsam::GaussianFactor::shared_ptr;

    class LoopDetector {
    public:
        ros::NodeHandle nh;

        int last_loop_found_index = 0;

        // loop hyper parameter
        double fitness_thres_;
        int loop_detect_distance_;
        int loop_found_sleep_;
        int loop_submap_length_;
        int loop_skip_last_;

    public:
        LoopDetector() {
            loop_detect_distance_ = nh.param<int>("loop_detect_distance", 15);
            loop_found_sleep_     = nh.param<int>("loop_found_sleep", 20);
            loop_submap_length_   = nh.param<int>("loop_submap_length", 10);
            loop_skip_last_       = nh.param<int>("loop_skip_last", loop_detect_distance_ * 2);
            fitness_thres_        = nh.param<double>("gcip_fitness_socre", 0.8);
        }

//    void submap_finetune(KeyframeVec::Ptr keyframeVec,
//                         Keyframe::Ptr latestKeyframe,
//                         std::vector<FactorPtr>& loopFactors);

        bool loop_detector(KeyframeVec::Ptr keyframeVec,
                           Keyframe::Ptr curr_keyframe,
                           vector<FactorPtr>& loopFactors,
                           int& last_found_index)
        {
            int curr_index = curr_keyframe->index;
            if (curr_index < loop_skip_last_ + 1)
                return false;
            if (last_loop_found_index > 0 && curr_index <= last_loop_found_index + loop_found_sleep_)
                return false;

            size_t buffer_size = curr_index - loop_skip_last_;
            // <frameCount, distance>
            std::vector<pair<int, int>> candidates;
            candidates.reserve(buffer_size);

            // read lock
            std::vector<gtsam::Pose3> poseVec = keyframeVec->read_poses(0, buffer_size, true);
            gtsam::Pose3 curr_pose = keyframeVec->read_pose(curr_index, true);

            for (int i = 0; i < buffer_size; i++) {
                gtsam::Pose3 pose_between = curr_pose.between(poseVec[i]);
                double distance = pose_between.translation().norm();
                // too far
                if (distance > loop_detect_distance_)
                    continue;
                candidates.emplace_back(i, distance);
            }

            if (candidates.empty())
                return false;

            std::sort(candidates.begin(), candidates.end(), [](const auto& p1, const auto& p2) {
                return p1.second < p2.second;
            });

            // select 2 closest key
            int success_loop_count = 0;
            for (int i = 0; i < min(2, int(candidates.size())); i++) {
                int closestKeyIdx = candidates[i].first;
                int submap_start = max(0, closestKeyIdx - loop_submap_length_);
                int submap_end   = min(closestKeyIdx + loop_submap_length_, curr_index - loop_skip_last_);

                auto submap_pcd = MapGenerator::generate_cloud(keyframeVec, submap_start, submap_end + 1, FeatureType::Full);
                pcl::transformPointCloud(*submap_pcd, *submap_pcd, poseVec[closestKeyIdx].inverse().matrix());
                auto curr_pcd = keyframeVec->keyframes[curr_index]->raw;

                Eigen::Matrix4d init_guess(poseVec[closestKeyIdx].between(curr_pose).matrix());
                Eigen::Matrix4d final;
                bool ok = tools::FastGeneralizedRegistration(curr_pcd, submap_pcd,final, init_guess, 2.0, fitness_thres_);

//                {
//                    static string path0 = "/home/ziv/mloam/debug_loop/";
//                    auto path = path0 + to_string(debug_count) + "/";
//
//                    boost::filesystem::path save_path(path + "1.txt");
//                    auto folder_path = save_path.parent_path();
//                    if (!boost::filesystem::exists(folder_path)) {
//                        boost::filesystem::create_directories(folder_path);
//                    }
//
//                    pcl::io::savePCDFileASCII(path + "submap.pcd", *submap_pcd);
//                    pcl::io::savePCDFileASCII(path + "curr.pcd", *curr_pcd);
//                    ofstream f(path + "guess.txt");
//                    f << init_guess << endl;
//                    f << final << endl;
//                    if (!ok) {
//                        f << "false" << endl;
//                    } else {
//                        f << "true" << endl;
//                    }
//                    f.close();
//                    debug_count++;
//                }

                if (!ok) {
                    continue;
                }

                auto information = tools::GetInformationMatrixFromPointClouds(curr_pcd, submap_pcd, 2.0, final);
                auto loop_model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-2, 1e-2, 1e-1).finished());
                cout << "loop information:\n" << information << endl;
                loopFactors.emplace_back(new gtsam::BetweenFactor<gtsam::Pose3>(
                        X(closestKeyIdx),
                        X(curr_index),
                        gtsam::Pose3(final),
                        loop_model));
//                        gtsam::noiseModel::Diagonal::Information(information)));

                printf("add loop factor: [%d] and [%d]\n", curr_index, closestKeyIdx);
                success_loop_count++;
            }

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
