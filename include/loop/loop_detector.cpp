//
// Created by ziv on 2020/12/16.
//

#include "loop_detector.h"

double LoopDetector::FITNESS_SCORE;

bool LoopDetector::loop_detector(KeyframeVec::Ptr keyframeVec, Keyframe::Ptr latestKeyframe,
                                 std::vector<FactorPtr>& loopFactors, int& last_found_index) {

    int latest_index = latestKeyframe->index;
    if (latest_index < LOOP_KEYFRAME_SKIP + 1)
        return false;
    if (last_loop_found_index > 0 && latest_index <= last_loop_found_index + LOOP_KEYFRAME_COOLDOWN)
        return false;

    size_t buffer_size = latest_index - LOOP_KEYFRAME_SKIP;
    // <frameCount, distance>
    std::vector<pair<int, int>> candidates;
    candidates.reserve(buffer_size);

    // read lock
    std::vector<gtsam::Pose3> poseVec = keyframeVec->read_poses(0, buffer_size);
    gtsam::Pose3 latest_pose = keyframeVec->read_pose(latest_index);

    for (int i = 0; i < buffer_size; i++) {
        gtsam::Pose3 pose_between = latest_pose.between(poseVec[i]);
        double distance = pose_between.translation().norm();
        // too far
        if (distance > LOOP_CLOSE_DISTANCE)
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
        int start_crop = max(0, closestKeyIdx - LOOP_KEYFRAME_CROP_LEN);
        int end_crop   = min(latest_index - LOOP_KEYFRAME_SKIP, closestKeyIdx + LOOP_KEYFRAME_CROP_LEN);

        // crop submap to closestKeyIdx's pose frame
        auto crop = MapGenerator::generate_cloud(keyframeVec, start_crop, end_crop, FeatureType::Full);
        pcl::transformPointCloud(*crop, *crop, poseVec[closestKeyIdx].inverse().matrix());
        auto latest = MapGenerator::generate_cloud(keyframeVec, latest_index, latest_index + 1, FeatureType::Full);
        pcl::transformPointCloud(*latest, *latest, latest_pose.inverse().matrix());

        gtsam::Pose3 pose_crop_latest_opti;

        auto pose_crop_latest_coarse = poseVec[closestKeyIdx].between(latest_pose);
        bool can_match = gicp_matching(crop, latest, pose_crop_latest_coarse, pose_crop_latest_opti);

        if (!can_match) {
            continue;
        }

        loopFactors.emplace_back(new gtsam::BetweenFactor<gtsam::Pose3>(X(closestKeyIdx), X(latest_index), pose_crop_latest_opti, loop_noise_model));
        cout << "find loop: [" << latest_index << "] and [" << closestKeyIdx << "]\n";
        cout << "pose coarse:     \n" << pose_crop_latest_coarse.matrix() << endl;
        cout << "pose after gicp: \n" << pose_crop_latest_opti.matrix() << endl;
        success_loop_count++; 
    }

    if (success_loop_count > 0) {
        last_loop_found_index = latest_index;
        last_found_index = last_loop_found_index;
        return true;
    }
    return false;

}

bool LoopDetector::gicp_matching(pcl::PointCloud<PointT>::Ptr cloud_to, pcl::PointCloud<PointT>::Ptr cloud_from, const gtsam::Pose3& pose_guess, gtsam::Pose3& pose) {

    Timer t_gicp_matching("gicp_matching");
    fast_gicp::FastGICP<PointT, PointT> gicp;
    gicp.setNumThreads(0);
    gicp.setTransformationEpsilon(0.1);
    gicp.setMaximumIterations(64);
    gicp.setMaxCorrespondenceDistance(2.0);
    gicp.setCorrespondenceRandomness(20);

    // Align clouds
    gicp.setInputSource(cloud_from);
    gicp.setInputTarget(cloud_to);

    pcl::PointCloud<PointT>::Ptr unused_result(new pcl::PointCloud<PointT>());
    gicp.align(*unused_result, pose_guess.matrix().cast<float>());

    if (!gicp.hasConverged()) {
        cout << "gicp fail, not converged" << endl;
        return false;
    }
    cout << "fitness score: " << gicp.getFitnessScore() << endl;

    if (gicp.getFitnessScore() > FITNESS_SCORE) {
        return false;
    }
    Eigen::Matrix4d result(gicp.getFinalTransformation().cast<double>());
    Eigen::Quaterniond q(result.block<3, 3>(0, 0));
    q.normalize();
    pose = gtsam::Pose3(gtsam::Rot3(q), result.block<3, 1>(0, 3));
    t_gicp_matching.count();
    return true;

}

void LoopDetector::submap_finetune(KeyframeVec::Ptr keyframeVec, Keyframe::Ptr latestKeyframe,
                                   vector<FactorPtr> &loopFactors) {

    int latest_index = latestKeyframe->index;

    const int LEN = 6;

    if (latest_index < LEN)
        return;

    gtsam::Pose3 submap_pose = keyframeVec->read_pose(latest_index - LEN);
    gtsam::Pose3 latest_pose = keyframeVec->read_pose(latest_index);

    auto submap = MapGenerator::generate_cloud(keyframeVec, latest_index - LEN, latest_index, FeatureType::Full);
    pcl::transformPointCloud(*submap, *submap, submap_pose.inverse().matrix());
    auto latest = MapGenerator::generate_cloud(keyframeVec, latest_index, latest_index + 1, FeatureType::Full);
    pcl::transformPointCloud(*latest, *latest, latest_pose.inverse().matrix());

    gtsam::Pose3 pose_crop_latest_opti;

    auto pose_crop_latest_coarse = submap_pose.between(latest_pose);
    bool can_match = gicp_matching(submap, latest, pose_crop_latest_coarse, pose_crop_latest_opti);

    if (!can_match) {
        return;
    }

    loopFactors.emplace_back(new gtsam::BetweenFactor<gtsam::Pose3>(X(latest_index - LEN), X(latest_index), pose_crop_latest_opti, submap_noise_model));

    std::cout << "add submap_finetune factor!" << std::endl;

}

LoopDetector::LoopDetector() {
    LOOP_KEYFRAME_CROP_LEN = nh.param<int>("loop_keyframe_crop_len", 10);
    LOOP_KEYFRAME_SKIP     = nh.param<int>("loop_keyframe_skip", 50);
    LOOP_KEYFRAME_COOLDOWN = nh.param<int>("loop_cooldown", 20);
    LOOP_CLOSE_DISTANCE    = nh.param<int>("loop_close_distance", 15);
    FITNESS_SCORE          = nh.param<double>("gcip_fitness_socre", 0.8);
}

