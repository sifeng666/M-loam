//
// Created by ziv on 2020/12/16.
//

#include "loop_detector.h"

void LoopDetector::loop_detector(KeyframeVec::Ptr keyframeVec, Keyframe::Ptr latestKeyframe, std::vector<LoopFactor>& loopFactors) {

    int latest_index = latestKeyframe->index;

    if (latest_index < LOOP_LATEST_KEYFRAME_SKIP + 1)
        return;

    if (last_loop_found_index > 0 && latest_index <= last_loop_found_index + LOOP_COOLDOWN_KEYFRAME_COUNT)
        return;

    size_t buffer_size = latest_index - LOOP_LATEST_KEYFRAME_SKIP;
    // <frameCount, distance>
    std::vector<pair<int, int>> candidates;
    candidates.reserve(buffer_size);

    std::vector<gtsam::Pose3> poseVec;
    poseVec.reserve(buffer_size);
    gtsam::Pose3 latest_pose;
    // shared_lock
    {
        keyframeVec->pose_mtx.lock_shared();
        latest_pose = latestKeyframe->pose_world_curr;
        for (size_t i = 0; i < buffer_size; i++) {
            poseVec.push_back(keyframeVec->keyframes[i]->pose_world_curr);
        }
        keyframeVec->pose_mtx.unlock_shared();
    }

    for (int i = 0; i < buffer_size; i++) {
        gtsam::Pose3 pose_between = latest_pose.between(poseVec[i]);
        double distance = pose_between.translation().norm();
        // too far
        if (distance > LOOP_CLOSE_DISTANCE)
            continue;
        candidates.emplace_back(i, distance);
    }

    if (candidates.empty())
        return;

    std::sort(candidates.begin(), candidates.end(), [](const auto& p1, const auto& p2) {
        return p1.second < p2.second;
    });

    // select 2 closest key
    int success_loop_count = 0;
    for (int i = 0; i < min(2, int(candidates.size())); i++) {
        int closestKeyIdx = candidates[i].first;
        int start_crop = max(0, closestKeyIdx - LOOP_KEYFRAME_CROP_LEN);
        int end_crop   = min(latest_index - LOOP_LATEST_KEYFRAME_SKIP, closestKeyIdx + LOOP_KEYFRAME_CROP_LEN);

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

        loopFactors.emplace_back(new gtsam::BetweenFactor<gtsam::Pose3>(X(closestKeyIdx), X(latest_index), pose_crop_latest_opti, loop_closure_gaussian_model));
        cout << "find loop: [" << latest_index << "] and [" << closestKeyIdx << "]\n";
        cout << "pose coarse:     \n" << pose_crop_latest_coarse.matrix() << endl;
        cout << "pose after gicp: \n" << pose_crop_latest_opti.matrix() << endl;
//        pcl::io::savePCDFileASCII(filepath + "loop/crop.pcd", *crop);
//        pcl::io::savePCDFileASCII(filepath + "loop/latest.pcd", *latest);
        success_loop_count++;
    }

    if (success_loop_count > 0) {
        last_loop_found_index = latest_index;
    }

}

bool LoopDetector::gicp_matching(pcl::PointCloud<PointT>::Ptr cloud_to, pcl::PointCloud<PointT>::Ptr cloud_from, const gtsam::Pose3& pose_guess, gtsam::Pose3& pose) {

    pclomp::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
    gicp.setTransformationEpsilon(1e-6);
    gicp.setEuclideanFitnessEpsilon(1e-6);
    gicp.setMaximumIterations(64);
    gicp.setUseReciprocalCorrespondences(false);
    gicp.setCorrespondenceRandomness(20);
    gicp.setMaximumOptimizerIterations(20);

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

    if (gicp.getFitnessScore() > 0.8) {
        return false;
    }
    pose = gtsam::Pose3(gicp.getFinalTransformation().cast<double>().matrix());
    pose = gtsam::Pose3(pose.rotation().normalized(), pose.translation());
    return true;

}

