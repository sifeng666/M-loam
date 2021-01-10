//
// Created by ziv on 2020/12/7.
//

#include "keyframe.h"

Keyframe::Keyframe(int index_, const ros::Time& time, PointCloudPtr EF, PointCloudPtr PF, PointCloudPtr RAW) :
    index(index_), cloud_in_time(time), edgeFeatures(EF), surfFeatures(PF), raw(RAW), planes(new pcl::PointCloud<PointT>()) {
}

void Keyframe::set_init(gtsam::Pose3 pose_world_curr_) {
    pose_world_curr = pose_world_curr_;
    add_frame();
}

bool Keyframe::is_init() const {
    return valid_frames > 0;
}

void Keyframe::add_frame() {
    ++valid_frames;
}

std::vector<gtsam::Pose3> KeyframeVec::read_poses(size_t begin, size_t end) const {

    if (begin >= end || end > keyframes.size()) {
        std::cerr << "read_poses invalid range" << std::endl;
    }

    if (!keyframes[end - 1]->is_init()) {
        end--;
    }

    std::vector<gtsam::Pose3> poseVec;
    poseVec.reserve(end - begin);

    std::shared_lock<std::shared_mutex> sl(pose_mtx);
    for (size_t i = begin; i < end; i++) {
        poseVec.emplace_back(keyframes[i]->pose_world_curr);
    }

    return poseVec;
}

gtsam::Pose3 KeyframeVec::read_pose(size_t index) const {
    if (index > keyframes.size()) {
        std::cerr << "read_pose invalid range" << std::endl;
    }
    if(!keyframes[index]->is_init()) {
        std::cerr << "pose not init!" << std::endl;
        return gtsam::Pose3();
    }

    std::shared_lock<std::shared_mutex> sl(pose_mtx);
    return keyframes[index]->pose_world_curr;
}
