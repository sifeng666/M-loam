//
// Created by ziv on 2020/12/7.
//

#include "keyframe.h"

Keyframe::Keyframe(int index_, PointCloudPtr EF, PointCloudPtr PF, PointCloudPtr RAW) :
    index(index_), edgeFeatures(EF), surfFeatures(PF), raw(RAW) {
    pose_world_curr = gtsam::Pose3();
    pose_last_curr = gtsam::Pose3();
}

void Keyframe::set_init(Keyframe::Ptr last_keyframe, gtsam::Pose3 pose_world_curr_) {
    pose_world_curr = pose_world_curr_;
    pose_last_curr = last_keyframe->pose_world_curr.between(pose_world_curr);
    add_frame();
}

bool Keyframe::is_init() const {
    return valid_frames > 0;
}

void Keyframe::add_frame() {
    ++valid_frames;
}

//pcl::PointCloud<PointT>::Ptr Keyframe::generate_sub_map(FeatureType featureType) const {
//
//    std::shared_lock<std::shared_mutex> lck(sharedMutex);
//
//    pcl::PointCloud<PointT>::Ptr cloud_out(new pcl::PointCloud<PointT>());
//
//    Eigen::Matrix4f pose_;
//    PointT dst_pt;
//
//    for (const auto& frame : sub_frames) {
//        pose_ = frame->pose.matrix().cast<float>();
//        if (featureType == FeatureType::Edge || featureType == FeatureType::Full) {
//            for(const auto& src_pt : frame->edgeFeatures->points) {
//                dst_pt.getVector4fMap() = pose_ * src_pt.getVector4fMap();
//                dst_pt.intensity = src_pt.intensity;
//                cloud_out->push_back(dst_pt);
//            }
//        }
//        if (featureType == FeatureType::Surf || featureType == FeatureType::Full) {
//            for(const auto& src_pt : frame->surfFeatures->points) {
//                dst_pt.getVector4fMap() = pose_ * src_pt.getVector4fMap();
//                dst_pt.intensity = src_pt.intensity;
//                cloud_out->push_back(dst_pt);
//            }
//        }
//    }
//
//    return cloud_out;
//}
//
//void Keyframe::insert(const gtsam::Pose3& pose_, PointCloudPtr EF, PointCloudPtr PF) {
//    std::unique_lock<std::shared_mutex> lck(sharedMutex);
//    sub_frames.push_back(boost::make_shared<Frame>(pose_, EF, PF));
//}

//gtsam::Pose3 Keyframe::pose() const {
//    if (!is_init()) {
//        throw "keyframe not inited!";
//    }
//    std::shared_lock<std::shared_mutex> lck(sharedMutex);
//    return sub_frames.front()->pose;
//}
//
//gtsam::Pose3& Keyframe::pose() {
//    if (!is_init()) {
//        throw "keyframe not inited!";
//    }
//    std::shared_lock<std::shared_mutex> lck(sharedMutex);
//    return sub_frames.front()->pose;
//}

//Eigen::Vector3d linear_interpolation(double t, Eigen::Vector3d start, Eigen::Vector3d end) {
//    end[0] = start[0] * (1 - t) + end[0] * t;
//    end[1] = start[1] * (1 - t) + end[1] * t;
//    end[2] = start[2] * (1 - t) + end[2] * t;
//    return end;
//}
//
//void Keyframe::optimize(const gtsam::Pose3& next_pose) {
//    std::unique_lock<std::shared_mutex> lck(sharedMutex);
//    auto this_pose = sub_frames.front()->pose;
//    int size = sub_frames.size();
//
//    Eigen::Quaterniond this_q(this_pose.rotation().matrix());
//    Eigen::Quaterniond next_q(next_pose.rotation().matrix());
//    Eigen::Vector3d this_t(this_pose.translation());
//    Eigen::Vector3d next_t(next_pose.translation());
//
//    for (int i = 1; i < size; i++) {
//        double t = double(i) / size;
//        Eigen::Quaterniond q(this_q.slerp(t, next_q));
//        Eigen::Vector3d trans(linear_interpolation(t, this_t, next_t));
//        sub_frames[i]->pose = gtsam::Pose3(gtsam::Rot3(q), trans);
//    }
//}


std::vector<gtsam::Pose3> KeyframeVec::read_poses(size_t begin, size_t end) const{

    if (begin >= end || end > keyframes.size()) {
        std::cerr << "read_poses invalid range" << std::endl;
    }

    std::vector<gtsam::Pose3> poseVec;
    poseVec.reserve(end - begin);

    std::shared_lock<std::shared_mutex> sl(pose_mtx);
    for (size_t i = begin; i < end; i++) {
        poseVec.emplace_back(keyframes[i]->pose_world_curr);
    }

    return poseVec;
}
