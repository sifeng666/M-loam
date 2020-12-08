//
// Created by ziv on 2020/12/7.
//

#include "keyframe.h"

Keyframe::Keyframe(int _index, PointCloudPtr EF, PointCloudPtr PF) :
        index(_index), edgeFeatures(EF), surfFeatures(PF) {
    // init
    pose = gtsam::Pose3::identity();
    frameCount = 0;

    edgeSlice = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
    surfSlice = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());

}


Keyframe::~Keyframe() {}


void Keyframe::normalizePose() {
    pose = gtsam::Pose3(pose.rotation().normalized(), pose.translation());
}