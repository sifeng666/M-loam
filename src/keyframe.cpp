//
// Created by ziv on 2020/12/7.
//

#include "keyframe.h"

Keyframe::Keyframe(int _index, PointCloudPtr EF, PointCloudPtr PF) :
        index(_index), edgeFeatures(EF), surfFeatures(PF) {
    pose = gtsam::Pose3::identity();
    inited = false;
}

Keyframe::~Keyframe() {}

void Keyframe::set_init() {
    inited = true;
}
