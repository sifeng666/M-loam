//
// Created by ziv on 2020/11/22.
//

#ifndef MLOAM_LOOP_DETECTOR_H
#define MLOAM_LOOP_DETECTOR_H

#include "frame.h"
#include "helper.h"


class Loop {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Ptr = boost::shared_ptr<Loop>;
    Loop(const Frame::Ptr& k1, const Frame::Ptr& k2, const Eigen::Isometry3d& _poseBetween) :
        keyframe1(k1), keyframe2(k2), poseBetween(_poseBetween) {
        // init
    }
public:
    Frame::Ptr keyframe1;
    Frame::Ptr keyframe2;
    Eigen::Isometry3d poseBetween;
};

class LoopDetector {
public:
    LoopDetector() {

    }

    std::vector<Loop::Ptr> detect(
            const std::unordered_map<int, Frame::Ptr>& frameMap,
            const std::vector<int>& keyframeVec,
            int currKeyFrame) {

        std::vector<Loop::Ptr> detected_loops;

        for ()

    }
};


#endif //MLOAM_LOOP_DETECTOR_H
