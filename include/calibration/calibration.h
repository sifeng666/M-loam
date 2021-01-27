//
// Created by ziv on 2021/1/11.
//

#ifndef MLOAM_CALIBRATION_H
#define MLOAM_CALIBRATION_H

#include "keyframe/keyframe.h"
#include "camodocal/PlanarHandEyeCalibration.h"
//#include "camodocal/HandEyeCalibration.h"

struct PoseTimeStamp {
    ros::Time    timestamp;
    gtsam::Pose3 pose;
    PoseTimeStamp(ros::Time rt, gtsam::Pose3 pose_) : timestamp(std::move(rt)), pose(std::move(pose_)) {}
};

class HandEyeCalibrator {
public:
    static bool calibrate(KeyframeVec::Ptr keyframeVec_0, KeyframeVec::Ptr keyframeVec_i, gtsam::Pose3& T_0_i);
    static bool sync_timestamp(std::vector<PoseTimeStamp>& PTS_0, std::vector<PoseTimeStamp>& PTS_i);
};

#endif //MLOAM_CALIBRATION_H
