//
// Created by ziv on 2020/12/16.
//

#ifndef MLOAM_LOOP_DETECTOR_H
#define MLOAM_LOOP_DETECTOR_H

#include "helper.h"
#include "keyframe.h"
#include "map_generator.h"
#include <pclomp/gicp_omp.h>

using LoopFactor = gtsam::NonlinearFactor::shared_ptr;

const int LOOP_KEYFRAME_CROP_LEN = 10;
const int LOOP_LATEST_KEYFRAME_SKIP = 50;
const int LOOP_COOLDOWN_KEYFRAME_COUNT = 15;
const int LOOP_CLOSE_DISTANCE = 15;
extern const std::string filepath;

static gtsam::SharedNoiseModel loop_closure_gaussian_model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-2, 1e-2, 1e-1).finished());

class LoopDetector {
public:
    int last_loop_found_index = 0;
public:
    void loop_detector(KeyframeVec::Ptr keyframeVec, Keyframe::Ptr latestKeyframe, std::vector<LoopFactor>& loopFactors);
    bool gicp_matching(pcl::PointCloud<PointT>::Ptr cloud_to, pcl::PointCloud<PointT>::Ptr cloud_from, const gtsam::Pose3& pose_guess, gtsam::Pose3& pose);
};


#endif //MLOAM_LOOP_DETECTOR_H
