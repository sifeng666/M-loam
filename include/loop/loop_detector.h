//
// Created by ziv on 2020/12/16.
//

#ifndef MLOAM_LOOP_DETECTOR_H
#define MLOAM_LOOP_DETECTOR_H

#include "helper.h"
#include "keyframe/keyframe.h"
#include "map_generator/map_generator.h"
#include <fast_gicp/gicp/fast_gicp.hpp>

using FactorPtr = gtsam::NonlinearFactor::shared_ptr;

static gtsam::SharedNoiseModel loop_noise_model   = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 5e-3, 5e-3, 5e-2).finished());
static gtsam::SharedNoiseModel submap_noise_model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 3e-2, 3e-2, 2e-1).finished());

class LoopDetector {
public:
    ros::NodeHandle nh;
    int last_loop_found_index = 0;

    int LOOP_KEYFRAME_CROP_LEN;
    int LOOP_KEYFRAME_SKIP;
    int LOOP_KEYFRAME_COOLDOWN;
    int LOOP_CLOSE_DISTANCE;
    static double FITNESS_SCORE;
public:
    explicit LoopDetector();
    void loop_detector(KeyframeVec::Ptr keyframeVec, Keyframe::Ptr latestKeyframe, std::vector<FactorPtr>& loopFactors);
    void submap_finetune(KeyframeVec::Ptr keyframeVec, Keyframe::Ptr latestKeyframe, std::vector<FactorPtr>& loopFactors);
    static bool gicp_matching(pcl::PointCloud<PointT>::Ptr cloud_to, pcl::PointCloud<PointT>::Ptr cloud_from, const gtsam::Pose3& pose_guess, gtsam::Pose3& pose);
};


#endif //MLOAM_LOOP_DETECTOR_H
