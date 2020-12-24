//
// Created by ziv on 2020/10/24.
//

#ifndef MLOAM_FRAME_H
#define MLOAM_FRAME_H

#include "helper.h"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/OrientedPlane3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>

using gtsam::symbol_shorthand::E; //  edge factor
using gtsam::symbol_shorthand::P; // plane factor
using gtsam::symbol_shorthand::X; // state

enum class FeatureType {
    Edge = 0,
    Surf = 1,
    Full = 2
};

class Keyframe {
public:
    using Ptr = boost::shared_ptr<Keyframe>;
    using PointCloudPtr = pcl::PointCloud<PointT>::Ptr;
public:
    int index;
    int valid_frames = 0;
    gtsam::Pose3 pose_world_curr;
    gtsam::Pose3 pose_last_curr;
    std::vector<gtsam::OrientedPlane3> planes;
    // feature point cloud
    pcl::PointCloud<PointT>::Ptr edgeFeatures;
    pcl::PointCloud<PointT>::Ptr surfFeatures;
    pcl::PointCloud<PointT>::Ptr raw;
public:
    Keyframe(int _index, PointCloudPtr EF, PointCloudPtr PF, PointCloudPtr RAW);
    void set_init(Keyframe::Ptr last_keyframe, gtsam::Pose3 pose_world_curr_);
    bool is_init() const;
    void add_frame();

};

struct KeyframeVec {
    using Ptr = boost::shared_ptr<KeyframeVec>;
    std::shared_mutex pose_mtx;
    std::vector<Keyframe::Ptr> keyframes;
};


#endif //MLOAM_FRAME_H
