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
    Full = 2,
    Plane = 3
};

struct Plane {
    using Ptr = boost::shared_ptr<Plane>;
    pcl::PointCloud<PointT>::Ptr points;
    Plane() : points(new pcl::PointCloud<PointT>()) {}
};

class Keyframe {
public:
    using Ptr = boost::shared_ptr<Keyframe>;
    using PointCloudPtr = pcl::PointCloud<PointT>::Ptr;
public:
    int index;
    bool fixed = false;
    int valid_frames = 0;
    ros::Time cloud_in_time;
    gtsam::Pose3 pose_world_curr;
    // feature point cloud
    pcl::PointCloud<PointT>::Ptr edgeFeatures;
    pcl::PointCloud<PointT>::Ptr surfFeatures;
    pcl::PointCloud<PointT>::Ptr raw;
    pcl::PointCloud<PointT>::Ptr planes;
public:
    Keyframe(int _index, const ros::Time& time, PointCloudPtr EF, PointCloudPtr PF, PointCloudPtr RAW);
    void set_fixed(gtsam::Pose3 pose_world_curr_ = gtsam::Pose3());
    void set_init(gtsam::Pose3  pose_world_curr_ = gtsam::Pose3());

    inline void fix()            { fixed = true; };
    inline void add_frame()      { ++valid_frames; };
    inline bool is_fixed() const { return fixed; };
    inline bool is_init()  const { return valid_frames > 0; };

};

class KeyframeVec {
public:
    using Ptr = boost::shared_ptr<KeyframeVec>;
public:
    mutable std::shared_mutex pose_mtx;
    std::vector<Keyframe::Ptr> keyframes;
public:
    // read poses that are inited or fixed
    std::vector<gtsam::Pose3> read_poses(size_t begin, size_t end, bool need_fixed = false) const;
    // read pose that is inited, not require fixed
    gtsam::Pose3 read_pose(size_t index) const;

    inline size_t size() const { return keyframes.size(); };
    inline Keyframe::Ptr& operator[](size_t index) { return this->keyframes.at(index); };
    inline const Keyframe::Ptr& operator[](size_t index) const { return this->keyframes.at(index); };
};


#endif //MLOAM_FRAME_H
