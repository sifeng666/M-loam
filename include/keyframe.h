//
// Created by ziv on 2020/10/24.
//

#ifndef MLOAM_FRAME_H
#define MLOAM_FRAME_H

#include "helper.h"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
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

struct EdgeFeatures {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d curr_point;
    Eigen::Vector3d point_a;
    Eigen::Vector3d point_b;

    EdgeFeatures(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3)
            : curr_point(std::move(p1)), point_a(std::move(p2)), point_b(std::move(p3)) {

    }
};

struct PlaneFeatures {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d curr_point;
    Eigen::Vector3d norm;
    double negative_OA_dot_norm;

    PlaneFeatures(Eigen::Vector3d p1, Eigen::Vector3d norm_, double nor)
            : curr_point(std::move(p1)), norm(std::move(norm_)), negative_OA_dot_norm(nor) {

    }
};

enum class FeatureType {
    Edge = 0,
    Surf = 1,
    Full = 2
};

//struct Frame {
//    using Ptr = boost::shared_ptr<Frame>;
//    gtsam::Pose3 pose;
//    pcl::PointCloud<PointT>::Ptr edgeFeatures;
//    pcl::PointCloud<PointT>::Ptr surfFeatures;
//    Frame(const gtsam::Pose3& pose_, pcl::PointCloud<PointT>::Ptr edgeFeatures_, pcl::PointCloud<PointT>::Ptr surfFeatures_) :
//            pose(pose_), edgeFeatures(edgeFeatures_), surfFeatures(surfFeatures_) {}
//};

class Keyframe {
public:
    using Ptr = boost::shared_ptr<Keyframe>;
    using PointCloudPtr = pcl::PointCloud<PointT>::Ptr;
public:
    int index;
//    mutable std::shared_mutex sharedMutex;
    gtsam::Pose3 pose;
    // feature point cloud
    pcl::PointCloud<PointT>::Ptr edgeFeatures;
    pcl::PointCloud<PointT>::Ptr surfFeatures;
    pcl::PointCloud<PointT>::Ptr raw;
//    // sub frames from current to next keyframe. [current, next)
//    std::vector<Frame::Ptr> sub_frames;
private:
    bool init = false;
public:
    Keyframe(int _index, PointCloudPtr EF, PointCloudPtr PF, PointCloudPtr RAW);
    ~Keyframe();
    void set_init(gtsam::Pose3 pose_);
    bool is_init() const;
//    pcl::PointCloud<PointT>::Ptr generate_sub_map(FeatureType featureType) const;
//    void insert(const gtsam::Pose3& pose_, PointCloudPtr EF, PointCloudPtr PF);
//    gtsam::Pose3 pose() const;
//    gtsam::Pose3& pose();
//    void optimize(const gtsam::Pose3& next_pose);

};



#endif //MLOAM_FRAME_H
