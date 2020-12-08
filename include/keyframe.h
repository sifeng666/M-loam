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

class Keyframe {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    gtsam::Pose3 pose;
    int index;
    int frameCount;
    using Ptr = boost::shared_ptr<Keyframe>;
    using PointCloudPtr = pcl::PointCloud<PointT>::Ptr;

    // feature point cloud
    pcl::PointCloud<PointT>::Ptr edgeFeatures;
    pcl::PointCloud<PointT>::Ptr surfFeatures;
    // slice-contains all frames between this and next keyframe
    pcl::PointCloud<PointT>::Ptr edgeSlice;
    pcl::PointCloud<PointT>::Ptr surfSlice;

public:
    Keyframe(int _index, PointCloudPtr EF, PointCloudPtr PF);
    ~Keyframe();
    void normalizePose();
};



#endif //MLOAM_FRAME_H
