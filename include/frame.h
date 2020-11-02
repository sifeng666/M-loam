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

using namespace gtsam;

//struct Pose3 {
//
//    Eigen::Quaterniond  q;
//    Eigen::Vector3d     t;
//
//    Pose3() {
//        reset();
//    }
//
//    void reset() {
//        q = Eigen::Quaterniond(1, 0, 0, 0);
//        t = Eigen::Vector3d(0, 0, 0);
//    }
//
//    void set(const Eigen::Quaterniond& _q, const Eigen::Vector3d& _t) {
//        q = _q;
//        t = _t;
//    }
//
//    Pose3 dot(const Pose3& other) const {
//        Pose3 ret = *this;
//        ret.multiply(other);
//        return ret;
//    }
//
//    Pose3 dot(const Eigen::Quaterniond& _q, const Eigen::Vector3d& _t) const {
//        Pose3 ret = *this;
//        ret.multiply(_q, _t);
//        return ret;
//    }
//
//    void set(const Eigen::Matrix4d& _T) {
//        q = Eigen::Quaterniond(_T.block<3, 3>(0, 0));
//        t = Eigen::Vector3d(_T.block<3, 1>(0, 3));
//    }
//
//    void multiply(const Pose3& other) {
//        t = t + q * other.t;
//        q = q * other.q;
//    }
//
//    void multiply(const Eigen::Quaterniond& _q, const Eigen::Vector3d& _t) {
//        t = t + q * _t;
//        q = q * _q;
//    }
//};



class Frame {

public:
    Eigen::Isometry3d pose;

    using Ptr = boost::shared_ptr<Frame>;
    using PointCloudPtr = pcl::PointCloud<PointT>::Ptr;

    // full pcd
    pcl::PointCloud<PointT>::Ptr pointCloudFull;

    // feature pcd
    pcl::PointCloud<PointT>::Ptr edgeFeatures;
    pcl::PointCloud<PointT>::Ptr surfFeatures;

public:

    explicit Frame(PointCloudPtr EF, PointCloudPtr PF, PointCloudPtr FULL = nullptr) :
            edgeFeatures(EF), surfFeatures(PF), pointCloudFull(FULL) {
        // init
        pose = Eigen::Isometry3d::Identity();
    }

    virtual void alloc() {
        return;
    }

    virtual bool is_keyframe() const {
        return false;
    }

    virtual void addEdgeFeaturesToSubMap(const PointT& p) {
        std::cout << "not keyframe, cannot add to submap." << std::endl;
        return;
    }

    virtual void addSurfFeaturesToSubMap(const PointT& p) {
        std::cout << "not keyframe, cannot add to submap." << std::endl;
        return;
    }

    virtual pcl::PointCloud<PointT>::Ptr getEdgeSubMap() {
        std::cout << "not keyframe, cannot get submap." << std::endl;
        return nullptr;
    }

    virtual pcl::PointCloud<PointT>::Ptr getSurfSubMap() {
        std::cout << "not keyframe, cannot get submap." << std::endl;
        return nullptr;
    }

    virtual ~Frame() {}

};

class Keyframe : public Frame {
public:
    Eigen::Isometry3d toLastKeyframe;
    pcl::PointCloud<PointT>::Ptr edgeSubMap;
    pcl::PointCloud<PointT>::Ptr surfSubMap;

    explicit Keyframe(PointCloudPtr EF, PointCloudPtr PF, PointCloudPtr FULL = nullptr) :
            Frame(EF, PF, FULL) {
        // init
    }

    virtual void alloc() {
        edgeSubMap = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
        surfSubMap = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
    }

    virtual bool is_keyframe() const {
        return true;
    }

    virtual void addEdgeFeaturesToSubMap(const PointT& p) {
        edgeSubMap->push_back(p);
    }

    virtual void addSurfFeaturesToSubMap(const PointT& p) {
        surfSubMap->push_back(p);
    }

    virtual pcl::PointCloud<PointT>::Ptr getEdgeSubMap() {
        return edgeSubMap;
    }

    virtual pcl::PointCloud<PointT>::Ptr getSurfSubMap() {
        return surfSubMap;
    }

    virtual ~Keyframe() {}

};

//Eigen::Affine3f fromPose3(const Pose3& poseIn) {
//    Eigen::Affine3f b;
//    Eigen::Matrix3f R = Eigen::Matrix3d(poseIn.q).cast<float>();
//    Eigen::Vector3f t = Eigen::Vector3d(poseIn.t).cast<float>();
//    b.matrix().block<3, 3>(0, 0) = R;
//    b.matrix().block<3, 1>(0, 3) = t;
//    return b;
//}
//
//gtsam::Pose3 trans2gtsamPose(const Pose3& poseIn) {
//    return gtsam::Pose3(gtsam::Quaternion(poseIn.q), gtsam::Point3(poseIn.t));
//}

gtsam::Pose3 trans2gtsamPose3(const Eigen::Isometry3d& trans) {
    return gtsam::Pose3(trans.matrix());
}

gtsam::Pose3 trans2gtsamPose3(double transformIn[]) {
    return gtsam::Pose3(gtsam::Rot3::Quaternion(transformIn[3], transformIn[0], transformIn[1], transformIn[2]),
                        gtsam::Point3(transformIn[4], transformIn[5], transformIn[6]));
}


class PoseWriter {
private:
    std::ofstream f;
public:
    explicit PoseWriter(const std::string& _name)
        : f("/home/ziv/catkin_ziv/src/M-Loam/odom/" + _name)
    { }

//    void write(const Pose3& pose, bool isKeyframe) {
//        char ret[100];
//        if (!isKeyframe)
//            sprintf(ret, "%f %f %f %f %f %f %f\n", pose.q.x(), pose.q.y(), pose.q.z(), pose.q.w(), pose.t.x(), pose.t.y(), pose.t.z());
//        else
//            sprintf(ret, "%f %f %f %f %f %f %f [keyframe]\n", pose.q.x(), pose.q.y(), pose.q.z(), pose.q.w(), pose.t.x(), pose.t.y(), pose.t.z());
//        f << ret;
//    }

    void write(const Eigen::Quaterniond& q, const Eigen::Vector3d& t, bool isKeyframe) {
        char ret[100];
        if (!isKeyframe)
            sprintf(ret, "%f %f %f %f %f %f %f\n", q.x(), q.y(), q.z(), q.w(), t.x(), t.y(), t.z());
        else
            sprintf(ret, "%f %f %f %f %f %f %f [keyframe]\n", q.x(), q.y(), q.z(), q.w(), t.x(), t.y(), t.z());
        f << ret;
    }

    void write(const Eigen::Isometry3d& mat, bool isKeyframe) {
        char ret[100];
        Eigen::Quaterniond q(mat.rotation().matrix());
        Eigen::Vector3d t(mat.translation());
        if (!isKeyframe)
            sprintf(ret, "%f %f %f %f %f %f %f\n", q.x(), q.y(), q.z(), q.w(), t.x(), t.y(), t.z());
        else
            sprintf(ret, "%f %f %f %f %f %f %f [keyframe]\n", q.x(), q.y(), q.z(), q.w(), t.x(), t.y(), t.z());
        f << ret;
    }
};


#endif //MLOAM_FRAME_H
