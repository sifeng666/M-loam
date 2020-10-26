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

struct Pose3 {

    Eigen::Quaterniond  q;
    Eigen::Vector3d     t;

    Pose3() {
        reset();
    }

    void reset() {
        q = Eigen::Quaterniond(1, 0, 0, 0);
        t = Eigen::Vector3d(0, 0, 0);
    }

    void set(const Eigen::Quaterniond& _q, const Eigen::Vector3d& _t) {
        q = _q;
        t = _t;
    }

    Pose3 dot(const Pose3& other) const {
        Pose3 ret = *this;
        ret.multiply(other);
        return ret;
    }

    Pose3 dot(const Eigen::Quaterniond& _q, const Eigen::Vector3d& _t) const {
        Pose3 ret = *this;
        ret.multiply(_q, _t);
        return ret;
    }

    void set(const Eigen::Matrix4d& _T) {
        q = Eigen::Quaterniond(_T.block<3, 3>(0, 0));
        t = Eigen::Vector3d(_T.block<3, 1>(0, 3));
    }

    void multiply(const Pose3& other) {
        t = t + q * other.t;
        q = q * other.q;
    }

    void multiply(const Eigen::Quaterniond& _q, const Eigen::Vector3d& _t) {
        t = t + q * _t;
        q = q * _q;
    }
};



class Frame {

private:
    Pose3 toLastFrame, toLastKeyframe;
    bool keyframe = false;

public:

    using Ptr = boost::shared_ptr<Frame>;
    using PointCloudPtr = pcl::PointCloud<PointT>::Ptr;

    // raw pcd
    pcl::PointCloud<PointT>::ConstPtr rawPointCloud;

    // feature pcd
    pcl::PointCloud<PointT>::ConstPtr edgeFeatures;
    pcl::PointCloud<PointT>::ConstPtr planeFeatures;
    pcl::PointCloud<PointT>::ConstPtr edgeLessFeatures;
    pcl::PointCloud<PointT>::ConstPtr planeLessFeatures;

public:

    explicit Frame(PointCloudPtr EF, PointCloudPtr PF, PointCloudPtr ELF, PointCloudPtr PLF, PointCloudPtr RAW = nullptr) :
            edgeFeatures(EF), planeFeatures(PF), edgeLessFeatures(ELF), planeLessFeatures(PLF), rawPointCloud(RAW) {
        // init
    }

    bool is_keyframe() const {
        return keyframe;
    }

    void set_keyframe() {
        keyframe = true;
    }

    void setTrans2LastFrame(const Eigen::Quaterniond& _q_2LastFrame, const Eigen::Vector3d& _t_2LastFrame) {
        toLastFrame.set(_q_2LastFrame, _t_2LastFrame);
    }

    void setTrans2LastFrame(const Eigen::Matrix4d& _T_2LastFrame) {
        toLastFrame.set(_T_2LastFrame);
    }

    void setTrans2LastKeyframe(const Eigen::Quaterniond& _q_2LastKeyframe, const Eigen::Vector3d& _t_2LastKeyframe) {
        toLastKeyframe.set(_q_2LastKeyframe, _t_2LastKeyframe);
    }

    void setTrans2LastKeyframe(const Eigen::Matrix4d& _T_2LastKeyframe) {
        toLastKeyframe.set(_T_2LastKeyframe);
    }
};

Eigen::Affine3f fromPose3(const Pose3& poseIn) {
    Eigen::Affine3f b;
    Eigen::Matrix3f R = Eigen::Matrix3d(poseIn.q).cast<float>();
    Eigen::Vector3f t = Eigen::Vector3d(poseIn.t).cast<float>();
    b.matrix().block<3, 3>(0, 0) = R;
    b.matrix().block<3, 1>(0, 3) = t;
    return b;
}

gtsam::Pose3 trans2gtsamPose(const Pose3& poseIn) {
    return gtsam::Pose3(gtsam::Quaternion(poseIn.q), gtsam::Point3(poseIn.t));
}


class PoseWriter {
private:
    std::ofstream f;
public:
    explicit PoseWriter(const std::string& _name) : f("/home/ziv/catkin_ziv/src/M-Loam/odom/" + _name) {

    }

    void write(const Pose3& pose, bool isKeyframe) {
        char ret[100];
        if (!isKeyframe)
            sprintf(ret, "%f %f %f %f %f %f %f\n", pose.q.x(), pose.q.y(), pose.q.z(), pose.q.w(), pose.t.x(), pose.t.y(), pose.t.z());
        else
            sprintf(ret, "%f %f %f %f %f %f %f [keyframe]\n", pose.q.x(), pose.q.y(), pose.q.z(), pose.q.w(), pose.t.x(), pose.t.y(), pose.t.z());
        f << ret;
    }

    void write(const Eigen::Quaterniond& q, const Eigen::Vector3d& t, bool isKeyframe) {
        char ret[100];
        if (!isKeyframe)
            sprintf(ret, "%f %f %f %f %f %f %f\n", q.x(), q.y(), q.z(), q.w(), t.x(), t.y(), t.z());
        else
            sprintf(ret, "%f %f %f %f %f %f %f [keyframe]\n", q.x(), q.y(), q.z(), q.w(), t.x(), t.y(), t.z());
        f << ret;
    }
};


#endif //MLOAM_FRAME_H
