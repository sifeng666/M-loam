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

using gtsam::symbol_shorthand::E; //  edge factor
using gtsam::symbol_shorthand::P; // plane factor
using gtsam::symbol_shorthand::X; // state


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

    virtual void setEdgeSubMap(pcl::PointCloud<PointT>::Ptr ptr) {
        std::cout << "not keyframe, cannot set submap." << std::endl;
        return;
    }

    virtual void setSurfSubMap(pcl::PointCloud<PointT>::Ptr ptr) {
        std::cout << "not keyframe, cannot set submap." << std::endl;
        return;
    }

    virtual ~Frame() {}

};

class Keyframe : public Frame {
public:
//    Eigen::Isometry3d toLastKeyframe;
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

    virtual void setEdgeSubMap(pcl::PointCloud<PointT>::Ptr ptr) {
        edgeSubMap = ptr;
    }

    virtual void setSurfSubMap(pcl::PointCloud<PointT>::Ptr ptr) {
        surfSubMap = ptr;
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
        : f("/home/ziv/catkin_ziv/src/M-loam/odometry/" + _name)
    { }

    void write(const Eigen::Quaterniond& q, const Eigen::Vector3d& t, bool isKeyframe) {
        if (!f.is_open()) return;
        char ret[100];
        if (!isKeyframe)
            sprintf(ret, "%f %f %f %f %f %f %f\n", q.x(), q.y(), q.z(), q.w(), t.x(), t.y(), t.z());
        else
            sprintf(ret, "%f %f %f %f %f %f %f [keyframe]\n", q.x(), q.y(), q.z(), q.w(), t.x(), t.y(), t.z());
        f << ret;
    }

    void write(const Eigen::Isometry3d& mat, bool isKeyframe) {
        Eigen::Quaterniond q(mat.rotation().matrix());
        Eigen::Vector3d t(mat.translation());
        write(q, t, isKeyframe);
    }

    void write(const gtsam::Pose3& p3, bool isKeyframe) {
        Eigen::Isometry3d mat(p3.matrix());
        write(mat, isKeyframe);
    }

    void close() {
        f.close();
    }
};

void _mkdir(const std::string& filename) {
    boost::filesystem::path save_path(filename);
    auto folder_path = save_path.parent_path();
    if (!boost::filesystem::exists(folder_path)) {
        boost::filesystem::create_directories(folder_path);
    }
}

gtsam::Pose3 pose_normalize(const gtsam::Pose3& pose) {
    Eigen::Quaterniond q(Eigen::Isometry3d(pose.matrix()).rotation());
    q.normalize();
    return gtsam::Pose3(gtsam::Rot3(q.matrix()), pose.translation());
}

geometry_msgs::TransformStamped navOdomToTransformStamped(const nav_msgs::Odometry& odometry) {

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = odometry.header.frame_id;
    odom_trans.child_frame_id = odometry.child_frame_id;

    odom_trans.transform.rotation = odometry.pose.pose.orientation;
    odom_trans.transform.translation.x = odometry.pose.pose.position.x;
    odom_trans.transform.translation.y = odometry.pose.pose.position.y;
    odom_trans.transform.translation.z = odometry.pose.pose.position.z;

    return odom_trans;

}

nav_msgs::Odometry poseToNavOdometry(
        const ros::Time& stamp,
        const Eigen::Isometry3d& pose,
        const std::string& frame_id,
        const std::string& child_frame_id) {

    nav_msgs::Odometry odometry;

    odometry.header.frame_id = frame_id;
    odometry.child_frame_id = child_frame_id;
    odometry.header.stamp = stamp;
    Eigen::Quaterniond q(pose.rotation().matrix());

    odometry.pose.pose.orientation.x    = q.x();
    odometry.pose.pose.orientation.y    = q.y();
    odometry.pose.pose.orientation.z    = q.z();
    odometry.pose.pose.orientation.w    = q.w();
    odometry.pose.pose.position.x       = pose.translation().x();
    odometry.pose.pose.position.y       = pose.translation().y();
    odometry.pose.pose.position.z       = pose.translation().z();

    return odometry;

}

#endif //MLOAM_FRAME_H
