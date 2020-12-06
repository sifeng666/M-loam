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

//using namespace gtsam;

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

    explicit Keyframe(int _index, PointCloudPtr EF, PointCloudPtr PF) :
            index(_index), edgeFeatures(EF), surfFeatures(PF) {
        // init
        pose = gtsam::Pose3::identity();
        frameCount = 0;

        edgeSlice = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
        surfSlice = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());

    }

    void normalizePose() {
        Eigen::Quaterniond q(Eigen::Isometry3d(pose.matrix()).rotation());
        q.normalize();
        pose = gtsam::Pose3(gtsam::Rot3(q.matrix()), pose.translation());
    }



};

string pose_to_str(const gtsam::Pose3& pose) {
    char ret[128];
    auto rpy = pose.rotation().rpy();
    sprintf(ret, "%f %f %f %f %f %f\n", rpy[0], rpy[1], rpy[2], pose.translation().x(), pose.translation().y(), pose.translation().z());
    return string(ret);
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
        const gtsam::Pose3& pose,
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
