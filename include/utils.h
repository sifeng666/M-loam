//
// Created by ziv on 2020/12/7.
//

#ifndef MLOAM_UTILS_H
#define MLOAM_UTILS_H

#include "helper.h"
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

extern const std::string filepath = "/home/ziv/mloam/";


void _mkdir(const std::string& filename) {
    boost::filesystem::path save_path(filename);
    auto folder_path = save_path.parent_path();
    if (!boost::filesystem::exists(folder_path)) {
        boost::filesystem::create_directories(folder_path);
    }
}


string pose_to_str(const gtsam::Pose3& pose) {
    char ret[128];
    auto rpy = pose.rotation().rpy();
    sprintf(ret, "%f %f %f %f %f %f\n", rpy[0], rpy[1], rpy[2], pose.translation().x(), pose.translation().y(), pose.translation().z());
    return string(ret);
}

void write_pose(std::ofstream& f, const Eigen::Quaterniond& q, const Eigen::Vector3d& t, bool isKeyframe) {
    if (!f.is_open()) return;
    char ret[100];
    if (!isKeyframe)
        sprintf(ret, "%f %f %f %f %f %f %f\n", q.x(), q.y(), q.z(), q.w(), t.x(), t.y(), t.z());
    else
        sprintf(ret, "%f %f %f %f %f %f %f [keyframe]\n", q.x(), q.y(), q.z(), q.w(), t.x(), t.y(), t.z());
    f << ret;
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


#endif //MLOAM_UTILS_H
