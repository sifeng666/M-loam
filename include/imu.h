//
// Created by cps on 2021/5/30.
//

#ifndef MLOAM_IMU_H
#define MLOAM_IMU_H

#endif //MLOAM_IMU_H

#include "factors.h"
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "tools/keyframe.hpp"
#include "tools/map_generator.hpp"
#include "tools/loop_detector.hpp"
#include "tools/registration.hpp"
#include "tools/calibration_graph.h"

using namespace tools;
using namespace std;

//IMU global status
class IMUStatus {
public:
    using Ptr = std::shared_ptr<IMUStatus>;
    bool status_change = false;
    bool is_end = false;
    int last_loop_found_index = 0;
};

class IMUMsgReader {
public:
    //using the mutex to lock the thread
    inline void lock() { imu_msg_mtx.lock(); }
    inline void unlock() { imu_msg_mtx.unlock(); }
    //put the new msg into the buff
    void IMUMsgHandler(const sensor_msgs::Imu & imuMsg){
        std::lock_guard lg(imu_msg_mtx);
        imuMsgBuf.push_back(imuMsg);
    }


private:
    std::mutex imu_msg_mtx;
public:
    // queue of ros pointcloud msg
    std::queue<sensor_msgs::Imu> imuMsgBuf;

};


class IMUSensor{
public:
    ros::NodeHandle nh; //node handler

    //topic
    string imuTopic; //imu_raw
    string odomTopic; //odometry/imu, the odometry by the imu ' s name

    // 坐标系
    string lidarFrame;      // 激光坐标系
    string baselinkFrame;   // 载体坐标系
    string odometryFrame;   // 里程计坐标系
    string mapFrame;        // 世界坐标系

    bool useImuHeadingInitialization; // whether use the imu to initilize the odom

    // IMU参数
    float imuAccNoise;          // 加速度噪声标准差
    float imuGyrNoise;          // 角速度噪声标准差
    float imuAccBiasN;          //
    float imuGyrBiasN;
    float imuGravity;           // 重力加速度
    float imuRPYWeight;
    vector<double> extRotV;
    vector<double> extRPYV;
    vector<double> extTransV;
    Eigen::Matrix3d extRot;     // xyz坐标系旋转
    Eigen::Matrix3d extRPY;     // RPY欧拉角的变换关系
    Eigen::Vector3d extTrans;   // xyz坐标系平移
    Eigen::Quaterniond extQRPY;

    IMUSensor() {
        //2 param value
        // 从param server中读取key为"iuTopic"对应的参数，存imuTopic，第三个参数是默认值
        // launch文件中定义<rosparam file="$(find lio_sam)/config/params.yaml" command="load" />，从yaml文件加载参数
        nh.param<std::string>("imuTopic", imuTopic, "imu_correct");
        nh.param<std::string>("odomTopic", odomTopic, "odometry/imu");
        nh.param<bool>("lio_sam/useImuHeadingInitialization", useImuHeadingInitialization, true);
        //other param
        nh.param<float>("imuAccNoise", imuAccNoise, 0.01);
        nh.param<float>("imuGyrNoise", imuGyrNoise, 0.001);
        nh.param<float>("imuAccBiasN", imuAccBiasN, 0.0002);
        nh.param<float>("imuGyrBiasN", imuGyrBiasN, 0.00003);
        nh.param<float>("imuGravity", imuGravity, 9.80511);
        nh.param<float>("imuRPYWeight", imuRPYWeight, 0.01);
        nh.param<vector<double>>("extrinsicRot", extRotV, vector<double>());
        nh.param<vector<double>>("lextrinsicRPY", extRPYV, vector<double>());
        nh.param<vector<double>>("extrinsicTrans", extTransV, vector<double>());
        extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
        extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
        extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
        extQRPY = Eigen::Quaterniond(extRPY);
    }

    /**
     * imu原始测量数据转换到lidar系，加速度、角速度、RPY
    */
    sensor_msgs::Imu imuConverter(const sensor_msgs::Imu& imu_in)
    {
        sensor_msgs::Imu imu_out = imu_in;
        // 加速度，只跟xyz坐标系的旋转有关系
        Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
        acc = extRot * acc;
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();
        // 角速度，只跟xyz坐标系的旋转有关系
        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
        gyr = extRot * gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();
        // RPY
        Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
        // 为什么是右乘，可以动手画一下看看
        Eigen::Quaterniond q_final = q_from * extQRPY;
        imu_out.orientation.x = q_final.x();
        imu_out.orientation.y = q_final.y();
        imu_out.orientation.z = q_final.z();
        imu_out.orientation.w = q_final.w();

        if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
        {
            ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
            ros::shutdown();
        }
        return imu_out;
    }
    /**
 * msg时间戳
*/
    template<typename T>
    double ROS_TIME(T msg)
    {
        return msg->header.stamp.toSec();
    }

/**
 * 提取imu角速度
*/
    template<typename T>
    void imuAngular2rosAngular(sensor_msgs::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z)
    {
        *angular_x = thisImuMsg->angular_velocity.x;
        *angular_y = thisImuMsg->angular_velocity.y;
        *angular_z = thisImuMsg->angular_velocity.z;
    }

/**
 * 提取imu加速度
*/
    template<typename T>
    void imuAccel2rosAccel(sensor_msgs::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z)
    {
        *acc_x = thisImuMsg->linear_acceleration.x;
        *acc_y = thisImuMsg->linear_acceleration.y;
        *acc_z = thisImuMsg->linear_acceleration.z;
    }

/**
 * 提取imu姿态角RPY
*/
    template<typename T>
    void imuRPY2rosRPY(sensor_msgs::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw)
    {
        double imuRoll, imuPitch, imuYaw;
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(thisImuMsg->orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

        *rosRoll = imuRoll;
        *rosPitch = imuPitch;
        *rosYaw = imuYaw;
    }

/**
 * 点到坐标系原点距离
*/
    float pointDistance(PointType p)
    {
        return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
    }

/**
 * 两点之间距离
*/
    float pointDistance(PointType p1, PointType p2)
    {
        return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
    }
};