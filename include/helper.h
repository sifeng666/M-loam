//
// Created by ziv on 2020/10/8.
//

#ifndef MLOAM_HELPER_H
#define MLOAM_HELPER_H

#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <algorithm>
#include <queue>
#include <deque>
#include <mutex>
#include <shared_mutex>
#include <cstdio>
#include <thread>
#include <utility>
#include <random>
#include <chrono>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseArray.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

using namespace std;

using PointT = pcl::PointXYZI;

static std::atomic<int> timerCount = 1;

/// Thread-safe function returning a pseudo-random integer.
/// The integer is drawn from a uniform distribution bounded by min and max
/// (inclusive) (reference from open3d)
[[maybe_unused]] static int UniformRandInt(const int min, const int max) {
    static thread_local std::mt19937 generator(std::random_device{}());
    std::uniform_int_distribution<int> distribution(min, max);
    return distribution(generator);
}


class TimeCounter {
private:
    using clock_type = std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds>;
    long long int startTime;
public:
    explicit TimeCounter() {
        clock_type tp1 = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
        startTime = tp1.time_since_epoch().count();
    }
    long long int count() {
        return std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()).time_since_epoch().count() - startTime;
    }
};

class Timer {
private:
    using clock_type = std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds>;
    long long int startTime;
    std::string name;
public:
    explicit Timer(std::string _name) {
        name = _name;
        clock_type tp1 = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
        startTime = tp1.time_since_epoch().count();
    }
    void count() {
        clock_type tp2 = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
        std::cout << "Timer_" << name << "_" << timerCount++ << " count: " << tp2.time_since_epoch().count() - startTime << "msec" << std::endl;
    }
};

class TicToc {
public:
    TicToc() {
        tic();
    }
    void tic() {
        start = std::chrono::system_clock::now();
    }
    double toc() {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }

private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};


#endif //MLOAM_HELPER_H
