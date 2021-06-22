//
// Created by ziv on 2021/4/27.
//

#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <chrono>
#include <random>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

using namespace std;

int main(int argc, char **argv) {

    ros::init(argc, argv, "latencyMaker");
    ros::NodeHandle nh;
    ros::Publisher publisher;
    publisher = nh.advertise<std_msgs::Int32>("/latency", 1);

    int interval, sustain;
    interval = nh.param<int>("interval", 5);
    sustain = nh.param<int>("sustain", 2);

    int first_good_time =  20;
    ros::Duration(first_good_time).sleep();
    while (ros::ok()) {

        std_msgs::Int32 data;
        data.data = sustain * 1000;
        publisher.publish(data);

        ros::spinOnce();
        ros::Duration(interval).sleep();
    }

    return 0;
}