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

    int good_time = 20;
    int bad_time = 3;


    int first_good_time =  30;
    ros::Duration(first_good_time).sleep();
    while (ros::ok()) {

        std_msgs::Int32 data;
        data.data = bad_time * 1000;
        publisher.publish(data);

        ros::spinOnce();
        ros::Duration(good_time).sleep();
    }

    return 0;
}