//
// Created by ziv on 2021/2/2.
//


#include <iostream>
#include <string>
#include <thread>

#include "../include/tools/registration.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <gtsam/geometry/Pose3.h>
using namespace std;
using namespace gtsam;

using PointT = pcl::PointXYZI;

struct Odom {
    size_t index;
    Pose3 pose;
    Eigen::Matrix6d information;
};

struct Loop {
    size_t from, to;
    Pose3 pose;
    Eigen::Matrix6d information;
};

void read_from_file(string pose_raw_filename, string loop_filename, vector<Odom>& odoms, vector<Loop>& loops) {

    ifstream f_fixed_raw(pose_raw_filename);
    ifstream f_loop_info(loop_filename);

    Eigen::Matrix4d pose(Eigen::Matrix4d::Identity());
    Eigen::Matrix6d info(Eigen::Matrix6d::Identity());

    while (!f_fixed_raw.eof()) {
        int k;
        f_fixed_raw >> k;
        // Read ceres result
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                f_fixed_raw >> *(pose.data() + i+j*4);
            }
        }
        if (k > 0) {
            for (int i = 0; i < 6; i++) {
                for (int j = 0; j < 6; j++) {
                    f_fixed_raw >> *(info.data() + i+j*6);
                }
            }
        }
        Odom odom;
        odom.pose = Pose3(pose);
        odom.information = info;
        odom.index = k;
        odoms.push_back(odom);
    }

//    while (!f_loop_info.eof()) {
//        int from, to;
//        f_loop_info >> from >> to;
//        // Read ceres result
//        for (int i = 0; i < 4; i++) {
//            for (int j = 0; j < 4; j++) {
//                f_loop_info >> *(pose.data() + i+j*4);
//            }
//        }
//
//        for (int i = 0; i < 6; i++) {
//            for (int j = 0; j < 6; j++) {
//                f_loop_info >> *(info.data() + i+j*6);
//            }
//        }
//
//        Loop loop;
//        loop.pose = Pose3(pose);
//        loop.information = info;
//        loop.from = from;
//        loop.to = to;
//        loops.push_back(loop);
//    }

    odoms.pop_back();
//    loops.pop_back();
}

int main() {

    string path = "/home/ziv/mloam/result/building-004/12-3-1-0.1/0/";

    pcl::PointCloud<PointT>::Ptr submap(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr temp(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr curr(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr align(new pcl::PointCloud<PointT>);

    pcl::io::loadPCDFile(path + "submap.pcd", *submap);
    pcl::io::loadPCDFile(path + "593.pcd", *curr);

    fstream tmp_result (path + "guess.txt");
    Eigen::Matrix4d init_guess;
    // Read ceres result
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            tmp_result >> *(init_guess.data() + i+j*4);
        }
    }
    cout << "init guess:\n" << init_guess << endl;

    Eigen::Matrix4d T;


    tools::FastGeneralizedRegistration(curr, submap, T, init_guess);

//    TicToc tic;
    auto info = tools::GetInformationMatrixFromPointClouds(curr, submap, 0.5, T);
    cout << "Infomation: \n" << info << endl;
//    cout << "cost: " << tic.toc() << "ms" << endl;
    info = tools::GetInformationMatrixFromPointClouds(submap, curr, 0.5, T);
    cout << "Infomation: \n" << info << endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));

    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> white(submap, 255, 255, 255);
    viewer->addPointCloud<PointT> (submap, white, "source");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source");

//    pcl::visualization::PointCloudColorHandlerCustom<PointT> green(curr, 0, 255, 0);
//    viewer->addPointCloud<PointT> (curr, green, "target");
//    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target");

    pcl::transformPointCloud(*curr, *align, init_guess);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> blue(align, 255, 0, 0);
    viewer->addPointCloud<PointT> (align, blue, "align");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "align");

    pcl::transformPointCloud(*curr, *curr, T);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> green(curr, 0, 255, 0);
    viewer->addPointCloud<PointT> (curr, green, "T");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "T");

    viewer->addCoordinateSystem (1.0);
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
    }

}