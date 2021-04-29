//
// Created by ziv on 2021/1/6.
//

#include <pcl/octree/octree_search.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "../include/tools/registration.hpp"
#include <gtsam/geometry/Pose3.h>

using namespace std;

using PointT = pcl::PointXYZI;

int main() {
    string path = "/home/ziv/mloam/result/test/debug/";

    vector<gtsam::Pose3> poseVec;
    vector<gtsam::Pose3> poseVec1;
    vector<pcl::PointCloud<PointT>::Ptr> pcdVec;
    fstream tmp_result(path + "lidar0fixed_poses_raw.txt");
    fstream tmp_result2(path + "lidar0backend_timecost.txt");
    fstream tmp_result3(path + "lidar0balm.txt");

    int count = 8;

    for (int i = 0; i < count; i++) {
        pcl::PointCloud<PointT>::Ptr temp(new pcl::PointCloud<PointT>());
        pcl::io::loadPCDFile(path + to_string(i) + ".pcd", *temp);
        Eigen::Matrix4d pose(Eigen::Matrix4d::Identity());
        // Read ceres result
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                tmp_result2 >> *(pose.data() + i+j*4);
            }
        }
        cout << "pose:\n" << pose.matrix() << endl;
        poseVec.push_back(gtsam::Pose3(pose));
        pcdVec.push_back(temp);
    }

    for (int i = 0; i < count; i++) {
        Eigen::Matrix4d pose(Eigen::Matrix4d::Identity());
        // Read ceres result
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                tmp_result3 >> *(pose.data() + i+j*4);
            }
        }
        cout << "pose:\n" << pose.matrix() << endl;
        poseVec1.push_back(gtsam::Pose3(pose));
    }

    pcl::PointCloud<PointT>::Ptr map1(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr map2(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr tmp(new pcl::PointCloud<PointT>());
    for (int i = 0; i < count; i++) {
        pcl::transformPointCloud(*pcdVec[i], *tmp, poseVec[i].matrix());
        *map1 += *tmp;
        pcl::transformPointCloud(*pcdVec[i], *tmp, poseVec1[i].matrix());
        *map2 += *tmp;
    }


    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> white(map1, 255, 255, 255);
    viewer->addPointCloud<PointT> (map1, white, "1");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "1");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> red(map1, 255, 0, 0);
    viewer->addPointCloud<PointT> (map2, red, "2");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "2");

    viewer->addCoordinateSystem (1.0);
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
    }

}

