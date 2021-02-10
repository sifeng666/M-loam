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
    string path = "/home/ziv/mloam/building003/";

    vector<gtsam::Pose3> poseVec;
    vector<pcl::PointCloud<PointT>::Ptr> pcdVec;
    fstream tmp_result (path + "without_loop.txt");
    for (int i = 0; i < 10; i++) {
        pcl::PointCloud<PointT>::Ptr temp(new pcl::PointCloud<PointT>());
        pcl::io::loadPCDFile(path + to_string(i) + ".pcd", *temp);
        Eigen::Matrix4d pose(Eigen::Matrix4d::Identity());
        string drop1, drop2;
        tmp_result >> drop1 >> drop2;
        // Read ceres result
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                tmp_result >> *(pose.data() + i+j*4);
            }
        }
        poseVec.push_back(gtsam::Pose3(pose));
        pcdVec.push_back(temp);
    }



    for (int i = 1; i < 10; i++) {
        auto poseBet = poseVec[i-1].between(poseVec[i]);
        cout << poseBet << endl;
        pcl::PointCloud<PointT>::Ptr temp(new pcl::PointCloud<PointT>());
        pcl::transformPointCloud(*pcdVec[i], *temp, poseBet.matrix());

        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));

        viewer->setBackgroundColor (0, 0, 0);
        pcl::visualization::PointCloudColorHandlerCustom<PointT> white(pcdVec[i - 1], 255, 255, 255);
        viewer->addPointCloud<PointT> (pcdVec[i - 1], white, "1");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "1");

//    pcl::visualization::PointCloudColorHandlerCustom<PointT> green(curr, 0, 255, 0);
//    viewer->addPointCloud<PointT> (curr, green, "target");
//    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target");

        pcl::visualization::PointCloudColorHandlerCustom<PointT> blue(temp, 255, 0, 0);
        viewer->addPointCloud<PointT> (temp, blue, "2");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "2");

        pcl::visualization::PointCloudColorHandlerCustom<PointT> green(pcdVec[i], 0, 255, 0);
        viewer->addPointCloud<PointT> (pcdVec[i], green, "3");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "3");

        viewer->addCoordinateSystem (1.0);
        while (!viewer->wasStopped ())
        {
            viewer->spinOnce (100);
        }

    }

}

