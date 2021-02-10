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

using namespace std;

using PointT = pcl::PointXYZI;

int main() {

    string path = "/home/ziv/mloam/debug_loop/2/";

    pcl::PointCloud<PointT>::Ptr submap(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr curr(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr align(new pcl::PointCloud<PointT>);

    pcl::io::loadPCDFile(path + "submap.pcd", *submap);
    pcl::io::loadPCDFile(path + "curr.pcd", *curr);

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