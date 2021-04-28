//
// Created by ziv on 2021/4/21.
//
#include "helper.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using PointT = pcl::PointXYZI;

int main() {
    string path = "/home/ziv/mloam/5g/";

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_outliers(new pcl::PointCloud<PointT>);

    TicToc t1;
    pcl::io::loadPCDFile(path + "loop.pcd", *cloud);
    cout << "load: " << t1.toc() << " ms" << endl;


    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(30);
    sor.setStddevMulThresh(1.0);
    cout << "starting" << endl;
    TicToc t2;
    sor.filter(*cloud_filtered);
    cout << "filter: " << t2.toc() << " ms" << endl;
    TicToc t3;
    sor.setNegative(true);
    sor.filter(*cloud_outliers);
    cout << "filter neg: " << t3.toc() << " ms" << endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));

    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> white(cloud_filtered, 255, 255, 255);
    viewer->addPointCloud<PointT> (cloud_filtered, white, "source");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> red(cloud_outliers, 255, 0, 0);
    viewer->addPointCloud<PointT> (cloud_outliers, red, "o");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "o");

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
    }

    pcl::io::savePCDFile(path + "loop_sor_50.pcd", *cloud_filtered);
    return 0;
}