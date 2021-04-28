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
#include <pcl/filters/voxel_grid.h>

#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
using namespace std;
using namespace gtsam;

using PointT = pcl::PointXYZI;

bool ndt_matching(pcl::PointCloud<PointT>::Ptr source,
         pcl::PointCloud<PointT>::Ptr target,
         Eigen::Matrix4d& final,
         const Eigen::Matrix4d& init_guess = Eigen::Matrix4d::Identity(),
         double max_correspondence_distance = 0.5,
         double fitness_thres = 0.8) {

    pcl::NormalDistributionsTransform<PointT, PointT> ndt;
    ndt.setTransformationEpsilon(0.01);
    ndt.setMaximumIterations(64);
    ndt.setResolution(2.0);

    // Align clouds
    ndt.setInputSource(source);
    ndt.setInputTarget(target);

    pcl::PointCloud<PointT>::Ptr unused_result(new pcl::PointCloud<PointT>());
    ndt.align(*unused_result, init_guess.cast<float>());

    if (!ndt.hasConverged()) {
        cout << "ndt not converged" << endl;
        return false;
    }
    printf("fitness score: %f\n", ndt.getFitnessScore());
    cout << "ndt: \n" << ndt.getFinalTransformation().cast<double>() << endl;
    if (ndt.getFitnessScore() > fitness_thres) {
        return false;
    }

    final = ndt.getFinalTransformation().cast<double>();
    return true;
}

bool icp_matching(pcl::PointCloud<PointT>::Ptr source,
                  pcl::PointCloud<PointT>::Ptr target,
                  Eigen::Matrix4d& final,
                  const Eigen::Matrix4d& init_guess = Eigen::Matrix4d::Identity(),
                  double max_correspondence_distance = 0.5,
                  double fitness_thres = 0.8) {

    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setTransformationEpsilon(0.01);
    icp.setMaximumIterations(100);
    icp.setMaxCorrespondenceDistance(10);
    icp.setUseReciprocalCorrespondences(false);

    // Align clouds
    icp.setInputSource(source);
    icp.setInputTarget(target);

    pcl::PointCloud<PointT>::Ptr unused_result(new pcl::PointCloud<PointT>());
    icp.align(*unused_result, init_guess.cast<float>());

    if (!icp.hasConverged()) {
        cout << "icp not converged" << endl;
        return false;
    }
    printf("fitness score: %f\n", icp.getFitnessScore());
    cout << "icp: \n" << icp.getFinalTransformation().cast<double>() << endl;
    if (icp.getFitnessScore() > fitness_thres) {
        return false;
    }

    final = icp.getFinalTransformation().cast<double>();
    return true;
}

bool gicp_matching(pcl::PointCloud<PointT>::Ptr source,
                  pcl::PointCloud<PointT>::Ptr target,
                  Eigen::Matrix4d& final,
                  const Eigen::Matrix4d& init_guess = Eigen::Matrix4d::Identity(),
                  double max_correspondence_distance = 0.5,
                  double fitness_thres = 0.8) {

    pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;

    gicp.setTransformationEpsilon(0.1);
    gicp.setMaximumIterations(64);
    gicp.setUseReciprocalCorrespondences(false);
    gicp.setMaxCorrespondenceDistance(10);
    gicp.setCorrespondenceRandomness(20);
    gicp.setMaximumOptimizerIterations(20);

    // Align clouds
    gicp.setInputSource(source);
    gicp.setInputTarget(target);

    pcl::PointCloud<PointT>::Ptr unused_result(new pcl::PointCloud<PointT>());
    gicp.align(*unused_result, init_guess.cast<float>());

    if (!gicp.hasConverged()) {
        return false;
    }
    printf("fitness score: %f\n", gicp.getFitnessScore());

    cout << "gicp: \n" << gicp.getFinalTransformation().cast<double>() << endl;

    if (gicp.getFitnessScore() > fitness_thres) {
        return false;
    }

    final = gicp.getFinalTransformation().cast<double>();
    return true;

}


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

    string path = "/home/ziv/mloam/debug_loop/30/";

//    pcl::PointCloud<PointT>::Ptr gmap(new pcl::PointCloud<PointT>);
//    pcl::io::loadPCDFile(path + "global_info_opti.pcd", *gmap);
//    pcl::VoxelGrid<PointT> vg;
//    vg.setLeafSize(0.2, 0.2, 0.2);
//    vg.setInputCloud(gmap);
//    vg.filter(*gmap);
//    pcl::io::savePCDFile(path + "global_info_opti_DS.pcd", *gmap);

//    return 0;
    pcl::PointCloud<PointT>::Ptr submap(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr temp(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr curr(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr align(new pcl::PointCloud<PointT>);

    pcl::io::loadPCDFile(path + "submap.pcd", *submap);
    pcl::io::loadPCDFile(path + "curr.pcd", *curr);

    pcl::VoxelGrid<PointT> f;
    f.setLeafSize(0.2, 0.2, 0.2);
    f.setInputCloud(submap);
    f.filter(*submap);
    f.setInputCloud(curr);
    f.filter(*curr);

    fstream tmp_result (path + "guess.txt");
    Eigen::Matrix4d init_guess;
    init_guess = Eigen::Matrix4d::Identity();
    // Read ceres result
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            tmp_result >> *(init_guess.data() + i+j*4);
        }
    }
    init_guess(14) = 0;
    cout << "init guess:\n" << init_guess << endl;
//    Eigen::Matrix4d init_guess2;
//    init_guess2 << 0.999896049500, -0.001376749598, 0.014355959371, 2.521043777466,
//    0.001601068303, 0.999876976013, -0.015625659376, 0.483040392399,
//    -0.014332676306, 0.015647012740 ,0.999774694443, -0.045610629022,
//    0.000000000000, 0.000000000000, 0.000000000000, 1.000000000000;
//    cout << init_guess2 << endl;
//
//    init_guess = init_guess2 * init_guess;
//    cout << "new init guess:\n" << init_guess << endl;
    Eigen::Matrix4d T;
    T = Eigen::Matrix4d::Identity();

    TicToc tic;
    cout << "fast-gicp: " << tools::FastGeneralizedRegistration(curr, submap, T, init_guess) << ", " << tic.toc() << " ms" << endl; tic.tic();
    cout << "T: \n" << T << endl;

    cout << "gicp: " <<  gicp_matching(curr, submap, T, init_guess) << ", " << tic.toc() << " ms" << endl; tic.tic();
    cout << "T: \n" << T << endl;
    cout << "icp: " <<  icp_matching(curr, submap, T, init_guess) << ", " << tic.toc() << " ms" << endl; tic.tic();
    cout << "T: \n" << T << endl;
    cout << "ndt: " <<  ndt_matching(curr, submap, T, init_guess) << ", " << tic.toc() << " ms" << endl; tic.tic();
    cout << "T: \n" << T << endl;
//
//    TicToc tic;
    auto info = tools::GetInformationMatrixFromPointClouds(curr, submap, 0.5, T);
    cout << "Infomation: \n" << info << endl;
    cout << "cost: " << tic.toc() << "ms" << endl;
//    info = tools::GetInformationMatrixFromPointClouds(submap, curr, 0.5, T);
//    cout << "Infomation: \n" << info << endl;

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