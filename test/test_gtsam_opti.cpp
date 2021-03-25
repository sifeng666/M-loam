//
// Created by ziv on 2021/2/9.
//

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>
#include "calibration_graph.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include "helper.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iomanip>

#include <gtsam/nonlinear/ISAM2.h>

using namespace std;
using namespace gtsam;
using namespace ct;
using PointT = pcl::PointXYZI;

struct Odom {
    Pose3 pose;
    double time_stamp;
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
        double k;
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
        odom.time_stamp = k;
        odom.pose = Pose3(pose);
        odom.information = info;
        odoms.push_back(odom);
    }

    while (!f_loop_info.eof()) {
        int from, to;
        f_loop_info >> from >> to;
        // Read ceres result
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                f_loop_info >> *(pose.data() + i+j*4);
            }
        }

        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                f_loop_info >> *(info.data() + i+j*6);
            }
        }

        Loop loop;
        loop.pose = Pose3(pose);
        loop.information = info;
        loop.from = from;
        loop.to = to;
        loops.push_back(loop);
    }

    odoms.pop_back();
    loops.pop_back();
}


int main(int argc, char** argv) {

    string path = "/home/ziv/mloam/result/dzf-005/";

    vector<Odom> odoms;
    vector<Loop> loops;
    int current_lidar = 0;

    TicToc tic;
    read_from_file(path + "/lidar"+to_string(current_lidar)+"fixed_poses_raw.txt", path + "/lidar"+to_string(current_lidar)+"f_loop.txt", odoms, loops);
    cout << "read_from_file: " << tic.toc() << "ms" << endl;
    cout << odoms.size() << endl;
//    return 0;
    /// iSAM params
//    ISAM2Params isam2Params;
//    isam2Params.relinearizeThreshold = 0.1;
//    isam2Params.relinearizeSkip = 1;
//    ISAM2 isam(isam2Params);

    /// noise models
    Vector6 diagonal; diagonal << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6;
    auto prior_model = noiseModel::Diagonal::Variances(diagonal);

    auto loop_model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-2, 1e-2, 1e-1).finished());
    auto odom_model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 5e-2, 5e-2, 5e-1).finished());

    /// Factors container
    NonlinearFactorGraph new_factors;
    Values new_values, current_estimate;

    vector<pcl::PointCloud<PointT>::Ptr> pcds;

    string pcd_path = path + to_string(current_lidar) + "/";
    tic.tic();
    for (int i = 0; i < odoms.size(); i++) {
        pcl::PointCloud<PointT>::Ptr pcd(new pcl::PointCloud<PointT>);
        pcl::io::loadPCDFile(pcd_path + to_string(i) + ".pcd", *pcd);
        pcds.push_back(pcd);
    }
    cout << "loadPCDFile: " << tic.toc() << "ms" << endl;

//    pcl::PointCloud<PointT>::Ptr submap(new pcl::PointCloud<PointT>);
//    pcl::PointCloud<PointT>::Ptr temp1(new pcl::PointCloud<PointT>);
//    for (int i = 0; i < 15; i++) {
//        cout << odoms[i].pose.matrix() << endl;
//        pcl::transformPointCloud(*pcds[i], *temp1, odoms[i].pose.matrix());
//        *submap += *temp1;
//    }
//    pcl::io::savePCDFileASCII(path + "submap.pcd", *submap);
//    return 1;

    //////////////////////////////////////////////  Traditional BA  //////////////////////////////////////////////////


    CalibrationGraph cg = CalibrationGraph();

    // Add from file data
    for (size_t i = 0; i < odoms.size(); i++) {
        if (i == 0) {
            cg.set_initial(0, odoms[i].pose.matrix());
        } else {
            Eigen::Matrix4d measurement = odoms[i-1].pose.between(odoms[i].pose).matrix();
            cg.add_edge(i - 1, i, measurement, odoms[i].information, true);
            cg.set_initial(i, odoms[i].pose.matrix());
        }
    }

    cg.optimize(5);

    cg.graph().saveGraph(path + "gtsam/lidar"+to_string(current_lidar)+"_before_raw.dot", cg.result());

    auto raw_res = cg.result();

    // Add loop
    for (const auto& loop : loops) {
        cg.add_edge(loop.from, loop.to, loop.pose.matrix(), loop.information, true);
        cout << "!" << endl;
    }
    cg.optimize(5);

    cg.graph().saveGraph(path + "gtsam/lidar"+to_string(current_lidar)+"after_opti.dot", cg.result());

    auto opti_res = cg.result();

    ofstream f_out(path + "Ours.txt");
    for (int i = 0; i < opti_res.size(); i++) {
        auto pose = opti_res.at<Pose3>(i);
        auto q = pose.rotation().toQuaternion();
        f_out << setprecision(20) << odoms[i].time_stamp/1e9 << " " << pose.x() << " " << pose.y() << " " << pose.z() << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
//        f_out << i << endl;
//        f_out << opti_res.at<Pose3>(i).matrix() << endl;
    }

    pcl::PointCloud<PointT>::Ptr global_raw(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr global_opti(new pcl::PointCloud<PointT>);

    pcl::PointCloud<PointT>::Ptr temp(new pcl::PointCloud<PointT>());
    tic.tic();
    for (int i = 0; i < raw_res.size(); i++) {
        pcl::transformPointCloud(*pcds[i], *temp, raw_res.at<Pose3>(i).matrix());
        *global_raw += *temp;
    }
    cout << "global_raw: " << tic.toc() << "ms" << endl;tic.tic();
    for (int i = 0; i < opti_res.size(); i++) {
        pcl::transformPointCloud(*pcds[i], *temp, opti_res.at<Pose3>(i).matrix());
        *global_opti += *temp;
    }
    cout << "global_opti: " << tic.toc() << "ms" << endl;

    pcl::io::savePCDFileASCII(pcd_path + "global_raw.pcd", *global_raw);
    pcl::io::savePCDFileASCII(pcd_path + "global_info_opti.pcd", *global_opti);

}