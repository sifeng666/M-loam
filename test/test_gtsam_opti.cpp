//
// Created by ziv on 2021/2/9.
//

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <gtsam/geometry/Pose3.h>
#include "calibration_graph.h"
#include "helper.h"

#include <gtsam/nonlinear/ISAM2.h>

using namespace std;
using namespace gtsam;
using namespace ct;


struct Odom {
    int index;
    Pose3 pose;
    Eigen::Matrix6d information;
};

struct Loop {
    int from, to;
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


int main() {

    string path = "/home/ziv/catkin_ziv/src/M-loam/test/";

    vector<Odom> odoms;
    vector<Loop> loops;

    read_from_file(path + "pose_fixed_raw.txt", path + "f_loop.txt", odoms, loops);

    CalibrationGraph cg;

    gtsam::ISAM2Params isam2Params;
    isam2Params.relinearizeThreshold = 0.1;
    isam2Params.relinearizeSkip = 1;
    ISAM2 isam(isam2Params);

    // isam
    for (int i = 0; i < odoms.size(); i++) {
        if (i == 0) {
            cg.set_initial(0, odoms[i].pose.matrix());
        } else {
            Eigen::Matrix4d measurement = odoms[i-1].pose.between(odoms[i].pose).matrix();
            cg.add_edge(i - 1, i, measurement, odoms[i].information);
            cg.set_initial(i, odoms[i].pose.matrix());
        }
        if (i % 4 == 0) {
            isam.update(cg.graph(), cg.initial());
            isam.update();
            cg.result() = isam.calculateEstimate();
            cg.initial().clear();
            cg.graph().resize(0);
        }

        for (const auto& loop : loops) {
            if (i == loop.to)
                cg.add_edge(loop.from, loop.to, loop.pose.matrix(), loop.information);
            isam.update(cg.graph(), cg.initial());
            isam.update();
            cg.result() = isam.calculateEstimate();
            cg.initial().clear();
            cg.graph().resize(0);
        }
    }

    isam.getFactorsUnsafe().saveGraph(path + "isam.dot", cg.result());

    cg = CalibrationGraph();

    for (int i = 0; i < odoms.size(); i++) {
        if (i == 0) {
            cg.set_initial(0, odoms[i].pose.matrix());
        } else {
            Eigen::Matrix4d measurement = odoms[i-1].pose.between(odoms[i].pose).matrix();
            cg.add_edge(i - 1, i, measurement, odoms[i].information);
            cg.set_initial(i, odoms[i].pose.matrix());
        }
        // just like what is done in slam, every 3 or 4 step
        if (i % 3 == 0) {
            cg.optimize(5);
        }

        for (const auto& loop : loops) {
            if (i == loop.to) {
                cg.add_edge(loop.from, loop.to, loop.pose.matrix(), loop.information);
                cg.optimize(5);
            }
        }
    }

    cg.graph().saveGraph(path + "cg.dot", cg.result());

}