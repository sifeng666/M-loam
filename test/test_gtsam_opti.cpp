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
#include "helper.h"

#include <gtsam/nonlinear/ISAM2.h>

using namespace std;
using namespace gtsam;
using namespace ct;


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

    if (argc != 2)
    {
        std::cerr << "./test_gtsam_opti /path/to/txt_file" << std::endl;
        std::exit(-1);
    }

    string path = argv[1];
    if (path.back() != '/') path += '/';

    vector<Odom> odoms;
    vector<Loop> loops;

    read_from_file(path + "/pose_fixed_raw.txt", path + "/f_loop.txt", odoms, loops);

    /// iSAM params
    ISAM2Params isam2Params;
    isam2Params.relinearizeThreshold = 0.1;
    isam2Params.relinearizeSkip = 1;
    ISAM2 isam(isam2Params);

    /// noise models
    Vector6 diagonal; diagonal << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6;
    auto prior_model = noiseModel::Diagonal::Variances(diagonal);

    /// Factors container
    NonlinearFactorGraph new_factors;
    Values new_values, current_estimate;

    for (size_t i = 0; i < odoms.size(); i++)
    {
        if (i == 0) 
        {
            // add prior
            new_factors.addPrior<Pose3>(i, odoms[i].pose, prior_model);
            new_values.insert(i, odoms[i].pose);
        } 
        else 
        {
            // normal odom
            auto model = noiseModel::Gaussian::Information(odoms[i].information);
            new_factors.emplace_shared<BetweenFactor<Pose3>>(i-1, i, odoms[i-1].pose.between(odoms[i].pose), model);
            new_values.insert(i, odoms[i].pose);
        }

        bool has_loop = false;
        // If exist a loop?
        for (const auto& loop : loops) {
            if (i == loop.to)
            {
                // find a loop
                auto model = noiseModel::Gaussian::Information(loop.information);
                new_factors.emplace_shared<BetweenFactor<Pose3>>(loop.from, loop.to, loop.pose, model);

                has_loop = true;
            }
        }

        if (has_loop)
        {
            // save before optimization
            isam.getFactorsUnsafe().saveGraph(path + "isam_before_loop.dot", current_estimate);
            writeG2o(isam.getFactorsUnsafe(), current_estimate, path + "isam_before_loop.g2o");
        }

        // Do optimization
        isam.update(new_factors, new_values);
        isam.update();

        // update values
        current_estimate = isam.calculateEstimate();
        new_factors.resize(0);
        new_values.clear();
    }

    isam.getFactorsUnsafe().saveGraph(path + "isam_final.dot", current_estimate);
    writeG2o(isam.getFactorsUnsafe(), current_estimate, path + "isam_final.g2o");


    //////////////////////////////////////////////  Traditional BA  //////////////////////////////////////////////////


    CalibrationGraph cg = CalibrationGraph();

    // Add from file data
    for (size_t i = 0; i < odoms.size(); i++) {
        if (i == 0) {
            cg.set_initial(0, odoms[i].pose.matrix());
        } else {
            Eigen::Matrix4d measurement = odoms[i-1].pose.between(odoms[i].pose).matrix();
            cg.add_edge(i - 1, i, measurement, odoms[i].information);
            cg.set_initial(i, odoms[i].pose.matrix());
        }
    }

    cg.graph().saveGraph(path + "cg_before_odom_opti.dot");
    writeG2o(cg.graph(), cg.initial(), path + "cg_before_odom_opti.g2o");

    cg.optimize(5);

    cg.graph().saveGraph(path + "cg_after_odom_opti.dot", cg.result());
    writeG2o(cg.graph(), cg.result(), path + "cg_after_odom_opti_opti.g2o");

    // Add loop
    for (const auto& loop : loops) {
        cg.add_edge(loop.from, loop.to, loop.pose.matrix(), loop.information);
    }
    cg.optimize(5);

    cg.graph().saveGraph(path + "cg_after_add_loop_opti.dot", cg.result());
    writeG2o(cg.graph(), cg.result(), path + "cg_after_add_loop_opti.g2o");

}