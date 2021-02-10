//
// Created by ziv on 2020/12/25.
//

#include "factors.h"
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/geometry/Pose3.h>

using gtsam::symbol_shorthand::E; //  edge factor
using gtsam::symbol_shorthand::P; // plane factor
using gtsam::symbol_shorthand::X; // state

int main()
{
    gtsam::NonlinearFactorGraph factors;
    gtsam::Values init_values;

    auto plane_plane_noise_model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(3) << 1e-2, 1e-2, 1e-3).finished());
    auto prior_gaussian_model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished());
    gtsam::Pose3 pose1(gtsam::Rot3::RzRyRx(1, 2, 3), gtsam::Vector3(1, 2, 3));
    gtsam::Pose3 pose2(gtsam::Rot3::RzRyRx(5, 6, 7), gtsam::Vector3(5, 6, 7));

    init_values.insert(X(0), pose1);
    init_values.insert(X(1), pose2);
    factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0), pose1, prior_gaussian_model);

    // make data
    std::vector<gtsam::OrientedPlane3> origin_planes;
    origin_planes.emplace_back(3, 4, 5, 6);
    origin_planes.emplace_back(4, 5, 6, 7);
    origin_planes.emplace_back(5, 6, 7, 8);
    origin_planes.emplace_back(6, 7, 8, 9);
    origin_planes.emplace_back(7, 8, 9, 10);
    std::cout << "gt: pose1" << std::endl;
    pose1.print();
    std::cout << "gt: pose2" << std::endl;
    pose2.print();

    for (int i = 0; i < origin_planes.size(); i++) {
        factors.emplace_shared<gtsam::PlaneToPlaneFactor>(X(0), origin_planes[i], X(1), origin_planes[i], plane_plane_noise_model);
    }

    gtsam::LevenbergMarquardtParams params;

    // solve
    auto result = gtsam::LevenbergMarquardtOptimizer(factors, init_values, params).optimize();

    // write result
    result.print("result:\n");

    return 0;
}