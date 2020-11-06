#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/geometry/Pose3.h>

#include "factors.h"

using gtsam::symbol_shorthand::E; //  edge factor
using gtsam::symbol_shorthand::P; // plane factor
using gtsam::symbol_shorthand::X; // state

int main()
{
    std::cout << "gttt" << std::endl;
    gtsam::NonlinearFactorGraph factors;
    gtsam::Values init_values;

    auto pose_noise_model_plane = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(0.1));
    auto pose_noise_model_edge  = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.05, 0.05, 0.1));

    gtsam::Pose3 init_pose = gtsam::Pose3::identity();
    init_values.insert(X(0), init_pose);

    // make data
    gtsam::Pose3 pose_w_c(gtsam::Rot3::RzRyRx(0.3, -0.2, 0.3), gtsam::Vector3(-0.5, 0.3, 0.15));
    std::cout <<"gt: \n" << pose_w_c << std::endl;

    gtsam::Point3 p1(3,0,0);
    gtsam::Point3 p2(0,4,0);
    gtsam::Point3 p3(0,0,5);

    gtsam::Point3 s1(1.2, 0, 3.0);
    gtsam::Point3 s2(1.5, 2.0, 0);
    gtsam::Point3 s3(0, 0.8, 4.0);

    // transform
    s1 = pose_w_c.inverse() * s1;
    s2 = pose_w_c.inverse() * s2;
    s3 = pose_w_c.inverse() * s3;


    factors.emplace_shared<gtsam::PointToEdgeFactor>(
            X(0), s2, p1, p2, pose_noise_model_edge);

    factors.emplace_shared<gtsam::PointToEdgeFactor>(
            X(0), s3, p2, p3, pose_noise_model_edge);

    factors.emplace_shared<gtsam::PointToEdgeFactor>(
            X(0), s1, p3, p1, pose_noise_model_edge);

    gtsam::LevenbergMarquardtParams params;

    // solve
    auto result = gtsam::LevenbergMarquardtOptimizer(factors, init_values, params).optimize();

    // write result
    result.print("result:\n");

    return 0;
}