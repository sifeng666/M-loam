#include "factors.h"
#include "point_cloud.h"
#include "visualization.h"
#include <fstream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

using namespace std;


int main()
{
    string root_path = "/home/ziv/debuging/";
    string sub_folder= "/499/";
    string time_folder = "/0/";
    string final_folder = root_path + sub_folder;

    // Read corresponding
    fstream if_edge (final_folder + time_folder +  "edge_correspondings.txt");
    fstream if_plane (final_folder + time_folder + "plane_correspondings.txt");
    fstream tmp_result (final_folder + time_folder + "tmp_result.txt");

    PointCloud::Ptr plane_feature(new PointCloud);
    plane_feature->LoadFromFile(final_folder + "plane_feature.pcd");

    PointCloud::Ptr  plane_submap(new PointCloud);;
    plane_submap->LoadFromFile(final_folder + "plane_submap.pcd");


    Eigen::Matrix<double, 4, 4, Eigen::RowMajor> ceres_result;

    // Read ceres result
    string temp_buffer;
    tmp_result >> temp_buffer;
    for (int i = 0; i < 16; ++i)
    {
        tmp_result >> *(ceres_result.data() + i);
    }
    ceres_result.bottomRows<1>().setZero();
    ceres_result(3, 3) = 1;
    cout << ceres_result << endl;

    struct EdgeCorr
    {
        gtsam::Point3 cur {};
        gtsam::Point3 a {};
        gtsam::Point3 b {};
    };

    // Read edge
    vector<EdgeCorr> e_vec;
    EdgeCorr e;
    while(if_edge >> e.cur[0] >> e.cur[1] >> e.cur[2] >>
                     e.a[0]   >> e.a[1]   >> e.a[2]   >>
                     e.b[0]   >> e.b[1]   >> e.b[2] )
    {
        e_vec.emplace_back(e);
    }

    cout << "e_vec.size(): " << e_vec.size() << endl;

    struct PlaneCorr
    {
        gtsam::Point3 cur {};
        Eigen::Vector3d norm {};
        double n_d_n {};
    };

    // Read plane
    vector<PlaneCorr> p_vec;
    PlaneCorr p;
    while(if_plane >> p.cur[0] >> p.cur[1] >> p.cur[2] >>
        p.cur[0] >> p.cur[1] >> p.cur[2] >> p.n_d_n )
    {
        p_vec.emplace_back(p);
    }

    cout << "p_vec.size(): " << p_vec.size() << endl;


    gtsam::NonlinearFactorGraph graph;

    auto edge_gaussian_model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(3) << 1.0, 1.0, 1.0).finished());
    auto surf_gaussian_model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(1) << 1.0).finished());

    auto edge_noise_model = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(0.1), edge_gaussian_model);
    auto surf_noise_model = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(0.1), surf_gaussian_model);


    using gtsam::symbol_shorthand::X;
    gtsam::Values initialEstimate;
    gtsam::Pose3 init_guess (ceres_result);
    initialEstimate.insert(X(0), init_guess);


    for (auto& e_c: e_vec)
    {
        graph.emplace_shared<gtsam::PointToEdgeFactor>(X(0), e_c.cur,
                                                       e_c.a, e_c.b,edge_noise_model);
    }
    for (auto& p_c: p_vec)
    {
        graph.emplace_shared<gtsam::PointToPlaneFactor>(X(0), p_c.cur,
                                                        p_c.norm, p_c.n_d_n, surf_noise_model);
    }


    gtsam::GaussNewtonParams parameters;
    parameters.verbosity = gtsam::GaussNewtonParams::ERROR;
    gtsam::GaussNewtonOptimizer optimizer(graph, initialEstimate, parameters);
    gtsam::Values result = optimizer.optimize();
    result.print("Final Result:\n");

    Eigen::Matrix4d T_s_f = result.at(X(0)).cast<gtsam::Pose3>().matrix();

    auto tran_plane_feature = plane_feature->Transformed(T_s_f);

    plane_feature->DrawColor({0, 0, 2});
    plane_submap->DrawColor({0.5, 0.5, 0.5});
    tran_plane_feature->DrawColor({0, 1, 0});

    plane_feature->point_size = 3;
    plane_submap->point_size = 3;
    tran_plane_feature->point_size = 3;

    BlockDrawPointClouds({plane_feature, plane_submap, tran_plane_feature});

}