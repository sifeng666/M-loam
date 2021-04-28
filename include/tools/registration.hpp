//
// Created by ziv on 2021/2/2.
//

#ifndef MLOAM_REGISTRATION_H
#define MLOAM_REGISTRATION_H

#include "helper.h"
#include <pcl/features/fpfh.h>
#include <pcl/common/transforms.h>
#include <pcl/search/pcl_search.h>
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <pcl/registration/gicp.h>

namespace Eigen
{
    typedef Matrix<double, 6, 6> Matrix6d;
    typedef Matrix<double, 6, 1> Vector6d;
}

namespace tools
{
    using Feature = pcl::PointCloud<pcl::FPFHSignature33>;

/// \class RegistrationResult
///
/// Class that contains the registration results.
    class RegistrationResult {
    public:
        /// \brief Parameterized Constructor.
        ///
        /// \param transformation The estimated transformation matrix.
        RegistrationResult(
                const Eigen::Matrix4d &transformation = Eigen::Matrix4d::Identity())
                : transformation_(transformation), inlier_rmse_(0.0), fitness_(0.0) {}
        ~RegistrationResult() {}

    public:
        /// The estimated transformation matrix.
        Eigen::Matrix4d transformation_;
        /// Correspondence set between source and target point cloud.
        std::vector<Eigen::Vector2i> correspondence_set_;
        /// RMSE of all inlier correspondences. Lower is better.
        double inlier_rmse_;
        /// The overlapping area (# of inlier correspondences / # of points in
        /// target). Higher is better.
        double fitness_;
    };

    static RegistrationResult GetRegistrationResultAndCorrespondences(
            pcl::PointCloud<PointT>::Ptr source,
            pcl::PointCloud<PointT>::Ptr target,
            double max_correspondence_distance,
            const Eigen::Matrix4d &transformation) {
        RegistrationResult result(transformation);

        if (max_correspondence_distance <= 0.0) return result;

        double error2 = 0.0;

        pcl::search::KdTree<PointT>::Ptr target_kdtree(new pcl::search::KdTree<PointT>);
        target_kdtree->setInputCloud(target);

#ifdef _OPENMP
#pragma omp parallel
        {
#endif
            double error2_private = 0.0;
            std::vector<Eigen::Vector2i> correspondence_set_private;
#ifdef _OPENMP
#pragma omp for nowait
#endif
            for (int i = 0; i < (int)source->size(); i++) {
                std::vector<int> indices(1);
                std::vector<float> dists(1);
                const auto &point = source->points[i];

                if (target_kdtree->radiusSearch(point, max_correspondence_distance,
                                                indices, dists, 1) > 0) {
                    error2_private += dists[0];
                    correspondence_set_private.emplace_back(i, indices[0]);
                }
            }
#ifdef _OPENMP
#pragma omp critical
#endif
            {
                for (int i = 0; i < (int)correspondence_set_private.size(); i++) {
                    result.correspondence_set_.push_back(
                            correspondence_set_private[i]);
                }
                error2 += error2_private;
            }
#ifdef _OPENMP
        }
#endif

        if (result.correspondence_set_.empty())
        {
            result.fitness_ = 0.0;
            result.inlier_rmse_ = 0.0;
        }
        else
        {
            size_t corres_number = result.correspondence_set_.size();
            result.fitness_ = (double)corres_number / (double)source->size();
            result.inlier_rmse_ = std::sqrt(error2 / (double)corres_number);
        }
        return result;
    }

    static Eigen::Matrix6d GetInformationMatrixFromPointClouds(
            pcl::PointCloud<PointT>::Ptr source,
            pcl::PointCloud<PointT>::Ptr target,
            double max_correspondence_distance,
            const Eigen::Matrix4d &transformation)
    {
        pcl::PointCloud<PointT>::Ptr pcd(new pcl::PointCloud<PointT>);
        pcl::transformPointCloud(*source, *pcd, transformation);

        RegistrationResult result;

        result = GetRegistrationResultAndCorrespondences(
                pcd, target, max_correspondence_distance,
                transformation);

        // write q^*
        // see http://redwood-data.org/indoor/registration.html
        // note: I comes first in this implementation
        Eigen::Matrix6d GTG = Eigen::Matrix6d::Zero();
#ifdef _OPENMP
#pragma omp parallel
        {
#endif
            Eigen::Matrix6d GTG_private = Eigen::Matrix6d::Zero();
            Eigen::Vector6d G_r_private = Eigen::Vector6d::Zero();
#ifdef _OPENMP
#pragma omp for nowait
#endif
            for (int c = 0; c < int(result.correspondence_set_.size()); c++) {
                int t = result.correspondence_set_[c](1);
                const auto& p = target->points[t];

                G_r_private.setZero();
                G_r_private(1) = p.z;
                G_r_private(2) = -p.y;
                G_r_private(3) = 1.0;
                GTG_private.noalias() += G_r_private * G_r_private.transpose();
                G_r_private.setZero();
                G_r_private(0) = -p.z;
                G_r_private(2) = p.x;
                G_r_private(4) = 1.0;
                GTG_private.noalias() += G_r_private * G_r_private.transpose();
                G_r_private.setZero();
                G_r_private(0) = p.y;
                G_r_private(1) = -p.x;
                G_r_private(5) = 1.0;
                GTG_private.noalias() += G_r_private * G_r_private.transpose();
            }
#ifdef _OPENMP
#pragma omp critical
#endif
            { GTG += GTG_private; }
#ifdef _OPENMP
        }
#endif
        return GTG;
    }

    static bool FastGeneralizedRegistration(
            pcl::PointCloud<PointT>::Ptr source,
            pcl::PointCloud<PointT>::Ptr target,
            Eigen::Matrix4d& final,
            const Eigen::Matrix4d& init_guess = Eigen::Matrix4d::Identity(),
            double max_correspondence_distance = 0.5,
            double fitness_thres = 0.8) {

        fast_gicp::FastGICP<PointT, PointT> gicp;
        gicp.setNumThreads(0);
        gicp.setTransformationEpsilon(0.01);
        gicp.setMaximumIterations(64);
        gicp.setMaxCorrespondenceDistance(max_correspondence_distance);
        gicp.setCorrespondenceRandomness(20);

        // Align clouds
        gicp.setInputSource(source);
        gicp.setInputTarget(target);

        pcl::PointCloud<PointT>::Ptr unused_result(new pcl::PointCloud<PointT>());
        gicp.align(*unused_result, init_guess.cast<float>());

        if (!gicp.hasConverged()) {
            return false;
        }
        printf("fitness score: %f\n", gicp.getFitnessScore());

        if (gicp.getFitnessScore() > fitness_thres) {
            return false;
        }

        final = gicp.getFinalTransformation().cast<double>();
        return true;
    }

    static bool GeneralizedRegistration(
            pcl::PointCloud<PointT>::Ptr source,
            pcl::PointCloud<PointT>::Ptr target,
            Eigen::Matrix4d& final,
            const Eigen::Matrix4d& init_guess = Eigen::Matrix4d::Identity(),
            double max_correspondence_distance = 0.5,
            double fitness_thres = 0.8) {

        pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
        gicp.setTransformationEpsilon(0.01);
        gicp.setMaximumIterations(64);
        gicp.setMaxCorrespondenceDistance(max_correspondence_distance);
        gicp.setUseReciprocalCorrespondences(false);
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

        if (gicp.getFitnessScore() > fitness_thres) {
            return false;
        }

        final = gicp.getFinalTransformation().cast<double>();
        return true;
    }
}


#endif //MLOAM_REGISTRATION_H
