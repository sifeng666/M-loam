//
// Created by ziv on 2020/10/13.
//

#ifndef MLOAM_PCL_UTILS_H
#define MLOAM_PCL_UTILS_H

#include "helper.h"

#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pclomp/ndt_omp.h>
#include <pclomp/gicp_omp.h>





class Registration {
public:

    static pcl::Registration<PointT, PointT>::Ptr getICPRegPCL() {
        std::cout << "registration: ICP" << std::endl;
        pcl::IterativeClosestPoint<PointT, PointT>::Ptr icp(new pcl::IterativeClosestPoint<PointT, PointT>());
        icp->setTransformationEpsilon(0.01);
        icp->setMaximumIterations(64);
        icp->setUseReciprocalCorrespondences(false);
        return icp;
    }

    static pcl::Registration<PointT, PointT>::Ptr getGICPRegPCL() {
        std::cout << "registration: GICP" << std::endl;
        pcl::GeneralizedIterativeClosestPoint<PointT, PointT>::Ptr gicp(new pcl::GeneralizedIterativeClosestPoint<PointT, PointT>());
        gicp->setTransformationEpsilon(0.01);
        gicp->setMaximumIterations(64);
        gicp->setUseReciprocalCorrespondences(false);
        gicp->setCorrespondenceRandomness(20);
        gicp->setMaximumOptimizerIterations(20);
        return gicp;
    }

    static pcl::Registration<PointT, PointT>::Ptr getGICPRegOMP() {
        std::cout << "registration: GICP_OMP" << std::endl;
        pclomp::GeneralizedIterativeClosestPoint<PointT, PointT>::Ptr gicp(new pclomp::GeneralizedIterativeClosestPoint<PointT, PointT>());
        gicp->setTransformationEpsilon(0.01);
        gicp->setMaximumIterations(64);
        gicp->setUseReciprocalCorrespondences(false);
        gicp->setCorrespondenceRandomness(20);
        gicp->setMaximumOptimizerIterations(20);
        return gicp;
    }

    static pcl::Registration<PointT, PointT>::Ptr getNDTRegPCL() {
        double ndt_resolution = 0.5;
        std::cout << "registration: NDT " << ndt_resolution << std::endl;
        boost::shared_ptr<pcl::NormalDistributionsTransform<PointT, PointT>> ndt(new pcl::NormalDistributionsTransform<PointT, PointT>());
        ndt->setTransformationEpsilon(0.01);
        ndt->setMaximumIterations(64);
        ndt->setResolution(ndt_resolution);
        return ndt;
    }

    static pcl::Registration<PointT, PointT>::Ptr getNDTRegOMP(string nn_search_method = "DIRECT7") {
        int num_threads = 0;
        double ndt_resolution = 0.5;
        std::cout << "registration: NDT_OMP " << nn_search_method << " " << ndt_resolution << " (" << num_threads << " threads)" << std::endl;
        boost::shared_ptr<pclomp::NormalDistributionsTransform<PointT, PointT>> ndt(new pclomp::NormalDistributionsTransform<PointT, PointT>());
        if(num_threads > 0) {
            ndt->setNumThreads(num_threads);
        }
        ndt->setTransformationEpsilon(0.01);
        ndt->setMaximumIterations(64);
        ndt->setResolution(ndt_resolution);
        if(nn_search_method == "KDTREE") {
            ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);
        } else if(nn_search_method == "DIRECT1") {
            ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);
        } else {
            ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
        }
        return ndt;
    }
};

class Filter {
public:

    static pcl::Filter<PointT>::Ptr getDownsampleFilter(const string& downsample_method, double downsample_resolution = 0.1) {
        pcl::Filter<PointT>::Ptr downsample_filter;
        if(downsample_method == "VOXELGRID") {
            std::cout << "downsample: VOXELGRID " << downsample_resolution << std::endl;
            boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
            voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
            downsample_filter = voxelgrid;
        } else if(downsample_method == "APPROX_VOXELGRID") {
            std::cout << "downsample: APPROX_VOXELGRID " << downsample_resolution << std::endl;
            boost::shared_ptr<pcl::ApproximateVoxelGrid<PointT>> approx_voxelgrid(new pcl::ApproximateVoxelGrid<PointT>());
            approx_voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
            downsample_filter = approx_voxelgrid;
        } else {
            if(downsample_method != "NONE") {
                std::cerr << "warning: unknown downsampling type (" << downsample_method << ")" << std::endl;
                std::cerr << "       : use passthrough filter" << std::endl;
            }
            std::cout << "downsample: NONE" << std::endl;
            downsample_filter = nullptr;
        }
        return downsample_filter;
    }

    static pcl::Filter<PointT>::Ptr getOutlierRemovalFilter(const string& outlier_removal_method) {
        pcl::Filter<PointT>::Ptr outlier_removal_filter;
        if(outlier_removal_method == "STATISTICAL") {
            int mean_k = 30;
            double stddev_mul_thresh = 1.2;
            std::cout << "outlier_removal: STATISTICAL " << mean_k << " - " << stddev_mul_thresh << std::endl;

            pcl::StatisticalOutlierRemoval<PointT>::Ptr sor(new pcl::StatisticalOutlierRemoval<PointT>());
            sor->setMeanK(mean_k);
            sor->setStddevMulThresh(stddev_mul_thresh);
            outlier_removal_filter = sor;
        } else if(outlier_removal_method == "RADIUS") {
            double radius = 0.5;
            int min_neighbors = 2;
            std::cout << "outlier_removal: RADIUS " << radius << " - " << min_neighbors << std::endl;

            pcl::RadiusOutlierRemoval<PointT>::Ptr rad(new pcl::RadiusOutlierRemoval<PointT>());
            rad->setRadiusSearch(radius);
            rad->setMinNeighborsInRadius(min_neighbors);
            outlier_removal_filter = rad;
        } else {
            std::cout << "outlier_removal: NONE" << std::endl;
            outlier_removal_filter = nullptr;
        }
        return outlier_removal_filter;
    }
};


#endif //MLOAM_PCL_UTILS_H
