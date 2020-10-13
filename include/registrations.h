//
// Created by ziv on 2020/10/12.
//

#ifndef MLOAM_REGISTRATIONS_H
#define MLOAM_REGISTRATIONS_H

#include "helper.h"
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>

#include <pclomp/ndt_omp.h>
#include <pclomp/gicp_omp.h>



class Registration {
public:
    using pclRegPtr = boost::shared_ptr<pcl::Registration<PointT, PointT>>;

    static void set_pcl_icp_reg();

    static void set_pcl_gicp_reg();

    static void set_gicp_omp_reg();

    static void set_pcl_ndt_reg();

    static void set_ndt_omp_reg();

    static void set_ndt_omp_reg(string nn_search_method);

    static pclRegPtr getRegistration() {
        if (!is_init) {
            set_ndt_omp_reg();
        }
        return registration;
    }

private:
    inline static bool is_init = false;
    inline static pclRegPtr registration;
};


void Registration::set_pcl_icp_reg() {
    std::cout << "registration: ICP" << std::endl;
    boost::shared_ptr<pcl::IterativeClosestPoint<PointT, PointT>> icp(new pcl::IterativeClosestPoint<PointT, PointT>());
    icp->setTransformationEpsilon(0.01);
    icp->setMaximumIterations(64);
    icp->setUseReciprocalCorrespondences(false);
    is_init = true;
    registration = icp;
}

void Registration::set_pcl_gicp_reg() {
    std::cout << "registration: GICP" << std::endl;
    boost::shared_ptr<pcl::GeneralizedIterativeClosestPoint<PointT, PointT>> gicp(new pcl::GeneralizedIterativeClosestPoint<PointT, PointT>());
    gicp->setTransformationEpsilon(0.01);
    gicp->setMaximumIterations(64);
    gicp->setUseReciprocalCorrespondences(false);
    gicp->setCorrespondenceRandomness(20);
    gicp->setMaximumOptimizerIterations(20);
    is_init = true;
    registration = gicp;
}

void Registration::set_gicp_omp_reg() {
    std::cout << "registration: GICP_OMP" << std::endl;
    boost::shared_ptr<pclomp::GeneralizedIterativeClosestPoint<PointT, PointT>> gicp(new pclomp::GeneralizedIterativeClosestPoint<PointT, PointT>());
    gicp->setTransformationEpsilon(0.01);
    gicp->setMaximumIterations(64);
    gicp->setUseReciprocalCorrespondences(false);
    gicp->setCorrespondenceRandomness(20);
    gicp->setMaximumOptimizerIterations(20);
    is_init = true;
    registration = gicp;
}

void Registration::set_pcl_ndt_reg() {
    double ndt_resolution = 0.5;
    std::cout << "registration: NDT " << ndt_resolution << std::endl;
    boost::shared_ptr<pcl::NormalDistributionsTransform<PointT, PointT>> ndt(new pcl::NormalDistributionsTransform<PointT, PointT>());
    ndt->setTransformationEpsilon(0.01);
    ndt->setMaximumIterations(64);
    ndt->setResolution(ndt_resolution);
    is_init = true;
    registration = ndt;
}

void Registration::set_ndt_omp_reg() {
    int num_threads = 0;
    double ndt_resolution = 0.5;
    std::string nn_search_method = "DIRECT7";
    std::cout << "registration: NDT_OMP " << nn_search_method << " " << ndt_resolution << " (" << num_threads << " threads)" << std::endl;
    boost::shared_ptr<pclomp::NormalDistributionsTransform<PointT, PointT>> ndt(new pclomp::NormalDistributionsTransform<PointT, PointT>());
    if(num_threads > 0) {
        ndt->setNumThreads(num_threads);
    }
    ndt->setTransformationEpsilon(0.01);
    ndt->setMaximumIterations(64);
    ndt->setResolution(ndt_resolution);
    ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7); // default DIRECT7
    is_init = true;
    registration = ndt;
}

void Registration::set_ndt_omp_reg(string nn_search_method) {
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
    is_init = true;
    registration = ndt;
}


#endif //MLOAM_REGISTRATIONS_H
