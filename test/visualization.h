#pragma once

#include <memory>
#include <utility>
#include <vector>
#include <random>

#include "point_cloud.h"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/poisson.h>


    pcl::PolygonMesh::Ptr GenerateMesh(std::shared_ptr<PointCloud> cloud)
    {
        pcl::Poisson<pcl::PointNormal> pn;
        pn.setConfidence(false);
        pn.setDegree(2);
        pn.setDepth(8);
        pn.setIsoDivide(8);
        pn.setManifold(false);
        pn.setOutputPolygons(false);
        pn.setSamplesPerNode(3.0);
        pn.setScale(1.25);
        pn.setSolverDivide(8);
        
        pn.setSearchMethod(cloud->GetKDTree());
        pn.setInputCloud(cloud->points);

        pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
        pn.performReconstruction(*mesh);
        return mesh;
    }


    void BlockDrawPointClouds(const std::vector<std::shared_ptr<PointCloud>>& clouds)
    {
        pcl::visualization::PCLVisualizer visualizer;
        visualizer.setBackgroundColor(1, 1, 1);

        int counter = 0;
        for (auto& cloud: clouds)
        {
            if (false)
            {
                auto mesh = GenerateMesh(cloud);
                pcl::visualization::PointCloudColorHandlerCustom<PointCloud::PointT> color_handler
                        (cloud->points,cloud->color.r,cloud->color.g,cloud->color.b);
                visualizer.addPolygonMesh(*mesh, "polygon" + std::to_string(counter++));
            }
            else
            {
                const std::string vis_name = "cloud"+std::to_string(counter++);
                pcl::visualization::PointCloudColorHandlerCustom<PointCloud::PointT> color_handler
                        (cloud->points,cloud->color.r,cloud->color.g,cloud->color.b);

                visualizer.addPointCloud(cloud->points, color_handler, vis_name);
                visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, cloud->point_size, vis_name);

            }

        }

        visualizer.spin();
        visualizer.close();
    }

    void BlockDrawPointCloud(std::shared_ptr<PointCloud> cloud)
    {
        BlockDrawPointClouds({cloud});
    }
