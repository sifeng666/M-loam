#include <thread>

#include "helper.h"
#include "point_cloud.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>


    PointCloud::PointCloud(): kdtree_cache_{nullptr}, has_estimate_normals_{false},
                  resolution_{0.0}, points{nullptr}, point_size{1}, normal_radius_(0.0) {}

    pcl::search::KdTree<PointCloud::PointT>::Ptr PointCloud::GetKDTree()
    {
        if (!points) return nullptr;

        {
            std::lock_guard<std::mutex> lock(kd_tree_mutex_);
            if (!kdtree_cache_)
            {
                kdtree_cache_.reset(new pcl::search::KdTree<PointCloud::PointT>);
                kdtree_cache_->setInputCloud(points);
            }
        }

        return kdtree_cache_;
    }

    void PointCloud::ComputeCloudResolution()
    {
        if (!points) return;

        std::cout << "computing cloud resolution for " << points->size();

        TicToc tic; tic.tic();
        resolution_ = 0;

        int numberOfPoints = 0, nres;
        std::vector<int> indices(2);
        std::vector<float> squaredDistances(2);

        auto kd_tree = GetKDTree();

        #pragma omp parallel reduction(+:resolution_)
        for (size_t i = 0; i < points->size(); ++i)
        {
            auto& p = points->points[i];
            if (!pcl::isFinite(p))
                continue;

            // Considering the second neighbor since the first is the point itself.
            nres = kd_tree->nearestKSearch(p, 2, indices, squaredDistances);
            if (nres == 2)
            {
                resolution_ += std::sqrt(squaredDistances[1]);
                ++numberOfPoints;
            }
        }

        if (numberOfPoints != 0)
            resolution_ /= numberOfPoints;

        std::cout << ", cost " <<  tic.toc() << " ms.\n";
    }

    PointCloud::Ptr
    PointCloud::VoxelDownSample(double voxel_size)
    {
        std::cout << "performing VoxelDownSample\n";

        TicToc tic; tic.tic();

        if (this->points == nullptr) return nullptr;

        PointCloud::Ptr ret (new PointCloud);
        ret->points.reset(new pcl::PointCloud<PointCloud::PointT>);

        pcl::VoxelGrid<PointCloud::PointT> filter;
        filter.setLeafSize(float(voxel_size), float(voxel_size), float(voxel_size));
        filter.setInputCloud(this->points);
        filter.setDownsampleAllData(false);
        filter.filter(*ret->points);

        std::cout << "Before: " << this->points->size() <<" points, After: "
            <<  ret->points->size() <<  ", cost " <<  tic.toc() << " ms.\n";

        return ret;
    }

    PointCloud::Ptr
    PointCloud::Transformed(const Eigen::Matrix4d& transformation)
    {
        if (points == nullptr) return nullptr;

        PointCloud::Ptr ret (new PointCloud);
        ret->points.reset(new pcl::PointCloud<PointCloud::PointT>);

        pcl::transformPointCloud(*points, *ret->points, transformation, false);

        ret->resolution_ = this->resolution_;

        /// ! normals and colors are not set now
        return ret;
    }

    void PointCloud::Transform(const Eigen::Matrix4d& transformation)
    {
        if (points == nullptr) return;

        pcl::transformPointCloud(*points, *points, transformation, false);

        //! normals has to be estimated again
        has_estimate_normals_ = false;

        //! kdtree also
        kdtree_cache_ = nullptr;
    }

    void PointCloud::DrawColor(const Eigen::Vector3d& color)
    {
        this->color.r = std::fmod(color.x()*255, 256);
        this->color.g = std::fmod(color.y()*255, 256);
        this->color.b = std::fmod(color.z()*255, 256);
    }

    bool PointCloud::LoadFromFile(std::string filename)
    {
        std::ifstream ifs_file(filename);
        if (!ifs_file.is_open())
        {
            return false;
        }

        std::cout << "LoadFromFile: " << filename << std::endl;

        TicToc tic; tic.tic();

        points.reset(new pcl::PointCloud<PointCloud::PointT>);
        if (pcl::io::loadPCDFile(filename, *points) == -1)
        {
            return false;
        }

        std::cout << "LoadFromFile cost " <<  tic.toc() << " ms.\n";

        return true;
    }

    void PointCloud::EstimateNormals(double radius_normal)
    {
        {
            std::lock_guard<std::mutex> lock(normals_mutex_);
            if (points && !has_estimate_normals_)
            {
                std::cout << "EstimateNormals for " << points->size() <<" points, ";

                TicToc tic; tic.tic();

                if (std::abs(normal_radius_-radius_normal)>1e-4)
                {
                    pcl::NormalEstimationOMP<PointCloud::PointT, PointCloud::PointT>
                            norm_est(std::thread::hardware_concurrency());

                    norm_est.setRadiusSearch(radius_normal);
                    norm_est.setInputCloud(points);
                    norm_est.compute (*points);

                    normal_radius_ = radius_normal;
                }

                has_estimate_normals_ = true;

                std::cout << "cost " <<  tic.toc() << " ms.\n";
            }
            else normal_radius_ = 0.0; ///! invalid flag
        }
    }

    double PointCloud::GetResolution()
    {
        {
            std::lock_guard<std::mutex> lock(resolution_mutex_);
            if (resolution_ == 0)
            {
                ComputeCloudResolution();
            }
        }

        return resolution_;
    }

    PointCloud::Ptr PointCloud::Clone(bool clone_all_field) const
    {
        PointCloud::Ptr ret(new PointCloud);

        // x, y, z and normals
        ret->points = this->points->makeShared();
        ret->has_estimate_normals_ = this->has_estimate_normals_;
        ret->resolution_ = ret->resolution_;
        ret->normal_radius_ = ret->normal_radius_;

        if (clone_all_field)
        {
            ret->color = this->color;
            ret->point_size = ret->point_size;
            if (kdtree_cache_) *(ret->kdtree_cache_) = *kdtree_cache_;
        }

        return ret;
    }

    void PointCloud::Concat(const PointCloud::ConstPtr& rhs)
    {
        if (points == nullptr)
        {
            // copy
            points = rhs->points->makeShared();
        }
        else
        {
            *points += *rhs->points;
            has_estimate_normals_ = false;
            kdtree_cache_ = nullptr;
            resolution_ = 0;
        }
    }

    bool PointCloud::SaveToFile(std::string filename)
    {
        if (pcl::io::savePCDFileBinaryCompressed(filename, *points) == -1)
        {
            return false;
        }
        return true;
    }

