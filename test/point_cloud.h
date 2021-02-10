#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/pcl_search.h>

#include <memory>
#include <mutex>


    class PointCloud
    {
    public:

        using Ptr = std::shared_ptr<PointCloud>;
        using ConstPtr = std::shared_ptr<const PointCloud>;
        using PointT = pcl::PointNormal;

        int point_size;

        pcl::RGB color;

        pcl::PointCloud<PointT>::Ptr points;

        PointCloud();

        std::shared_ptr<PointCloud> VoxelDownSample(double voxel_size);

        std::shared_ptr<PointCloud> Transformed(const Eigen::Matrix4d& transformation);

        void Transform(const Eigen::Matrix4d& transformation);

        void DrawColor(const Eigen::Vector3d& color);

        bool LoadFromFile(std::string filename);

        bool SaveToFile(std::string filename);

        //! thread safe estimation of normals
        void EstimateNormals(double radius_normal = 0.2);

        bool HasNormal() const { return has_estimate_normals_;}

        //! thread safe getter of kdtree, lazy call
        pcl::search::KdTree<PointT>::Ptr GetKDTree();

        //! thread safe estimation of resolution of pointcloud, lazy call
        double GetResolution();

        Ptr Clone(bool clone_all_field = false) const;

        void Concat(const PointCloud::ConstPtr& rhs);

    private:

        // for multiple thread safe
        std::mutex normals_mutex_;
        std::mutex kd_tree_mutex_;
        std::mutex resolution_mutex_;

        pcl::search::KdTree<PointT>::Ptr kdtree_cache_;

        bool has_estimate_normals_;

        double resolution_;

        double normal_radius_;

        void ComputeCloudResolution();
    };