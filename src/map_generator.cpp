//
// Created by ziv on 2020/12/7.
//

#include "map_generator.h"

#include <pcl/octree/octree_search.h>

MapGenerator::MapGenerator() {}

MapGenerator::~MapGenerator() {}

pcl::PointCloud<PointT>::Ptr MapGenerator::generate(const std::vector<Keyframe::Ptr>& keyframes, double resolution,
                                      ConstIter start, ConstIter end, FeatureType featureType) const {

    if(keyframes.empty()) {
        std::cerr << "warning: keyframes empty!!" << std::endl;
        return nullptr;
    }

    if (start == keyframes.end() || start == end) {
        std::cerr << "warning: iter err!!" << std::endl;
        return nullptr;
    }

    size_t size = 0;
    if (featureType == FeatureType::Edge) {
        size = (*start)->edgeSlice->size() * (end - start);
    } else if (featureType == FeatureType::Surf) {
        size = (*start)->surfSlice->size() * (end - start);
    } else {
        size = ((*start)->surfSlice->size() + (*start)->edgeSlice->size()) * (end - start);
    }

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    cloud->reserve(size);

    for (auto iter = start; iter != end; ++iter) {
        auto keyframe = *iter;
        Eigen::Matrix4f pose = keyframe->pose.matrix().cast<float>();
        if (featureType == FeatureType::Edge || featureType == FeatureType::Full) {
            for(const auto& src_pt : keyframe->edgeSlice->points) {
                PointT dst_pt;
                dst_pt.getVector4fMap() = pose * src_pt.getVector4fMap();
                dst_pt.intensity = src_pt.intensity;
                cloud->push_back(dst_pt);
            }
        }
        if (featureType == FeatureType::Surf || featureType == FeatureType::Full) {
            for(const auto& src_pt : keyframe->surfSlice->points) {
                PointT dst_pt;
                dst_pt.getVector4fMap() = pose * src_pt.getVector4fMap();
                dst_pt.intensity = src_pt.intensity;
                cloud->push_back(dst_pt);
            }
        }
    }

    cloud->width = cloud->size();
    cloud->height = 1;
    cloud->is_dense = false;

    if (resolution <= 0.0)
        return cloud; // To get unfiltered point cloud with intensity

    pcl::octree::OctreePointCloud<PointT> octree(resolution);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    octree.getOccupiedVoxelCenters(filtered->points);

    filtered->width = filtered->size();
    filtered->height = 1;
    filtered->is_dense = false;

    return filtered;
}

pcl::PointCloud<PointT>::Ptr MapGenerator::generate(const std::vector<Keyframe::Ptr>& keyframes, double resolution, ConstIter start, ConstIter end) const {
    return generate(keyframes, resolution, start, end, FeatureType::Full);
}

pcl::PointCloud<PointT>::Ptr MapGenerator::generate(const std::vector<Keyframe::Ptr>& keyframes, double resolution) const {
    return generate(keyframes, resolution, keyframes.begin(), keyframes.end());
}

