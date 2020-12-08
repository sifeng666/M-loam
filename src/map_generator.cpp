//
// Created by ziv on 2020/12/7.
//

#include "map_generator.h"



MapGenerator::MapGenerator(double resolution) : octree(resolution) {}

MapGenerator::~MapGenerator() {}

void MapGenerator::clear() {
    octree.deleteTree();
}

void MapGenerator::insert(ConstIter begin, ConstIter end) {

    if (begin >= end) {
        return;
    }

    pcl::PointCloud<PointT>::Ptr edge(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr surf(new pcl::PointCloud<PointT>());
    edge->reserve((*begin)->edgeFeatures->size() * 2);
    surf->reserve((*begin)->surfFeatures->size() * 2);

    for (auto iter = begin; iter != end; ++iter) {
        auto keyframe = *iter;
        edge->clear(); surf->clear();
        Eigen::Matrix4f pose = keyframe->pose.matrix().cast<float>();

        // EDGE
        for(const auto& src_pt : keyframe->edgeFeatures->points) {
            PointT dst_pt;
            dst_pt.getVector4fMap() = pose * src_pt.getVector4fMap();
            dst_pt.intensity = src_pt.intensity;
            edge->push_back(dst_pt);
        }
        edge->width = edge->size();
        edge->height = 1;
        edge->is_dense = false;
        octree.setInputCloud(edge);
        octree.addPointsFromInputCloud();

        // SURFACE
        for(const auto& src_pt : keyframe->surfFeatures->points) {
            PointT dst_pt;
            dst_pt.getVector4fMap() = pose * src_pt.getVector4fMap();
            dst_pt.intensity = src_pt.intensity;
            surf->push_back(dst_pt);
        }
        surf->width = surf->size();
        surf->height = 1;
        surf->is_dense = false;
        octree.setInputCloud(surf);
        octree.addPointsFromInputCloud();
    }

}

pcl::PointCloud<PointT>::Ptr MapGenerator::get() const {
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    octree.getOccupiedVoxelCenters(filtered->points);
    if (filtered->empty()) {
        return nullptr;
    }
    filtered->width = filtered->size();
    filtered->height = 1;
    filtered->is_dense = false;
    return filtered;
}

