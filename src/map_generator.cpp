//
// Created by ziv on 2020/12/7.
//

#include "map_generator.h"



MapGenerator::MapGenerator() : map(new pcl::PointCloud<PointT>()) {
    voxelGrid.setLeafSize(0.01f, 0.01f, 0.01f);
}

MapGenerator::~MapGenerator() {}

void MapGenerator::clear() {
    map->clear();
}

void MapGenerator::insert(std::vector<Keyframe::Ptr>& keyframes, int begin, int end) {
    if (begin >= end) {
        return;
    }
    if (map->empty()) {
        map->reserve(keyframes[begin]->raw->size() * (end - begin));
    }

    for (int i = begin; i < end; i++) {
        auto keyframe = keyframes[i];
        Eigen::Matrix4f pose = keyframe->pose.matrix().cast<float>();

        for(const auto& src_pt : keyframe->raw->points) {
            PointT dst_pt;
            dst_pt.getVector4fMap() = pose * src_pt.getVector4fMap();
            dst_pt.intensity = src_pt.intensity;
            map->push_back(dst_pt);
        }
        voxelGrid.setInputCloud(map);
        voxelGrid.filter(*map);
    }
}

pcl::PointCloud<PointT>::Ptr MapGenerator::get() const {
    return map;
}

