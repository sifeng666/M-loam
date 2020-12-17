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

pcl::PointCloud<PointT>::Ptr MapGenerator::generate_cloud(const vector<Keyframe::Ptr> &keyframes, int begin, int end, FeatureType featureType) {
    if (begin >= end) {
        std::cerr << "generate_cloud warning: range err!!" << std::endl;
        return nullptr;
    }

    size_t size;
    if (featureType == FeatureType::Edge) {
        size = keyframes[begin]->edgeFeatures->size() * (end - begin);
    } else if (featureType == FeatureType::Surf) {
        size = keyframes[begin]->surfFeatures->size() * (end - begin);
    } else {
        size = (keyframes[begin]->raw->size()) * (end - begin);
    }

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    cloud->reserve(size);

    for (int i = begin; i < end; i++) {
        auto keyframe = keyframes[i];
        Eigen::Matrix4f pose = keyframe->pose.matrix().cast<float>();

        if (featureType == FeatureType::Edge) {
            for(const auto& src_pt : keyframe->edgeFeatures->points) {
                PointT dst_pt;
                dst_pt.getVector4fMap() = pose * src_pt.getVector4fMap();
                dst_pt.intensity = src_pt.intensity;
                cloud->push_back(dst_pt);
            }
        }
        else if (featureType == FeatureType::Surf) {
            for(const auto& src_pt : keyframe->surfFeatures->points) {
                PointT dst_pt;
                dst_pt.getVector4fMap() = pose * src_pt.getVector4fMap();
                dst_pt.intensity = src_pt.intensity;
                cloud->push_back(dst_pt);
            }
        } else {
            for(const auto& src_pt : keyframe->raw->points) {
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
    return cloud;
}



