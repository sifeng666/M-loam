//
// Created by ziv on 2020/12/7.
//

#include "map_generator.h"



MapGenerator::MapGenerator() : map(new pcl::PointCloud<PointT>()) {}

MapGenerator::~MapGenerator() {}

void MapGenerator::clear() {
    map->clear();
}

void MapGenerator::insert(KeyframeVec::Ptr keyframeVec, size_t begin, size_t end) {

    if (begin >= end) {
        return;
    }

    if (map->empty()) {
        map->reserve(keyframeVec->keyframes[begin]->raw->size() * (end - begin));
    }

    // read lock
    std::vector<gtsam::Pose3> poseVec = keyframeVec->read_poses(begin, end);

    for (size_t i = 0; i < poseVec.size(); i++) {
        Keyframe::Ptr keyframe = keyframeVec->keyframes[i + begin];
        Eigen::Matrix4f pose = poseVec[i].matrix().cast<float>();

        for(const auto& src_pt : keyframe->raw->points) {
            PointT dst_pt;
            dst_pt.getVector4fMap() = pose * src_pt.getVector4fMap();
            dst_pt.intensity = src_pt.intensity;
            map->push_back(dst_pt);
        }
    }
}

pcl::PointCloud<PointT>::Ptr MapGenerator::get(float resolution = 0.0f) const {
    if (resolution == 0)
        return map;
    pcl::VoxelGrid<PointT> f;
    f.setLeafSize(resolution, resolution, resolution);
    f.setInputCloud(map);
    pcl::PointCloud<PointT>::Ptr map1(new pcl::PointCloud<PointT>());
    f.filter(*map1);
    return map1;
}

pcl::PointCloud<PointT>::Ptr MapGenerator::generate_cloud(KeyframeVec::Ptr keyframeVec, size_t begin, size_t end, FeatureType featureType) {

    if (begin >= end) {
        std::cerr << "generate_cloud warning: range err!!" << std::endl;
        return nullptr;
    }

    size_t size;
    if (featureType == FeatureType::Edge) {
        size = keyframeVec->keyframes[begin]->edgeFeatures->size() * (end - begin);
    } else if (featureType == FeatureType::Surf) {
        size = keyframeVec->keyframes[begin]->surfFeatures->size() * (end - begin);
    } else if (featureType == FeatureType::Plane){
        size = keyframeVec->keyframes[begin]->planes->size() * (end - begin);
    } else {
        size = (keyframeVec->keyframes[begin]->raw->size()) * (end - begin);
    }

    // read lock
    std::vector<gtsam::Pose3> poseVec = keyframeVec->read_poses(begin, end);

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    cloud->reserve(size);

    for (size_t i = 0; i < poseVec.size(); i++) {
        Keyframe::Ptr keyframe = keyframeVec->keyframes[i + begin];
        Eigen::Matrix4f pose = poseVec[i].matrix().cast<float>();

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
        } else if (featureType == FeatureType::Plane) {
            for(const auto& src_pt : keyframe->planes->points) {
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



