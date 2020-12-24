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

    std::vector<gtsam::Pose3> poseVec;
    poseVec.reserve(end - begin);
    // shared_lock
    {
        keyframeVec->pose_mtx.lock_shared();
        for (size_t i = begin; i < end; i++) {
            poseVec.push_back(keyframeVec->keyframes[i]->pose_world_curr);
        }
        keyframeVec->pose_mtx.unlock_shared();
    }

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

pcl::PointCloud<PointT>::Ptr MapGenerator::get() const {
    return map;
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
    } else {
        size = (keyframeVec->keyframes[begin]->raw->size()) * (end - begin);
    }

    std::vector<gtsam::Pose3> poseVec;
    poseVec.reserve(end - begin);
    // shared_lock
    {
        keyframeVec->pose_mtx.lock_shared();
        for (size_t i = begin; i < end; i++) {
            poseVec.push_back(keyframeVec->keyframes[i]->pose_world_curr);
        }
        keyframeVec->pose_mtx.unlock_shared();
    }

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



