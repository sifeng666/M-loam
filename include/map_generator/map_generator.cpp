//
// Created by ziv on 2020/12/7.
//

#include "map_generator.h"



MapGenerator::MapGenerator() : map(new pcl::PointCloud<PointT>()) {}

MapGenerator::~MapGenerator() {}

void MapGenerator::clear() {
    std::lock_guard<std::mutex> lg(mtx);
    map->clear();
}

void MapGenerator::insert(KeyframeVec::Ptr keyframeVec, size_t begin, size_t end) {
    std::lock_guard<std::mutex> lg(mtx);
    if (begin >= end) {
        return;
    }

    if (map->empty()) {
        map->reserve(keyframeVec->keyframes[begin]->raw->size() * (end - begin));
    }

    // read lock
    std::vector<gtsam::Pose3> poseVec = keyframeVec->read_poses(begin, end, true);

    for (size_t i = 0; i < poseVec.size(); i++) {
        Keyframe::Ptr keyframe = keyframeVec->keyframes[i + begin];
        Eigen::Affine3f pose(Eigen::Matrix4f(poseVec[i].matrix().cast<float>()));

        for(const auto& src_pt : keyframe->raw->points) {
            map->emplace_back(pcl::transformPoint(src_pt, pose));
        }
    }
}

pcl::PointCloud<PointT>::Ptr MapGenerator::get(float resolution) const {
    std::lock_guard<std::mutex> lg(mtx);
    pcl::PointCloud<PointT>::Ptr map_out;
    map_out = map->makeShared();
//    pcl::VoxelGrid<PointT> v;
//    v.setLeafSize(resolution, resolution, resolution);
//    v.setInputCloud(map_out);
//    v.filter(*map_out);
    down_sampling_voxel(*map_out, resolution);
    return map_out;
}

pcl::PointCloud<PointT>::Ptr MapGenerator::generate_cloud(KeyframeVec::Ptr keyframeVec, size_t begin, size_t end,
                                                          FeatureType featureType, bool need_fixed) {

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
    std::vector<gtsam::Pose3> poseVec = keyframeVec->read_poses(begin, end, need_fixed);

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    cloud->reserve(size);

    for (size_t i = 0; i < poseVec.size(); i++) {
        Keyframe::Ptr keyframe = keyframeVec->keyframes[i + begin];
        Eigen::Affine3f pose(Eigen::Matrix4f(poseVec[i].matrix().cast<float>()));

        if (featureType == FeatureType::Edge) {
            for(const auto& src_pt : keyframe->edgeFeatures->points) {
                cloud->emplace_back(pcl::transformPoint(src_pt, pose));
            }
        }
        else if (featureType == FeatureType::Surf) {
            for(const auto& src_pt : keyframe->surfFeatures->points) {
                cloud->emplace_back(pcl::transformPoint(src_pt, pose));
            }
        } else if (featureType == FeatureType::Plane) {
            for(const auto& src_pt : keyframe->planes->points) {
                cloud->emplace_back(pcl::transformPoint(src_pt, pose));
            }
        } else {
            for(const auto& src_pt : keyframe->raw->points) {
                cloud->emplace_back(pcl::transformPoint(src_pt, pose));
            }
        }
    }

    cloud->is_dense = false;
    return cloud;
}
