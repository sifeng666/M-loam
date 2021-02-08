//
// Created by ziv on 2020/12/7.
//

#ifndef MLOAM_MAP_GENERATOR_H
#define MLOAM_MAP_GENERATOR_H

#include "tools/keyframe.hpp"
#include "balmclass.hpp"
#include <pcl/octree/octree_search.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

namespace tools
{
    class MapGenerator {
    private:
        mutable std::mutex mtx;
        pcl::PointCloud<PointT>::Ptr map;
        pcl::VoxelGrid<PointT> v;
    public:
        MapGenerator() : map(new pcl::PointCloud<PointT>())
        {

        }

        void clear() {
            std::lock_guard<std::mutex> lg(mtx);
            map->clear();
        }

        void insert(KeyframeVec::Ptr keyframeVec, size_t begin, size_t end) {
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

        pcl::PointCloud<PointT>::Ptr get(float resolution = 0.0f) {
            std::lock_guard<std::mutex> lg(mtx);
            pcl::PointCloud<PointT>::Ptr map_out;
            map_out = map->makeShared();

            v.setInputCloud(map_out);
            v.filter(*map_out);
//            down_sampling_voxel(*map_out, resolution);
            return map_out;
        }

        static pcl::PointCloud<PointT>::Ptr generate_cloud(KeyframeVec::Ptr keyframeVec, int begin, int end,
                                                           FeatureType featureType, bool need_fixed = false) {
            if (!keyframeVec || begin >= end) {
//                std::cerr << "generate_cloud warning: range err!!" << std::endl;
                return nullptr;
            }
            begin = max(0, begin);

            size_t size;
            if (featureType == FeatureType::Edge) {
                size = keyframeVec->keyframes[begin]->corn_features->size() * (end - begin);
            } else if (featureType == FeatureType::Surf) {
                size = keyframeVec->keyframes[begin]->surf_features->size() * (end - begin);
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
                    for(const auto& src_pt : keyframe->corn_features->points) {
                        cloud->emplace_back(pcl::transformPoint(src_pt, pose));
                    }
                }
                else if (featureType == FeatureType::Surf) {
                    for(const auto& src_pt : keyframe->surf_features->points) {
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

    };
}





#endif //MLOAM_MAP_GENERATOR_H
