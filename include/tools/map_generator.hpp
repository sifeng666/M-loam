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
    //生成地图
    class MapGenerator {
    private:
        mutable std::mutex mtx; //锁
        pcl::PointCloud<PointT>::Ptr map; //点云指针
        pcl::VoxelGrid<PointT> v; //体素下采样过滤器
    public:
        MapGenerator() : map(new pcl::PointCloud<PointT>())
        {

        }
        //加锁然后调用点云指针的clear函数
        void clear() {
            std::lock_guard<std::mutex> lg(mtx);
            map->clear();
        }
        //插入
        void insert(KeyframeVec::Ptr keyframeVec, size_t begin, size_t end) {
            std::lock_guard<std::mutex> lg(mtx);
            if (begin >= end) {
                return;
            }
            //map为空指针，则先扩充map的空间
            if (map->empty()) {
                map->reserve(keyframeVec->keyframes[begin]->raw->size() * (end - begin));
            }

            // read lock，带锁将给定范围内的位姿读取到vector中
            std::vector<gtsam::Pose3> poseVec = keyframeVec->read_poses(begin, end, true);
            //遍历所有帧
            for (size_t i = 0; i < poseVec.size(); i++) {
                //读取关键帧
                Keyframe::Ptr keyframe = keyframeVec->keyframes[i + begin];
                //转换成位姿
                Eigen::Affine3f pose(Eigen::Matrix4f(poseVec[i].matrix().cast<float>()));
                //对所有的点
                for(const auto& src_pt : keyframe->raw->points) {
                    //插入到map中
                    map->push_back(pcl::transformPoint(src_pt, pose));
                }
            }
        }

        //
        pcl::PointCloud<PointT>::Ptr get(float resolution = 0.0f) {
            std::lock_guard<std::mutex> lg(mtx);
            pcl::PointCloud<PointT>::Ptr map_out;
            map_out = map->makeShared();

            v.setInputCloud(map_out);
            v.filter(*map_out); //降采样?
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
            if (featureType == FeatureType::Corn) {
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

                if (featureType == FeatureType::Corn) {
                    for(const auto& src_pt : keyframe->corn_features->points) {
                        cloud->push_back(pcl::transformPoint(src_pt, pose));
                    }
                }
                else if (featureType == FeatureType::Surf) {
                    for(const auto& src_pt : keyframe->surf_features->points) {
                        cloud->push_back(pcl::transformPoint(src_pt, pose));
                    }
                } else {
                    for(const auto& src_pt : keyframe->raw->points) {
                        cloud->push_back(pcl::transformPoint(src_pt, pose));
                    }
                }
            }

            cloud->is_dense = false;
            return cloud;
        }

    };
}





#endif //MLOAM_MAP_GENERATOR_H
