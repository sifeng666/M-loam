//
// Created by ziv on 2020/12/7.
//

#ifndef MLOAM_MAP_GENERATOR_H
#define MLOAM_MAP_GENERATOR_H

#include "keyframe.h"
#include <pcl/octree/octree_search.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class MapGenerator {
private:
    pcl::PointCloud<PointT>::Ptr map;
public:
    MapGenerator();

    ~MapGenerator();

    void clear();

    void insert(KeyframeVec::Ptr keyframeVec, size_t begin, size_t end);

    pcl::PointCloud<PointT>::Ptr get(float) const;

    static pcl::PointCloud<PointT>::Ptr generate_cloud(KeyframeVec::Ptr keyframeVec, size_t begin, size_t end, FeatureType featureType);

};



#endif //MLOAM_MAP_GENERATOR_H
