//
// Created by ziv on 2020/12/7.
//

#ifndef MLOAM_MAP_GENERATOR_H
#define MLOAM_MAP_GENERATOR_H

#include "keyframe.h"
#include <pcl/octree/octree_search.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class MapGenerator {
private:
    using ConstIter = std::vector<Keyframe::Ptr>::const_iterator;
    pcl::octree::OctreePointCloud<PointT> octree;
public:
    MapGenerator(double resolution = 0.1);
    ~MapGenerator();
    void clear();
    void insert(ConstIter begin, ConstIter end);
    pcl::PointCloud<PointT>::Ptr get() const;
};



#endif //MLOAM_MAP_GENERATOR_H
