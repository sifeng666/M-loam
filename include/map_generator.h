//
// Created by ziv on 2020/12/7.
//

#ifndef MLOAM_MAP_GENERATOR_H
#define MLOAM_MAP_GENERATOR_H

#include "keyframe.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class MapGenerator {
public:
    using ConstIter = std::vector<Keyframe::Ptr>::const_iterator;
    MapGenerator();
    ~MapGenerator();

    pcl::PointCloud<PointT>::Ptr generate(const std::vector<Keyframe::Ptr>& keyframes, double resolution) const;
    pcl::PointCloud<PointT>::Ptr generate(const std::vector<Keyframe::Ptr>& keyframes, double resolution,
                                          ConstIter start, ConstIter end) const;
    pcl::PointCloud<PointT>::Ptr generate(const std::vector<Keyframe::Ptr>& keyframes, double resolution,
                                          ConstIter start, ConstIter end, FeatureType featureType) const;
};



#endif //MLOAM_MAP_GENERATOR_H
