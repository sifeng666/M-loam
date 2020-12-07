//
// Created by ziv on 2020/12/7.
//

#ifndef MLOAM_MAP_GENERATOR_H
#define MLOAM_MAP_GENERATOR_H

//#include "helper.h"
#include "frame.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class MapGenerator {
public:
    MapGenerator();
    ~MapGenerator();

    pcl::PointCloud<PointT>::Ptr generate(const std::vector<Keyframe::Ptr>& keyframes, double resolution) const;
};



#endif //MLOAM_MAP_GENERATOR_H
