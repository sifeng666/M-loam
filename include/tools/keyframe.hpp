//
// Created by ziv on 2020/10/24.
//

#ifndef MLOAM_KEYFRAME_H
#define MLOAM_KEYFRAME_H

#include "helper.h"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/OrientedPlane3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <pcl/common/transforms.h>

using gtsam::symbol_shorthand::E; //  edge factor
using gtsam::symbol_shorthand::P; // plane factor
using gtsam::symbol_shorthand::X; // state

namespace tools
{
    enum class FeatureType {
        Corn = 0,
        Surf = 1,
        Full = 2,
        Plane = 3
    };

    class Frame {
    public:
        using Ptr = std::shared_ptr<Frame>;
        int index;
        ros::Time cloud_in_time;
        gtsam::Pose3 pose_w_curr;
        pcl::PointCloud<PointT>::Ptr corn_features;
        pcl::PointCloud<PointT>::Ptr surf_features;
        pcl::PointCloud<PointT>::Ptr raw;
        bool is_keyframe;
        Frame(int index_, const ros::Time time,
              const pcl::PointCloud<PointT>::Ptr& corn,
              const pcl::PointCloud<PointT>::Ptr& surf,
              const pcl::PointCloud<PointT>::Ptr& raw_,
              const gtsam::Pose3& pose,
              bool is_keyframe_ = false) :
              index(index_), cloud_in_time(time), corn_features(corn),
              surf_features(surf), raw(raw_), pose_w_curr(pose), is_keyframe(is_keyframe_) {}
    };

    class NScans {
    private:
        std::list<Frame::Ptr> scans;
        int n;
    public:

        NScans(int n_ = 6) : n(n_) {}

        void add(Frame::Ptr new_scan) {
            scans.push_front(new_scan);
            if (scans.size() > n) {
                scans.pop_back();
            }
        }

        void read(pcl::PointCloud<PointT>::Ptr& corn_scans, pcl::PointCloud<PointT>::Ptr& surf_scans) {
            assert(corn_scans && surf_scans);

            pcl::PointCloud<PointT>::Ptr corn_tmp(new pcl::PointCloud<PointT>);
            pcl::PointCloud<PointT>::Ptr surf_tmp(new pcl::PointCloud<PointT>);

            for (const auto& scan : scans) {
                pcl::transformPointCloud(*scan->corn_features, *corn_tmp, scan->pose_w_curr.matrix());
                pcl::transformPointCloud(*scan->surf_features, *surf_tmp, scan->pose_w_curr.matrix());
                *corn_scans += *corn_tmp;
                *surf_scans += *surf_tmp;
            }
        }
    };


    class Keyframe {
    public:
        using Ptr = std::shared_ptr<Keyframe>;
        using PointCloudPtr = pcl::PointCloud<PointT>::Ptr;
    public:
        int index;
        bool fixed = false;
        int valid_frames = 0;
        ros::Time cloud_in_time;
        gtsam::Pose3 pose_w_curr;
        // feature point cloud
        pcl::PointCloud<PointT>::Ptr corn_features;
        pcl::PointCloud<PointT>::Ptr surf_features;
        pcl::PointCloud<PointT>::Ptr raw;
    public:
        Keyframe(int index_, const ros::Time& time,
                 PointCloudPtr EF, PointCloudPtr PF, PointCloudPtr RAW) :
                index(index_), cloud_in_time(time),
                corn_features(EF), surf_features(PF), raw(RAW)
        {

        }
        inline void set_fixed(gtsam::Pose3 pose_w_curr_ = gtsam::Pose3()) {
            pose_w_curr = pose_w_curr_;
            fixed = true;
        }
        void set_init(gtsam::Pose3 pose_w_curr_ = gtsam::Pose3()) {
            pose_w_curr = pose_w_curr_;
            add_frame();
        }
        inline void fix() {
            fixed = true;
        }
        inline void add_frame() {
            ++valid_frames;
        }
        inline bool is_fixed() const {
            return fixed;
        }
        inline bool is_init() const {
            return valid_frames > 0;
        }

    };

    class KeyframeVec {
    public:
        using Ptr = std::shared_ptr<KeyframeVec>;
    public:
        mutable std::shared_mutex pose_mtx;
        std::vector<Keyframe::Ptr> keyframes;
    public:
        // read poses that are inited or fixed
        std::vector<gtsam::Pose3> read_poses(size_t begin, size_t end, bool need_fixed = false) const {
            if (begin >= end || end > keyframes.size()) {
//                std::cerr << "read_poses invalid range" << std::endl;
                return {};
            }

            std::vector<gtsam::Pose3> poseVec;

            std::shared_lock<std::shared_mutex> sl(pose_mtx);
            for (size_t i = begin; i < end; i++) {
                if (need_fixed) {
                    if (keyframes[i]->is_fixed()) {
                        poseVec.emplace_back(keyframes[i]->pose_w_curr);
                    }
                } else {
                    if (keyframes[i]->is_init()) {
                        poseVec.emplace_back(keyframes[i]->pose_w_curr);
                    }
                }
            }

            return poseVec;
        }
        // read pose that is inited, not require fixed
        gtsam::Pose3 read_pose(size_t index) const {
            if (index > keyframes.size()) {
                std::cerr << "read_pose invalid range" << std::endl;
                return gtsam::Pose3();
            }
            if(!keyframes[index]->is_init()) {
                std::cerr << "pose not init!" << std::endl;
                return gtsam::Pose3();
            }

            std::shared_lock<std::shared_mutex> sl(pose_mtx);
            return keyframes[index]->pose_w_curr;
        }

        inline size_t size() const {
            return keyframes.size();
        }
        inline Keyframe::Ptr& at(size_t index) {
            return this->keyframes.at(index);
        }
        inline const Keyframe::Ptr& at(size_t index) const {
            return this->keyframes.at(index);
        }
        inline void emplace_back(Keyframe::Ptr keyframe) {
            this->keyframes.emplace_back(keyframe);
        }
        inline void push_back(Keyframe::Ptr keyframe) {
            this->keyframes.push_back(keyframe);
        }
    };

}


#endif //MLOAM_KEYFRAME_H
