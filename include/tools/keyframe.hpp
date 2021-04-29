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

namespace tools {
    enum class FeatureType {
        Corn = 0,
        Surf = 1,
        Full = 2
    };

    enum class Type {
        Odom = 0,
        Opti = 1,
        Delta = 2
    };

    class Submap {
    public:
        using Ptr = std::shared_ptr<Submap>;
        mutable std::mutex mtx;
        int from, end;
        pcl::PointCloud<PointT>::Ptr submap_corn;
        pcl::PointCloud<PointT>::Ptr submap_surf;

        Submap() {
            submap_corn.reset(new pcl::PointCloud<PointT>);
            submap_surf.reset(new pcl::PointCloud<PointT>);
        }

        pcl::PointCloud<PointT>::Ptr merge() const {
            pcl::PointCloud<PointT>::Ptr ret(new pcl::PointCloud<PointT>);
            std::lock_guard lg(mtx);
            *ret += *submap_corn;
            *ret += *submap_surf;
            return ret;
        }

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

        Frame(int index_, const ros::Time time,
              const pcl::PointCloud<PointT>::Ptr &corn,
              const pcl::PointCloud<PointT>::Ptr &surf,
              const pcl::PointCloud<PointT>::Ptr &raw_,
              const gtsam::Pose3 &pose) :
                index(index_), cloud_in_time(time), corn_features(corn),
                surf_features(surf), raw(raw_), pose_w_curr(pose) {}
    };

    template<class T>
    class Channel {
    public:
        using Ptr = std::shared_ptr<Channel>;
    private:
        std::mutex mtx;
        std::queue<T> buf;
    public:
        inline void lock() {
            mtx.lock();
        }

        inline void unlock() {
            mtx.unlock();
        }

        inline void push(T frame) {
            lock();
            buf.push(frame);
            unlock();
        }

        inline void clear() {
            lock();
            while (!buf.empty()) {
                buf.pop();
            }
            unlock();
        }

        inline T get_front() {
            T t = nullptr;
            lock();
            if (!buf.empty()) {
                t = buf.front();
                buf.pop();
            }
            unlock();
            return t;
        }

        inline std::vector<T> get_all() {
            std::vector<T> vec;
            lock();
            while (!buf.empty()) {
                vec.push_back(buf.front());
                buf.pop();
            }
            unlock();
            return vec;
        }

        inline bool empty() {
            std::lock_guard lg(mtx);
            return buf.empty();
        }
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

        void read(pcl::PointCloud<PointT>::Ptr &corn_scans, pcl::PointCloud<PointT>::Ptr &surf_scans) {
            assert(corn_scans && surf_scans);

            pcl::PointCloud<PointT>::Ptr corn_tmp(new pcl::PointCloud<PointT>);
            pcl::PointCloud<PointT>::Ptr surf_tmp(new pcl::PointCloud<PointT>);

            for (const auto &scan : scans) {
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
        gtsam::Pose3 pose_odom;      // for odom and mapping
        gtsam::Pose3 pose_last_curr; // increment of pose_odom
        gtsam::Pose3 pose_fixed;     // for global path and map
        // feature point cloud
        pcl::PointCloud<PointT>::Ptr corn_features;
        pcl::PointCloud<PointT>::Ptr surf_features;
        pcl::PointCloud<PointT>::Ptr raw;
    public:
        Keyframe(int index_, const ros::Time &time,
                 PointCloudPtr EF, PointCloudPtr PF, PointCloudPtr RAW) :
                index(index_), cloud_in_time(time),
                corn_features(EF), surf_features(PF), raw(RAW) {}

        inline void set_fixed(gtsam::Pose3 pose_) {
            pose_fixed = std::move(pose_);
            fixed = true;
        }

        inline void set_init(gtsam::Pose3 pose_) {
            pose_odom = std::move(pose_);
            add_frame();
        }

        inline void set_increment(gtsam::Pose3 pose_) {
            pose_last_curr = std::move(pose_);
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
                        poseVec.emplace_back(keyframes[i]->pose_fixed);
                    }
                } else {
                    if (keyframes[i]->is_init()) {
                        poseVec.emplace_back(keyframes[i]->pose_odom);
                    }
                }
            }

            return poseVec;
        }

        // read pose that is inited, not require fixed
        gtsam::Pose3 read_pose(size_t index, Type type) const {
            if (index > keyframes.size()) {
                std::cerr << "read_pose invalid range: " << index << ", but size is " <<
                keyframes.size() << std::endl;
                return gtsam::Pose3();
            }

            if (type == Type::Odom) {
                if (keyframes[index]->is_init()) {
                    std::shared_lock<std::shared_mutex> sl(pose_mtx);
                    return keyframes[index]->pose_odom;
                } else {
                    std::cerr << "read pose_odom fail, pose not init!" << std::endl;
                    return gtsam::Pose3();
                }
            } else if (type == Type::Opti) {
                if (keyframes[index]->is_fixed()) {
                    std::shared_lock<std::shared_mutex> sl(pose_mtx);
                    return keyframes[index]->pose_fixed;
                } else {
                    std::cerr << "read pose_opti, pose not fixed!" << std::endl;
                    return gtsam::Pose3();
                }
            } else {
                std::shared_lock<std::shared_mutex> sl(pose_mtx);
                return keyframes[index]->pose_last_curr;
            }
        }

        // read pose that is inited, not require fixed
        gtsam::Pose3 read_pose(size_t index, bool need_fixed = false) const {
            if (index > keyframes.size()) {
                std::cerr << "read_pose invalid range" << std::endl;
                return gtsam::Pose3();
            }

            if (need_fixed) {
                if (keyframes[index]->is_fixed()) {
                    std::shared_lock<std::shared_mutex> sl(pose_mtx);
                    return keyframes[index]->pose_fixed;
                } else {
                    std::cerr << "pose not fixed!" << std::endl;
                    return gtsam::Pose3();
                }
            } else {
                if (keyframes[index]->is_init()) {
                    std::shared_lock<std::shared_mutex> sl(pose_mtx);
                    return keyframes[index]->pose_odom;
                } else {
                    std::cerr << "pose not init!" << std::endl;
                    return gtsam::Pose3();
                }
            }
        }

        inline size_t size() const {
            return keyframes.size();
        }

        inline Keyframe::Ptr &at(size_t index) {
            return this->keyframes.at(index);
        }

        inline const Keyframe::Ptr &at(size_t index) const {
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
