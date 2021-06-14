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
    //子地图的数据结构
    class Submap {
    public:
        using Ptr = std::shared_ptr<Submap>; //指针别名
        mutable std::mutex mtx; //锁
        int from, end; //应该是子地图对应的初始帧和结束帧的序号
        pcl::PointCloud<PointT>::Ptr submap_corn; //角特征点云
        pcl::PointCloud<PointT>::Ptr submap_surf; //面特征点云

        Submap() {
            //初始化
            submap_corn.reset(new pcl::PointCloud<PointT>);
            submap_surf.reset(new pcl::PointCloud<PointT>);
        }
        //将面特征和角特征的点云融合返回
        pcl::PointCloud<PointT>::Ptr merge() const {
            pcl::PointCloud<PointT>::Ptr ret(new pcl::PointCloud<PointT>);
            std::lock_guard lg(mtx);
            *ret += *submap_corn;
            *ret += *submap_surf;
            return ret;
        }

    };
    //激光帧的数据结构
    class Frame {
    public:
        using Ptr = std::shared_ptr<Frame>; //using是定义别名，即将一个类型代替其他别名，跟typedef的用法类似
        int index; //序号
        ros::Time cloud_in_time; //激光帧的时间戳
        gtsam::Pose3 pose_w_curr; //当前坐标系的位姿
        pcl::PointCloud<PointT>::Ptr corn_features; //角特征点云
        pcl::PointCloud<PointT>::Ptr surf_features; //面特征点云
        pcl::PointCloud<PointT>::Ptr raw; //原始点云

        Frame(int index_, const ros::Time time,
              const pcl::PointCloud<PointT>::Ptr &corn,
              const pcl::PointCloud<PointT>::Ptr &surf,
              const pcl::PointCloud<PointT>::Ptr &raw_,
              const gtsam::Pose3 &pose) :
                index(index_), cloud_in_time(time), corn_features(corn),
                surf_features(surf), raw(raw_), pose_w_curr(pose) {}
    };

    //带锁地执行T类型的buf序列的插入提取等操作
    template<class T>
    class Channel {
    public:
        using Ptr = std::shared_ptr<Channel>; //数据结构的指针
    private:
        std::mutex mtx; //加锁
        std::queue<T> buf; //T类型的队列
    public:
        inline void lock() { //加锁
            mtx.lock();
        }

        inline void unlock() { //解锁
            mtx.unlock();
        }

        inline void push(T frame) { //带锁地将帧插入到buf中
            lock();
            buf.push(frame);
            unlock();
        }

        inline void clear() { //带锁地清空帧
            lock();
            while (!buf.empty()) {
                buf.pop();
            }
            unlock();
        }

        inline T get_front() { //带锁拿到该channel的buff中的头帧
            T t = nullptr;
            lock();
            if (!buf.empty()) {
                t = buf.front();
                buf.pop();
            }
            unlock();
            return t;
        }

        inline std::vector<T> get_all() { //将buf中的所有帧提取，而且清空buf
            std::vector<T> vec;
            lock();
            while (!buf.empty()) {
                vec.push_back(buf.front());
                buf.pop();
            }
            unlock();
            return vec;
        }

        inline bool empty() { //判断该帧是否为空
            std::lock_guard lg(mtx);
            return buf.empty();
        }
    };
    //存储最近N帧的数据结构
    class NScans {
    private:
        std::list<Frame::Ptr> scans;
        int n;
    public:

        NScans(int n_ = 6) : n(n_) {}
        //插入最新帧，满了则pop掉最旧的帧
        void add(Frame::Ptr new_scan) {
            scans.push_front(new_scan);
            if (scans.size() > n) {
                scans.pop_back();
            }
        }
        //读取最近N帧，并累加成角特征的点云和面特征的点云
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
    //单帧关键帧的数据结构
    class Keyframe {
    public:
        using Ptr = std::shared_ptr<Keyframe>; //keyframe的指针
        using PointCloudPtr = pcl::PointCloud<PointT>::Ptr; //点云的指针
    public:
        int index; //序号
        bool fixed = false; //是否被optimized过
        int valid_frames = 0; //有效的帧数
        ros::Time cloud_in_time; //时间戳
        gtsam::Pose3 pose_odom;      // for odom and mapping
        gtsam::Pose3 pose_last_curr; // increment of pose_odom  odom中相对上一帧的变化量
        gtsam::Pose3 pose_fixed;     // for global path and map
        // feature point cloud
        pcl::PointCloud<PointT>::Ptr corn_features; //角特征点云
        pcl::PointCloud<PointT>::Ptr surf_features; //面特征点云
        pcl::PointCloud<PointT>::Ptr raw; //原始点云
    public:
        Keyframe(int index_, const ros::Time &time,
                 PointCloudPtr EF, PointCloudPtr PF, PointCloudPtr RAW) :
                index(index_), cloud_in_time(time),
                corn_features(EF), surf_features(PF), raw(RAW) {}
        //将输入的pose通过move转移到pose_fixed中，并将该帧的fixed状态设置成true
        inline void set_fixed(gtsam::Pose3 pose_) {
            pose_fixed = std::move(pose_);
            fixed = true;
        }
        //将初始化的pose通过move转移给pose_odom，并调用add_frame()函数，将valid_frame（记录有效帧的值）+1
        //这里是不是应该用一个static变量记录valid_frames？
        inline void set_init(gtsam::Pose3 pose_) {
            pose_odom = std::move(pose_);
            add_frame();
        }
        //将输入的pose的值转移给pose_last_curr（增量）
        inline void set_increment(gtsam::Pose3 pose_) {
            pose_last_curr = std::move(pose_);
        }

        inline void add_frame() {
            ++valid_frames;
        }
        //判断本帧是否经过优化
        inline bool is_fixed() const {
            return fixed;
        }
        //判断是否init的依据是有效的帧数量是否大于0
        inline bool is_init() const {
            return valid_frames > 0;
        }
    };
    //关键帧vector数据结构，存储所有的关键帧；此处的fixed是指关键帧的位姿已经被optimized过了
    class KeyframeVec {
    public:
        using Ptr = std::shared_ptr<KeyframeVec>; //别名，typedef
    public:
        mutable std::shared_mutex pose_mtx; //mutable修饰的变量永远可变，即使在一个const函数当中
        std::vector<Keyframe::Ptr> keyframes; //存储所有关键帧的vector
    public:
        // read poses that are inited or fixed
        //给定begin和end两个序号，读取之间的所有pose
        std::vector<gtsam::Pose3> read_poses(size_t begin, size_t end, bool need_fixed = false) const {
            //取值范围异常处理
            if (begin >= end || end > keyframes.size()) {
//                std::cerr << "read_poses invalid range" << std::endl;
                return {};
            }
            //位姿存储vector
            std::vector<gtsam::Pose3> poseVec;
            //加锁
            std::shared_lock<std::shared_mutex> sl(pose_mtx);
            //循环遍历
            for (size_t i = begin; i < end; i++) {
                if (need_fixed) { //需要pose是Optimize过的，只插入is_fixed（被优化过）为true的关键帧
                    if (keyframes[i]->is_fixed()) {
                        poseVec.emplace_back(keyframes[i]->pose_fixed);
                    }
                } else {//不需要是优化过的，只要keyframe是初始化过的，就可以插入
                    if (keyframes[i]->is_init()) {
                        poseVec.emplace_back(keyframes[i]->pose_odom);
                    }
                }
            }

            return poseVec;
        }

        // read pose that is inited, not require fixed
        gtsam::Pose3 read_pose(size_t index, Type type) const {
            //边界异常处理，返回默认值
            if (index > keyframes.size()) {
                std::cerr << "read_pose invalid range: " << index << ", but size is " <<
                keyframes.size() << std::endl;
                return gtsam::Pose3();
            }
            //Type为Odom即未优化，只要该帧是init后的即返回pose_odom，否则提示错误并返回默认初值
            if (type == Type::Odom) {
                if (keyframes[index]->is_init()) {
                    std::shared_lock<std::shared_mutex> sl(pose_mtx);
                    return keyframes[index]->pose_odom;
                } else {
                    std::cerr << "read pose_odom fail, pose not init!" << std::endl;
                    return gtsam::Pose3();
                }
                //Type为Optimized，判断位姿是否修正过（fixed），如果没有则报错返回默认值
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
                return keyframes[index]->pose_last_curr; //返回相对上一帧的位姿变化量（即变换矩阵）
            }
        }

        // read pose that is inited, not require fixed
        // 读取初始化后但是否优化可选择的位姿
        gtsam::Pose3 read_pose(size_t index, bool need_fixed = false) const {
            //输入非法检测
            if (index > keyframes.size()) {
                std::cerr << "read_pose invalid range" << std::endl;
                return gtsam::Pose3();
            }
            //需要优化
            if (need_fixed) {
                //判断是否优化
                if (keyframes[index]->is_fixed()) {
                    std::shared_lock<std::shared_mutex> sl(pose_mtx);
                    return keyframes[index]->pose_fixed;
                } else {
                    std::cerr << "pose not fixed!" << std::endl;
                    return gtsam::Pose3();
                }
                //不需要优化
            } else {
                //是否初始化
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
