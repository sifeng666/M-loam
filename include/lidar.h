//
// Created by ziv on 2020/12/31.
//

#ifndef MLOAM_LIDAR_H
#define MLOAM_LIDAR_H

#include "tools/keyframe.hpp"
#include "tools/map_generator.hpp"
#include "tools/loop_detector.hpp"
#include "tools/registration.hpp"

#include "factors.h"
#include "balmclass.hpp"

#include <pcl/segmentation/region_growing.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>

using namespace tools;

class LidarStatus {
public:
    using Ptr = std::shared_ptr<LidarStatus>;
    bool status_change  = false;
    bool is_end = false;
    int last_loop_found_index = 0;
};

class FixedKeyframeChannel {
public:
    using Ptr = boost::shared_ptr<FixedKeyframeChannel>;
    inline void lock()   { mtx.lock(); }
    inline void unlock() { mtx.unlock(); }
    std::mutex mtx;
    std::queue<Keyframe::Ptr> FixedKeyframeBuf;
    inline void push(Keyframe::Ptr keyframe) { lock(); FixedKeyframeBuf.push(keyframe); unlock(); }
    inline Keyframe::Ptr get_front() {
        Keyframe::Ptr keyframe = nullptr;
        if (!FixedKeyframeBuf.empty()) {
            lock(); keyframe = FixedKeyframeBuf.front(); FixedKeyframeBuf.pop(); unlock();
        }
        return keyframe;
    }
};

class FrameChannel {
public:
    using Ptr = std::shared_ptr<FrameChannel>;
    std::mutex mtx;
    std::queue<Frame::Ptr> frameBuf;
    inline void lock() {
        mtx.lock();
    }
    inline void unlock() {
        mtx.unlock();
    }
    inline void push(Frame::Ptr frame) {
        lock();
        frameBuf.push(frame);
        unlock();
    }
    inline void clear() {
        lock();
        while (!frameBuf.empty()) {
            frameBuf.pop();
        }
        unlock();
    }

    inline Frame::Ptr get_front() {
        Frame::Ptr frame = nullptr;
        if (!frameBuf.empty()) {
            lock();
            frame = frameBuf.front();
            frameBuf.pop();
            unlock();
        }
        return frame;
    }
};

class LidarMsgReader {
public:
    inline void lock()   { pcd_msg_mtx.lock(); }
    inline void unlock() { pcd_msg_mtx.unlock(); }
    void pointCloudFullHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);
    void pointCloudEdgeHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);
    void pointCloudSurfHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);
    void pointCloudLessEdgeHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);
    void pointCloudLessSurfHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);
private:
    std::mutex pcd_msg_mtx;
public:
    // queue of ros pointcloud msg
    std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudFullBuf;
    std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudEdgeBuf;
    std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudSurfBuf;
    std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudLessEdgeBuf;
    std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudLessSurfBuf;
};

class LidarSensor {
public:
    explicit LidarSensor();

    inline void set_name(const std::string& lidar_name) { this->lidar_name = lidar_name; }
    inline std::string get_lidar_name() const { return lidar_name; }
    inline KeyframeVec::Ptr get_keyframeVec() const { return keyframeVec; }
    inline FrameChannel::Ptr get_frameChannel() const { return frameChannel; }

    static void addCornCostFactor(const pcl::PointCloud<PointT>::Ptr& pc_in,
                                  const pcl::PointCloud<PointT>::Ptr& map_in,
                                  const pcl::KdTreeFLANN<PointT>& kdtree_corn,
                                  const gtsam::Pose3& odom,
                                  gtsam::NonlinearFactorGraph& factors,
                                  const gtsam::Pose3& point_transform = gtsam::Pose3::identity());

    static void addSurfCostFactor(const pcl::PointCloud<PointT>::Ptr& pc_in,
                                  const pcl::PointCloud<PointT>::Ptr& map_in,
                                  const pcl::KdTreeFLANN<PointT>& kdtree_surf,
                                  const gtsam::Pose3& odom,
                                  gtsam::NonlinearFactorGraph& factors,
                                  const gtsam::Pose3& point_transform = gtsam::Pose3::identity());
    
    void updateSubmap(size_t range_from = 0, size_t range_to = 0);
    bool nextFrameToBeKeyframe();
    void handleRegistration();
    void updatePoses();
    void initParam();
    void initBALMParam();

    void feature_based_scan_matching(const pcl::PointCloud<PointT>::Ptr& corn,
                                     const pcl::PointCloud<PointT>::Ptr& surf,
                                     const pcl::PointCloud<PointT>::Ptr& corns,
                                     const pcl::PointCloud<PointT>::Ptr& surfs,
                                     pcl::KdTreeFLANN<PointT>& kdtree_corn,
                                     pcl::KdTreeFLANN<PointT>& kdtree_surf,
                                     gtsam::Pose3& T_w_c);

    void point_wise_scan_matching(const pcl::PointCloud<PointT>::Ptr& corn,
                                  const pcl::PointCloud<PointT>::Ptr& surf,
                                  const pcl::PointCloud<PointT>::Ptr& corns,
                                  const pcl::PointCloud<PointT>::Ptr& surfs,
                                  gtsam::Pose3& T_w_c);

    void update_keyframe(const ros::Time& cloud_in_time, const pcl::PointCloud<PointT>::Ptr& corn,
                         const pcl::PointCloud<PointT>::Ptr& surf, const pcl::PointCloud<PointT>::Ptr& full);

    gtsam::Pose3 scan2scan(const ros::Time& cloud_in_time, const pcl::PointCloud<PointT>::Ptr& corn,
                           const pcl::PointCloud<PointT>::Ptr& surf, const pcl::PointCloud<PointT>::Ptr& raw);

    gtsam::Pose3 scan2submap(const pcl::PointCloud<PointT>::Ptr& corn, const pcl::PointCloud<PointT>::Ptr& surf,
                             const gtsam::Pose3& guess, int method = 0);

    gtsam::Pose3 update_odom(const ros::Time& cloud_in_time, const pcl::PointCloud<PointT>::Ptr& corn,
                             const pcl::PointCloud<PointT>::Ptr& surf, const pcl::PointCloud<PointT>::Ptr& raw);

    gtsam::Pose3 update_mapping(const Frame::Ptr& frame);

    void addOdomFactor(int last_index, int index);

    void BA_optimization();

    void factor_graph_opti();

    void loop_detect_thread();


private:

    /*********************************************************************
   ** Global Variables
   *********************************************************************/
    ros::NodeHandle nh;
    std::string lidar_name;

    KeyframeVec::Ptr keyframeVec;
    LoopDetector loopDetector;

    // ros hyper param
    bool need_loop = true;
    double surf_filter_length;
    double corn_filter_length;
    double keyframe_distance_thres;
    double keyframe_angle_thres;

    bool is_keyframe_next = true;
    int opti_counter;

    // gtsam
    gtsam::NonlinearFactorGraph BAGraph;
    gtsam::Values BAEstimate;
    std::shared_ptr<gtsam::ISAM2> isam;
    gtsam::Values isamOptimize;

    // noise model
    static gtsam::SharedNoiseModel edge_noise_model;
    static gtsam::SharedNoiseModel surf_noise_model;
    gtsam::SharedNoiseModel prior_noise_model;
    gtsam::SharedNoiseModel odometry_noise_model;


    /*********************************************************************
   ** Odometry
   *********************************************************************/
    Keyframe::Ptr current_keyframe;

    // raw odometry 10Hz
    gtsam::Pose3 T_w_odomcurr;
    gtsam::Pose3 T_w_odomlast;
    gtsam::Pose3 T_last_curr;
    // finetune odometry
    gtsam::Pose3 T_odom_map;    // <10Hz
    gtsam::Pose3 T_w_curr;      // 10Hz
    gtsam::Pose3 T_w_mapcurr;

    bool is_init = false;

    // frame-to-submap
    std::mutex submap_mtx;
    pcl::PointCloud<PointT>::Ptr submap_corn;
    pcl::PointCloud<PointT>::Ptr submap_surf;
    pcl::KdTreeFLANN<PointT> kdtree_corn_submap;
    pcl::KdTreeFLANN<PointT> kdtree_surf_submap;

    int n_scans;
    NScans nscans_gen;
    pcl::KdTreeFLANN<PointT> kdtree_corn_nscans;
    pcl::KdTreeFLANN<PointT> kdtree_surf_nscans;

    pcl::VoxelGrid<PointT> ds_corn;
    pcl::VoxelGrid<PointT> ds_surf;

    // loop
    std::mutex fixed_mtx;
    std::queue<Keyframe::Ptr> FixedKeyframeBuf;
    std::mutex loop_factors_mtx;
    std::queue<FactorPtr> factorsBuf;

    int mapping_method;
    int submap_len;

    FrameChannel::Ptr frameChannel;
    Frame::Ptr curr_frame;
    int frameCount = 0;



public:
    LidarStatus::Ptr status;
};



#endif //MLOAM_LIDAR_H
