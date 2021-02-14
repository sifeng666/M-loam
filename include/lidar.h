//
// Created by ziv on 2020/12/31.
//

#ifndef MLOAM_LIDAR_H
#define MLOAM_LIDAR_H

#include "tools/keyframe.hpp"
#include "tools/map_generator.hpp"
#include "tools/loop_detector.hpp"
#include "tools/registration.hpp"
#include "tools/calibration_graph.h"

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

    // point_transform is applied when curr_point adding to gtsam cost function
    static void addCornCostFactor(const pcl::PointCloud<PointT>::Ptr& pc_in,
                                  const pcl::PointCloud<PointT>::Ptr& map_in,
                                  const pcl::KdTreeFLANN<PointT>& kdtree_corn,
                                  const gtsam::Pose3& odom,
                                  gtsam::NonlinearFactorGraph& factors,
                                  const gtsam::Pose3& point_transform = gtsam::Pose3::identity());

    // point_transform is applied when curr_point adding to gtsam cost function
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
    void updateISAM();
    void initParam();
    void initBALMParam();

    pcl::PointCloud<PointT>::Ptr extract_planes(pcl::PointCloud<PointT>::Ptr currSurf);

    void update_keyframe(const ros::Time& cloud_in_time,
                         pcl::PointCloud<PointT>::Ptr currEdge,
                         pcl::PointCloud<PointT>::Ptr currSurf,
                         pcl::PointCloud<PointT>::Ptr currFull);

    void getTransToSubmap(pcl::PointCloud<PointT>::Ptr currEdge,
                          pcl::PointCloud<PointT>::Ptr currSurf);

    gtsam::Pose3 update(const ros::Time cloud_in_time,
                pcl::PointCloud<PointT>::Ptr currEdge,
                pcl::PointCloud<PointT>::Ptr currSurf,
                pcl::PointCloud<PointT>::Ptr currRaw);

    void addOdomFactor(int last_index, int index);

    void BA_optimization();

    void BALM_backend(const std::vector<Keyframe::Ptr>& keyframes_buf,
                      std::vector<Keyframe::Ptr>& fixedKeyframes_buf);

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


    /*********************************************************************
   ** Odometry
   *********************************************************************/
    Keyframe::Ptr current_keyframe;
    gtsam::Pose3 odom;      // world to current frame
    gtsam::Pose3 last_odom; // world to last frame
    gtsam::Pose3 delta;     // last frame to current frame
    gtsam::Pose3 pose_w_c;  // to be optimized

    bool is_init = false;

    // frame-to-submap
    std::mutex submap_mtx;
    pcl::PointCloud<PointT>::Ptr submap_corn;
    pcl::PointCloud<PointT>::Ptr submap_surf;
    pcl::KdTreeFLANN<PointT> kdtree_corn;
    pcl::KdTreeFLANN<PointT> kdtree_surf;
    pcl::VoxelGrid<PointT> voxelGrid;

    /*********************************************************************
   ** backend BA
   *********************************************************************/
    std::mutex BA_mtx;
    std::queue<Keyframe::Ptr> BAKeyframeBuf;

    // loop
    std::mutex fixed_mtx;
    std::queue<Keyframe::Ptr> FixedKeyframeBuf;
    std::mutex loop_factors_mtx;
    std::queue<FactorPtr> factorsBuf;


    int window_size = 6;
    int margi_size = 3;
    const int filter_num = 1;
    const int thread_num = 4;


    LM_SLWD_VOXEL opt_lsv;
    pcl::PointCloud<PointT>::Ptr pl_corn;
    pcl::PointCloud<PointT>::Ptr pl_surf;
    vector<pcl::PointCloud<PointT>::Ptr> pl_full_buf;
    unordered_map<VOXEL_LOC, OCTO_TREE*> surf_map, corn_map;
    vector<Eigen::Quaterniond> q_poses;
    vector<Eigen::Vector3d> t_poses;
    Eigen::Quaterniond q_odom, q_gather_pose, q_last;
    Eigen::Vector3d t_odom, t_gather_pose, t_last;
    int plcount, window_base;

public:
    LidarStatus::Ptr status;
    FixedKeyframeChannel::Ptr fixedKeyframeChannel;



    std::ofstream f_pose_fixed;


};



#endif //MLOAM_LIDAR_H
