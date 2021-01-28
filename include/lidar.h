//
// Created by ziv on 2020/12/31.
//

#ifndef MLOAM_LIDAR_H
#define MLOAM_LIDAR_H

#include "keyframe/keyframe.h"
#include "map_generator/map_generator.h"
#include "loop/loop_detector.h"
#include "factors.h"

#include "balmclass.hpp"

#include <pcl/segmentation/region_growing.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>

// extract from balm back
static double voxel_size[2] = {1, 1};
static void cut_voxel(unordered_map<VOXEL_LOC, OCTO_TREE*> &feat_map, pcl::PointCloud<PointT>::Ptr pl_feat, Eigen::Matrix3d R_p, Eigen::Vector3d t_p, int feattype, int fnum, int capacity)
{
    uint plsize = pl_feat->size();
    for(uint i=0; i<plsize; i++)
    {
        PointT &p_c = pl_feat->points[i];
        Eigen::Vector3d pvec_orig(p_c.x, p_c.y, p_c.z);
        Eigen::Vector3d pvec_tran = R_p*pvec_orig + t_p;

        float loc_xyz[3];
        for(int j=0; j<3; j++)
        {
            loc_xyz[j] = pvec_tran[j] / voxel_size[feattype];
            if(loc_xyz[j] < 0)
            {
                loc_xyz[j] -= 1.0;
            }
        }

        VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
        auto iter = feat_map.find(position);
        if(iter != feat_map.end())
        {
            iter->second->plvec_orig[fnum]->push_back(pvec_orig);
            iter->second->plvec_tran[fnum]->push_back(pvec_tran);
            iter->second->is2opt = true;
        }
        else
        {
            OCTO_TREE *ot = new OCTO_TREE(feattype, capacity);
            ot->plvec_orig[fnum]->push_back(pvec_orig);
            ot->plvec_tran[fnum]->push_back(pvec_tran);

            ot->voxel_center[0] = (0.5+position.x) * voxel_size[feattype];
            ot->voxel_center[1] = (0.5+position.y) * voxel_size[feattype];
            ot->voxel_center[2] = (0.5+position.z) * voxel_size[feattype];
            ot->quater_length = voxel_size[feattype] / 4.0;
            feat_map[position] = ot;
        }
    }
}

class LidarStatus {
public:
    using Ptr = boost::shared_ptr<LidarStatus>;
    bool status_change  = false;
    bool map_refresh_signal = false;
    bool is_end = false;
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

    int addEdgeCostFactor(const pcl::PointCloud<PointT>::Ptr& pc_in, 
        const pcl::PointCloud<PointT>::Ptr& map_in,
        const pcl::KdTreeFLANN<PointT>::Ptr& kdtreeEdge,
        const gtsam::Pose3& odom,
        gtsam::NonlinearFactorGraph& factors);

    int addSurfCostFactor(
        const pcl::PointCloud<PointT>::Ptr& pc_in,
        const pcl::PointCloud<PointT>::Ptr& map_in,
        const pcl::KdTreeFLANN<PointT>::Ptr& kdtreeSurf,
        const gtsam::Pose3& odom,
        gtsam::NonlinearFactorGraph& factors);
    
    void updateSubmap(size_t range_from = 0, size_t range_to = 0);
    bool nextFrameToBeKeyframe();
    void handleRegistration();
    void updatePoses();
    void initParam();
    void resetBALMParam();

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
                pcl::PointCloud<PointT>::Ptr currLessEdge,
                pcl::PointCloud<PointT>::Ptr currLessSurf,
                pcl::PointCloud<PointT>::Ptr currRaw);

    void addOdomFactor(int last_index, int index);

    void BA_optimization();

    void BALM_backend(const std::vector<Keyframe::Ptr>& keyframes_buf,
                      std::vector<Keyframe::Ptr>& fixedKeyframes_buf);


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
    gtsam::SharedNoiseModel edge_noise_model;
    gtsam::SharedNoiseModel surf_noise_model;
    gtsam::SharedNoiseModel prior_noise_model;
    gtsam::SharedNoiseModel odometry_noise_model;


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
    pcl::PointCloud<PointT>::Ptr submapEdge;
    pcl::PointCloud<PointT>::Ptr submapSurf;
    pcl::KdTreeFLANN<PointT>::Ptr kdtreeEdgeSubmap;
    pcl::KdTreeFLANN<PointT>::Ptr kdtreeSurfSubmap;

    // plane segment
    pcl::NormalEstimationOMP<PointT, NormalT> ne;
    pcl::search::KdTree<PointT>::Ptr searchKdtree;
    pcl::RegionGrowing<PointT, NormalT> regionGrowing;


    /*********************************************************************
   ** backend BA
   *********************************************************************/
    std::mutex BA_mtx;
    std::queue<Keyframe::Ptr> BAKeyframeBuf;


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
};



#endif //MLOAM_LIDAR_H
