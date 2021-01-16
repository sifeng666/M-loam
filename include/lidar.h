//
// Created by ziv on 2020/12/31.
//

#ifndef MLOAM_LIDAR_H
#define MLOAM_LIDAR_H

#include "keyframe.h"

#include "map_generator.h"
#include "factors.h"
#include "loop_detector.h"

#include <pcl/segmentation/region_growing.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>

class LidarStatus {
public:
    using Ptr = boost::shared_ptr<LidarStatus>;
    bool status_change = false;
};

class LidarMsgReader {
public:
    void lock();
    void unlock();
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
    virtual ~LidarSensor();
    void setName(const std::string& lidar_name);
    KeyframeVec::Ptr get_keyframeVec() const;
    LidarStatus::Ptr get_status() const;
    std::string get_lidar_name() const;

    void downsampling(const pcl::PointCloud<PointT>::Ptr& input,
                      pcl::PointCloud<PointT>& output,
                      FeatureType featureType);
    
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
    
    void updateSubmap();
    bool nextFrameToBeKeyframe();
    void initOdomFactor();
    void addOdomFactor();
    void handleRegistration();
    void updatePoses();
    void initParam();

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

    void BA_optimization();


private:

    /*********************************************************************
   ** Global Variables
   *********************************************************************/
    ros::NodeHandle nh;
    std::string lidar_name;

    KeyframeVec::Ptr keyframeVec;
    LoopDetector loopDetector;
    bool need_loop = true;

    double map_resolution;
    double KEYFRAME_DIST_THRES;
    double KEYFRAME_ANGLE_THRES;
    int SUBMAP_LEN;

    bool is_keyframe_next = true;

    // gtsam
//    std::mutex graph_mtx;
    gtsam::NonlinearFactorGraph BAGraph;
    gtsam::Values BAEstimate;
    std::shared_ptr<gtsam::ISAM2> isam;
    gtsam::Values isamOptimize;

    std::mutex poses_opti_mtx;
    std::vector<gtsam::Pose3> poses_optimized;
    bool poses_opti_update = false;
    int opti_counter;

    // gaussian model
    gtsam::SharedNoiseModel edge_gaussian_model, surf_gaussian_model;
    // noise model
    gtsam::SharedNoiseModel edge_noise_model, surf_noise_model, prior_noise_model, odometry_noise_model;


    /*********************************************************************
   ** Laser Odometry
   *********************************************************************/
    Keyframe::Ptr current_keyframe;
    gtsam::Pose3 odom;      // world to current frame
    gtsam::Pose3 last_odom; // world to last frame
    gtsam::Pose3 delta;     // last frame to current frame
    gtsam::Pose3 pose_w_c;  // to be optimized

    bool is_init = false;

    // frame-to-submap
    pcl::PointCloud<PointT>::Ptr submapEdge;
    pcl::PointCloud<PointT>::Ptr submapSurf;
    pcl::KdTreeFLANN<PointT>::Ptr kdtreeEdgeSubmap;
    pcl::KdTreeFLANN<PointT>::Ptr kdtreeSurfSubmap;

    // downsample filter
    pcl::VoxelGrid<PointT> downSizeFilterEdge;
    pcl::VoxelGrid<PointT> downSizeFilterSurf;
    pcl::VoxelGrid<PointT> downSizeFilterNor;

    // plane segment
    pcl::NormalEstimationOMP<PointT, NormalT> ne;
    pcl::search::KdTree<PointT>::Ptr searchKdtree;
    pcl::RegionGrowing<PointT, NormalT> regionGrowing;


    /*********************************************************************
   ** backend BA
   *********************************************************************/
    std::mutex BA_mtx;
    std::queue<Keyframe::Ptr> BAKeyframeBuf;

public:
    LidarStatus::Ptr status;
};



#endif //MLOAM_LIDAR_H
