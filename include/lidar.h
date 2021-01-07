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

class LidarMsgReader {
public:
    void lock();
    void unlock();
    void pointCloudEdgeHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);
    void pointCloudSurfHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);
private:
    std::mutex pcd_msg_mtx;
public:
    // queue of ros pointcloud msg
    std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudEdgeBuf;
    std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudSurfBuf;
};

class LidarSensor {
public:

    explicit LidarSensor();
    virtual ~LidarSensor();
    void setName(const std::string& lidar_name);
    void setResolution(double map_resolution);

    void pointAssociate(PointT const *const pi, PointT *const po, const gtsam::Pose3& odom);
    void downsampling(const pcl::PointCloud<PointT>::Ptr& input, pcl::PointCloud<PointT>& output, FeatureType featureType);
    
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
    void update_keyframe(pcl::PointCloud<PointT>::Ptr currEdge, pcl::PointCloud<PointT>::Ptr currSurf, pcl::PointCloud<PointT>::Ptr currFull);
    void getTransToSubmap(pcl::PointCloud<PointT>::Ptr currEdgeDS, pcl::PointCloud<PointT>::Ptr currSurfDS);
    gtsam::Pose3 update(pcl::PointCloud<PointT>::Ptr currEdge, pcl::PointCloud<PointT>::Ptr currSurf);
    KeyframeVec::Ptr get_keyframeVec() const;
    std::string get_lidar_name() const;

    void BA_optimization();


private:

    /*********************************************************************
   ** Global Variables
   *********************************************************************/
    ros::NodeHandle nh;
    ros::Time cloud_in_time;
    std::string lidar_name;

    KeyframeVec::Ptr keyframeVec;
    LoopDetector loopDetector;

    double map_resolution;
    double KEYFRAME_DIST_THRES;
    double KEYFRAME_ANGLE_THRES;
    int SUBMAP_LEN;

    bool is_keyframe_next = true;

    // gtsam
    std::mutex graph_mtx;
    gtsam::NonlinearFactorGraph BAGraph;
    gtsam::Values BAEstimate;
    std::shared_ptr<gtsam::ISAM2> isam;
    gtsam::Values isamOptimize;

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
    // pcl::PointCloud<PointT>::Ptr submapPlane;
    pcl::KdTreeFLANN<PointT>::Ptr kdtreeEdgeSubmap;
    pcl::KdTreeFLANN<PointT>::Ptr kdtreeSurfSubmap;
    // pcl::KdTreeFLANN<PointT>::Ptr kdtreePlaneSubmap;

    // downsample filter
    pcl::VoxelGrid<PointT> downSizeFilterEdge;
    pcl::VoxelGrid<PointT> downSizeFilterSurf;
    pcl::VoxelGrid<PointT> downSizeFilterNor;

    // point cloud from laserProcessing
    pcl::PointCloud<PointT>::Ptr cloud_in_edge;
    pcl::PointCloud<PointT>::Ptr cloud_in_surf;
    pcl::PointCloud<PointT>::Ptr cloud_in_full;

    // plane segment
    pcl::NormalEstimationOMP<PointT, NormalT> ne;
    pcl::search::KdTree<PointT>::Ptr searchKdtree;
    pcl::RegionGrowing<PointT, NormalT> regionGrowing;


    /*********************************************************************
   ** backend BA
   *********************************************************************/
    std::mutex BA_mtx;
    std::queue<Keyframe::Ptr> BAKeyframeBuf;
    bool status_change = false;
    
};



#endif //MLOAM_LIDAR_H
