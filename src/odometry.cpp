//
// Created by ziv on 2020/10/12.
//

#include "helper.h"
#include "pcl_utils.h"
#include "frame.h"
#include "lidarFactor.hpp"

class Odometry {
public:

    void undistortPoint(PointT const *const pi, PointT *const po) {
        //interpolation ratio
        double s = (pi->intensity - int(pi->intensity)) * SCAN_FREQUENCY;
        s = 1;

        Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, q_curr2last);
        Eigen::Vector3d t_point_last = s * t_curr2last;
        Eigen::Vector3d point(pi->x, pi->y, pi->z);
        Eigen::Vector3d un_point = q_point_last * point + t_point_last;

        po->x = un_point.x();
        po->y = un_point.y();
        po->z = un_point.z();
        po->intensity = pi->intensity;
    }

    //receive /laser_cloud_sharp
    void laserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsSharp2) {
        std::lock_guard lockGuard(mBuf);
        cornerSharpBuf.push(cornerPointsSharp2);
    }

    //receive /laser_cloud_less_sharp
    void laserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsLessSharp2) {
        std::lock_guard lockGuard(mBuf);
        cornerLessSharpBuf.push(cornerPointsLessSharp2);
    }

    //receive /laser_cloud_flat
    void laserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsFlat2) {
        std::lock_guard lockGuard(mBuf);
        surfFlatBuf.push(surfPointsFlat2);
    }

    //receive /laser_cloud_less_flat
    void laserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsLessFlat2) {
        std::lock_guard lockGuard(mBuf);
        surfLessFlatBuf.push(surfPointsLessFlat2);
    }

    //receive /velodyne_cloud_2
    void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullRes2) {
        std::lock_guard lockGuard(mBuf);
        fullPointsBuf.push(laserCloudFullRes2);
    }

    void downsample(const pcl::PointCloud<PointT>::Ptr &cloud_in, pcl::PointCloud<PointT>::Ptr& cloud_out) const {
        if(!downsample_filter) {
            return;
        }

        downsample_filter->setInputCloud(cloud_in);
        downsample_filter->filter(*cloud_out);
        cloud_out->header = cloud_in->header;
    }

    void outlier_removal(const pcl::PointCloud<PointT>::Ptr &cloud_in, pcl::PointCloud<PointT>::Ptr& cloud_out) const {
        if(!outlier_removal_filter) {
            return;
        }

        outlier_removal_filter->setInputCloud(cloud_in);
        outlier_removal_filter->filter(*cloud_out);
        cloud_out->header = cloud_in->header;
    }

    void swapPointCloud(pcl::PointCloud<PointT>::Ptr& p1, pcl::PointCloud<PointT>::Ptr& p2) {
        pcl::PointCloud<PointT>::Ptr pTemp = p1;
        p1 = p2;
        p2 = pTemp;
    }

    // get pose between this frame to last frame(or keyframe), and set into frame's pose
    void getPose3BetweenFrames(const Frame::Ptr& toFrame, Frame::Ptr& thisFrame, bool toKeyframe) {

        int edgePointSize = thisFrame->edgeFeatures->size();
        int planePointSize = thisFrame->planeFeatures->size();

        int edge_correspondence = 0, plane_correspondence = 0;

        ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
        ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();
        ceres::Problem::Options problem_options;

        ceres::Problem problem(problem_options);

        // between keyframes, calculate delta_t from current frame to last keyframe
        if (toKeyframe) {
            problem.AddParameterBlock(para_q_2key, 4, q_parameterization);
            problem.AddParameterBlock(para_t_2key, 3);
        } else { // calculate delta_t from current frame to last normal frame
            problem.AddParameterBlock(para_q_2last, 4, q_parameterization);
            problem.AddParameterBlock(para_t_2last, 3);
        }

        PointT pointSel;
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        kdtreeEdgeFeature->setInputCloud(toFrame->edgeLessFeatures);
        kdtreePlaneFeature->setInputCloud(toFrame->planeLessFeatures);

        auto edgeFeaturesLast = toFrame->edgeLessFeatures;
        auto planeFeaturesLast = toFrame->planeLessFeatures;

        for (int i = 0; i < edgePointSize; ++i) {

            undistortPoint(&(thisFrame->edgeFeatures->points[i]), &pointSel);
            kdtreeEdgeFeature->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

            int closestPointInd = -1, minPointInd2 = -1;
            if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD) {
                closestPointInd = pointSearchInd[0];
                int closestPointScanID = int(edgeFeaturesLast->points[closestPointInd].intensity);

                double minPointSqDis2 = DISTANCE_SQ_THRESHOLD;
                // 寻找点O的另外一个最近邻的点（记为点B） in the direction of increasing scan line
                for (int j = closestPointInd + 1; j < (int)edgeFeaturesLast->points.size(); ++j) { // laserCloudCornerLast 来自上一帧的corner_less_sharp特征点,由于提取特征时是
                                                                                                       // 按照scan的顺序提取的，所以laserCloudCornerLast中的点也是按照scanID递增的顺序存放的
                    // if in the same scan line, continue
                    if (int(edgeFeaturesLast->points[j].intensity) <= closestPointScanID)// intensity整数部分存放的是scanID
                        continue;

                    // if not in nearby scans, end the loop
                    if (int(edgeFeaturesLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                        break;

                    double pointSqDis = (edgeFeaturesLast->points[j].x - pointSel.x) *
                                        (edgeFeaturesLast->points[j].x - pointSel.x) +
                                        (edgeFeaturesLast->points[j].y - pointSel.y) *
                                        (edgeFeaturesLast->points[j].y - pointSel.y) +
                                        (edgeFeaturesLast->points[j].z - pointSel.z) *
                                        (edgeFeaturesLast->points[j].z - pointSel.z);

                    if (pointSqDis < minPointSqDis2) { // 第二个最近邻点有效,，更新点B
                        // find nearer point
                        minPointSqDis2 = pointSqDis;
                        minPointInd2 = j;
                    }
                }

                // 寻找点O的另外一个最近邻的点B in the direction of decreasing scan line
                for (int j = closestPointInd - 1; j >= 0; --j) {
                    // if in the same scan line, continue
                    if (int(edgeFeaturesLast->points[j].intensity) >= closestPointScanID)
                        continue;

                    // if not in nearby scans, end the loop
                    if (int(edgeFeaturesLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                        break;

                    double pointSqDis = (edgeFeaturesLast->points[j].x - pointSel.x) *
                                        (edgeFeaturesLast->points[j].x - pointSel.x) +
                                        (edgeFeaturesLast->points[j].y - pointSel.y) *
                                        (edgeFeaturesLast->points[j].y - pointSel.y) +
                                        (edgeFeaturesLast->points[j].z - pointSel.z) *
                                        (edgeFeaturesLast->points[j].z - pointSel.z);

                    if (pointSqDis < minPointSqDis2) { // 第二个最近邻点有效，更新点B
                        // find nearer point
                        minPointSqDis2 = pointSqDis;
                        minPointInd2 = j;
                    }
                }
            }
            if (minPointInd2 >= 0) { // both closestPointInd and minPointInd2 is valid
                                     // 即特征点O的两个最近邻点A和B都有效
                Eigen::Vector3d curr_point(thisFrame->edgeFeatures->points[i].x,
                                           thisFrame->edgeFeatures->points[i].y,
                                           thisFrame->edgeFeatures->points[i].z);
                Eigen::Vector3d last_point_a(edgeFeaturesLast->points[closestPointInd].x,
                                             edgeFeaturesLast->points[closestPointInd].y,
                                             edgeFeaturesLast->points[closestPointInd].z);
                Eigen::Vector3d last_point_b(edgeFeaturesLast->points[minPointInd2].x,
                                             edgeFeaturesLast->points[minPointInd2].y,
                                             edgeFeaturesLast->points[minPointInd2].z);

                double s = (thisFrame->edgeFeatures->points[i].intensity - int(thisFrame->edgeFeatures->points[i].intensity)) * SCAN_FREQUENCY;
                // 用点O，A，B构造点到线的距离的残差项，注意这三个点都是在上一帧的Lidar坐标系下，即，残差 = 点O到直线AB的距离
                // 具体到介绍lidarFactor.cpp时再说明该残差的具体计算方法
                ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, last_point_a, last_point_b, s);
                if (toKeyframe) {
                    problem.AddResidualBlock(cost_function, loss_function, para_q_2key, para_t_2key);
                } else {
                    problem.AddResidualBlock(cost_function, loss_function, para_q_2last, para_t_2last);
                }
                edge_correspondence++;
            }
        }
        for (int i = 0; i < planePointSize; ++i) {
            undistortPoint(&(thisFrame->planeFeatures->points[i]), &pointSel);
            kdtreePlaneFeature->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

            int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
            if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD) { // 找到的最近邻点A有效
                closestPointInd = pointSearchInd[0];

                // get closest point's scan ID
                int closestPointScanID = int(planeFeaturesLast->points[closestPointInd].intensity);
                double minPointSqDis2 = DISTANCE_SQ_THRESHOLD, minPointSqDis3 = DISTANCE_SQ_THRESHOLD;

                // search in the direction of increasing scan line
                for (int j = closestPointInd + 1; j < (int)planeFeaturesLast->points.size(); ++j) {
                    // if not in nearby scans, end the loop
                    if (int(planeFeaturesLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                        break;

                    double pointSqDis = (planeFeaturesLast->points[j].x - pointSel.x) *
                                        (planeFeaturesLast->points[j].x - pointSel.x) +
                                        (planeFeaturesLast->points[j].y - pointSel.y) *
                                        (planeFeaturesLast->points[j].y - pointSel.y) +
                                        (planeFeaturesLast->points[j].z - pointSel.z) *
                                        (planeFeaturesLast->points[j].z - pointSel.z);

                    // if in the same or lower scan line
                    if (int(planeFeaturesLast->points[j].intensity) <= closestPointScanID && pointSqDis < minPointSqDis2) {
                        minPointSqDis2 = pointSqDis;// 找到的第2个最近邻点有效，更新点B，注意如果scanID准确的话，一般点A和点B的scanID相同
                        minPointInd2 = j;
                    }
                        // if in the higher scan line
                    else if (int(planeFeaturesLast->points[j].intensity) > closestPointScanID && pointSqDis < minPointSqDis3) {
                        minPointSqDis3 = pointSqDis;// 找到的第3个最近邻点有效，更新点C，注意如果scanID准确的话，一般点A和点B的scanID相同,且与点C的scanID不同，与LOAM的paper叙述一致
                        minPointInd3 = j;
                    }
                }

                // search in the direction of decreasing scan line
                for (int j = closestPointInd - 1; j >= 0; --j) {
                    // if not in nearby scans, end the loop
                    if (int(planeFeaturesLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                        break;

                    double pointSqDis = (planeFeaturesLast->points[j].x - pointSel.x) *
                                        (planeFeaturesLast->points[j].x - pointSel.x) +
                                        (planeFeaturesLast->points[j].y - pointSel.y) *
                                        (planeFeaturesLast->points[j].y - pointSel.y) +
                                        (planeFeaturesLast->points[j].z - pointSel.z) *
                                        (planeFeaturesLast->points[j].z - pointSel.z);

                    // if in the same or higher scan line
                    if (int(planeFeaturesLast->points[j].intensity) >= closestPointScanID && pointSqDis < minPointSqDis2) {
                        minPointSqDis2 = pointSqDis;
                        minPointInd2 = j;
                    }
                    else if (int(planeFeaturesLast->points[j].intensity) < closestPointScanID && pointSqDis < minPointSqDis3) {
                        // find nearer point
                        minPointSqDis3 = pointSqDis;
                        minPointInd3 = j;
                    }
                }

                if (minPointInd2 >= 0 && minPointInd3 >= 0) { // 如果三个最近邻点都有效

                    Eigen::Vector3d curr_point(thisFrame->planeFeatures->points[i].x,
                                               thisFrame->planeFeatures->points[i].y,
                                               thisFrame->planeFeatures->points[i].z);
                    Eigen::Vector3d last_point_a(planeFeaturesLast->points[closestPointInd].x,
                                                 planeFeaturesLast->points[closestPointInd].y,
                                                 planeFeaturesLast->points[closestPointInd].z);
                    Eigen::Vector3d last_point_b(planeFeaturesLast->points[minPointInd2].x,
                                                 planeFeaturesLast->points[minPointInd2].y,
                                                 planeFeaturesLast->points[minPointInd2].z);
                    Eigen::Vector3d last_point_c(planeFeaturesLast->points[minPointInd3].x,
                                                 planeFeaturesLast->points[minPointInd3].y,
                                                 planeFeaturesLast->points[minPointInd3].z);

                    double s = (thisFrame->planeFeatures->points[i].intensity - int(thisFrame->planeFeatures->points[i].intensity)) * SCAN_FREQUENCY;
                    // 用点O，A，B，C构造点到面的距离的残差项，注意这三个点都是在上一帧的Lidar坐标系下，即，残差 = 点O到平面ABC的距离
                    // 同样的，具体到介绍lidarFactor.cpp时再说明该残差的具体计算方法
                    ceres::CostFunction *cost_function = LidarPlaneFactor::Create(curr_point, last_point_a, last_point_b, last_point_c, s);
                    if (toKeyframe) {
                        problem.AddResidualBlock(cost_function, loss_function, para_q_2key, para_t_2key);
                    } else {
                        problem.AddResidualBlock(cost_function, loss_function, para_q_2last, para_t_2last);
                    }
                    plane_correspondence++;
                }
            }
        }
//        printf("corr: %d %d!!!!\n",  edge_correspondence, plane_correspondence);
        if ((edge_correspondence + plane_correspondence) < 20) {
            printf("less correspondence! *************************************************\n");
        }

//        Timer t_ceres;
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 4;
        options.minimizer_progress_to_stdout = false;
        ceres::Solver::Summary summary;
        // 基于构建的所有残差项，求解最优的当前帧位姿与上一帧位姿的位姿增量：para_q和para_t
        ceres::Solve(options, &problem, &summary);
//        t_ceres.count("ceres solver");
//        if (!toKeyframe)
//            printf("%f %f %f\n", para_t_2last[0], para_t_2last[1], para_t_2last[2]);
//        cout << q_curr2last.matrix() << endl << t_curr2last.matrix() << endl;
    }

    void getPose3ToKeyFrames(const Frame::Ptr& toFrame, Frame::Ptr& thisFrame) {

        int edgePointSize = thisFrame->edgeFeatures->size();
        int planePointSize = thisFrame->planeFeatures->size();

        int edge_correspondence = 0, plane_correspondence = 0;

        ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
        ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();
        ceres::Problem::Options problem_options;

        ceres::Problem problem(problem_options);

        problem.AddParameterBlock(para_q_2key, 4, q_parameterization);
        problem.AddParameterBlock(para_t_2key, 3);

        PointT pointSel;
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        kdtreeEdgeFeature->setInputCloud(toFrame->edgeLessFeatures);
        kdtreePlaneFeature->setInputCloud(toFrame->planeLessFeatures);

        auto edgeFeaturesLast = toFrame->edgeLessFeatures;
        auto planeFeaturesLast = toFrame->planeLessFeatures;

        for (int i = 0; i < edgePointSize; ++i) {

            undistortPoint(&(thisFrame->edgeFeatures->points[i]), &pointSel);
            kdtreeEdgeFeature->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

            int closestPointInd = -1, minPointInd2 = -1;
            if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD) {
                closestPointInd = pointSearchInd[0];
                int closestPointScanID = int(edgeFeaturesLast->points[closestPointInd].intensity);

                double minPointSqDis2 = DISTANCE_SQ_THRESHOLD;
                // 寻找点O的另外一个最近邻的点（记为点B） in the direction of increasing scan line
                for (int j = closestPointInd + 1; j < (int)edgeFeaturesLast->points.size(); ++j) { // laserCloudCornerLast 来自上一帧的corner_less_sharp特征点,由于提取特征时是
                    // 按照scan的顺序提取的，所以laserCloudCornerLast中的点也是按照scanID递增的顺序存放的
                    // if in the same scan line, continue
                    if (int(edgeFeaturesLast->points[j].intensity) <= closestPointScanID)// intensity整数部分存放的是scanID
                        continue;

                    // if not in nearby scans, end the loop
                    if (int(edgeFeaturesLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                        break;

                    double pointSqDis = (edgeFeaturesLast->points[j].x - pointSel.x) *
                                        (edgeFeaturesLast->points[j].x - pointSel.x) +
                                        (edgeFeaturesLast->points[j].y - pointSel.y) *
                                        (edgeFeaturesLast->points[j].y - pointSel.y) +
                                        (edgeFeaturesLast->points[j].z - pointSel.z) *
                                        (edgeFeaturesLast->points[j].z - pointSel.z);

                    if (pointSqDis < minPointSqDis2) { // 第二个最近邻点有效,，更新点B
                        // find nearer point
                        minPointSqDis2 = pointSqDis;
                        minPointInd2 = j;
                    }
                }

                // 寻找点O的另外一个最近邻的点B in the direction of decreasing scan line
                for (int j = closestPointInd - 1; j >= 0; --j) {
                    // if in the same scan line, continue
                    if (int(edgeFeaturesLast->points[j].intensity) >= closestPointScanID)
                        continue;

                    // if not in nearby scans, end the loop
                    if (int(edgeFeaturesLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                        break;

                    double pointSqDis = (edgeFeaturesLast->points[j].x - pointSel.x) *
                                        (edgeFeaturesLast->points[j].x - pointSel.x) +
                                        (edgeFeaturesLast->points[j].y - pointSel.y) *
                                        (edgeFeaturesLast->points[j].y - pointSel.y) +
                                        (edgeFeaturesLast->points[j].z - pointSel.z) *
                                        (edgeFeaturesLast->points[j].z - pointSel.z);

                    if (pointSqDis < minPointSqDis2) { // 第二个最近邻点有效，更新点B
                        // find nearer point
                        minPointSqDis2 = pointSqDis;
                        minPointInd2 = j;
                    }
                }
            }
            if (minPointInd2 >= 0) { // both closestPointInd and minPointInd2 is valid
                // 即特征点O的两个最近邻点A和B都有效
                Eigen::Vector3d curr_point(thisFrame->edgeFeatures->points[i].x,
                                           thisFrame->edgeFeatures->points[i].y,
                                           thisFrame->edgeFeatures->points[i].z);
                Eigen::Vector3d last_point_a(edgeFeaturesLast->points[closestPointInd].x,
                                             edgeFeaturesLast->points[closestPointInd].y,
                                             edgeFeaturesLast->points[closestPointInd].z);
                Eigen::Vector3d last_point_b(edgeFeaturesLast->points[minPointInd2].x,
                                             edgeFeaturesLast->points[minPointInd2].y,
                                             edgeFeaturesLast->points[minPointInd2].z);

                double s = (thisFrame->edgeFeatures->points[i].intensity - int(thisFrame->edgeFeatures->points[i].intensity)) * SCAN_FREQUENCY;
                // 用点O，A，B构造点到线的距离的残差项，注意这三个点都是在上一帧的Lidar坐标系下，即，残差 = 点O到直线AB的距离
                // 具体到介绍lidarFactor.cpp时再说明该残差的具体计算方法
                ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, last_point_a, last_point_b, s);
                problem.AddResidualBlock(cost_function, loss_function, para_q_2key, para_t_2key);
                edge_correspondence++;
            }
        }
        for (int i = 0; i < planePointSize; ++i) {
            undistortPoint(&(thisFrame->planeFeatures->points[i]), &pointSel);
            kdtreePlaneFeature->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

            int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
            if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD) { // 找到的最近邻点A有效
                closestPointInd = pointSearchInd[0];

                // get closest point's scan ID
                int closestPointScanID = int(planeFeaturesLast->points[closestPointInd].intensity);
                double minPointSqDis2 = DISTANCE_SQ_THRESHOLD, minPointSqDis3 = DISTANCE_SQ_THRESHOLD;

                // search in the direction of increasing scan line
                for (int j = closestPointInd + 1; j < (int)planeFeaturesLast->points.size(); ++j) {
                    // if not in nearby scans, end the loop
                    if (int(planeFeaturesLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                        break;

                    double pointSqDis = (planeFeaturesLast->points[j].x - pointSel.x) *
                                        (planeFeaturesLast->points[j].x - pointSel.x) +
                                        (planeFeaturesLast->points[j].y - pointSel.y) *
                                        (planeFeaturesLast->points[j].y - pointSel.y) +
                                        (planeFeaturesLast->points[j].z - pointSel.z) *
                                        (planeFeaturesLast->points[j].z - pointSel.z);

                    // if in the same or lower scan line
                    if (int(planeFeaturesLast->points[j].intensity) <= closestPointScanID && pointSqDis < minPointSqDis2) {
                        minPointSqDis2 = pointSqDis;// 找到的第2个最近邻点有效，更新点B，注意如果scanID准确的话，一般点A和点B的scanID相同
                        minPointInd2 = j;
                    }
                        // if in the higher scan line
                    else if (int(planeFeaturesLast->points[j].intensity) > closestPointScanID && pointSqDis < minPointSqDis3) {
                        minPointSqDis3 = pointSqDis;// 找到的第3个最近邻点有效，更新点C，注意如果scanID准确的话，一般点A和点B的scanID相同,且与点C的scanID不同，与LOAM的paper叙述一致
                        minPointInd3 = j;
                    }
                }

                // search in the direction of decreasing scan line
                for (int j = closestPointInd - 1; j >= 0; --j) {
                    // if not in nearby scans, end the loop
                    if (int(planeFeaturesLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                        break;

                    double pointSqDis = (planeFeaturesLast->points[j].x - pointSel.x) *
                                        (planeFeaturesLast->points[j].x - pointSel.x) +
                                        (planeFeaturesLast->points[j].y - pointSel.y) *
                                        (planeFeaturesLast->points[j].y - pointSel.y) +
                                        (planeFeaturesLast->points[j].z - pointSel.z) *
                                        (planeFeaturesLast->points[j].z - pointSel.z);

                    // if in the same or higher scan line
                    if (int(planeFeaturesLast->points[j].intensity) >= closestPointScanID && pointSqDis < minPointSqDis2) {
                        minPointSqDis2 = pointSqDis;
                        minPointInd2 = j;
                    }
                    else if (int(planeFeaturesLast->points[j].intensity) < closestPointScanID && pointSqDis < minPointSqDis3) {
                        // find nearer point
                        minPointSqDis3 = pointSqDis;
                        minPointInd3 = j;
                    }
                }

                if (minPointInd2 >= 0 && minPointInd3 >= 0) { // 如果三个最近邻点都有效

                    Eigen::Vector3d curr_point(thisFrame->planeFeatures->points[i].x,
                                               thisFrame->planeFeatures->points[i].y,
                                               thisFrame->planeFeatures->points[i].z);
                    Eigen::Vector3d last_point_a(planeFeaturesLast->points[closestPointInd].x,
                                                 planeFeaturesLast->points[closestPointInd].y,
                                                 planeFeaturesLast->points[closestPointInd].z);
                    Eigen::Vector3d last_point_b(planeFeaturesLast->points[minPointInd2].x,
                                                 planeFeaturesLast->points[minPointInd2].y,
                                                 planeFeaturesLast->points[minPointInd2].z);
                    Eigen::Vector3d last_point_c(planeFeaturesLast->points[minPointInd3].x,
                                                 planeFeaturesLast->points[minPointInd3].y,
                                                 planeFeaturesLast->points[minPointInd3].z);

                    double s = (thisFrame->planeFeatures->points[i].intensity - int(thisFrame->planeFeatures->points[i].intensity)) * SCAN_FREQUENCY;
                    // 用点O，A，B，C构造点到面的距离的残差项，注意这三个点都是在上一帧的Lidar坐标系下，即，残差 = 点O到平面ABC的距离
                    // 同样的，具体到介绍lidarFactor.cpp时再说明该残差的具体计算方法
                    ceres::CostFunction *cost_function = LidarPlaneFactor::Create(curr_point, last_point_a, last_point_b, last_point_c, s);
                    problem.AddResidualBlock(cost_function, loss_function, para_q_2key, para_t_2key);
                    plane_correspondence++;
                }
            }
        }
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 4;
        options.minimizer_progress_to_stdout = false;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
    }

    bool frameToBeKeyframe() {
        return toBeKeyframeInterval % 5;

        if (toBeKeyframeInterval < MIN_KEYFRAME_INTERVAL) return false;
        if (toBeKeyframeInterval > MAX_KEYFRAME_INTERVAL) {
            toBeKeyframeInterval = 0;
            return true;
        }

        Eigen::Vector3d eulerAngle = pose_delta2key.q.matrix().eulerAngles(2, 1, 0);

        bool isKeyframe = min(abs(eulerAngle.x()), abs(abs(eulerAngle.x()) - M_PI)) > keyframeAngleThreshold ||
                          min(abs(eulerAngle.y()), abs(abs(eulerAngle.y()) - M_PI)) > keyframeAngleThreshold ||
                          min(abs(eulerAngle.z()), abs(abs(eulerAngle.z()) - M_PI)) > keyframeAngleThreshold ||
                          pose_delta2key.t.norm() > keyframeDistThreshold;

//        printf("isKeyframe: %d [%f %f %f %f %d]",
//               isKeyframe,
//               min(abs(eulerAngle.x()), abs(abs(eulerAngle.x()) - M_PI)),
//               min(abs(eulerAngle.y()), abs(abs(eulerAngle.y()) - M_PI)),
//               min(abs(eulerAngle.z()), abs(abs(eulerAngle.z()) - M_PI)),
//               pose_delta2key.t.norm(),
//               toBeKeyframeInterval
//            );

        if (isKeyframe) {
            toBeKeyframeInterval = 0;
            return true;
        }
        return false;
    }

    void run() {
        ros::Rate rate(100);
        while (ros::ok()) {

            ros::spinOnce();

            if (!cornerSharpBuf.empty() && !cornerLessSharpBuf.empty() &&
                !surfFlatBuf.empty() && !surfLessFlatBuf.empty() && !fullPointsBuf.empty()) {

                timeCornerPointsSharp = cornerSharpBuf.front()->header.stamp.toSec();
                timeCornerPointsLessSharp = cornerLessSharpBuf.front()->header.stamp.toSec();
                timeSurfPointsFlat = surfFlatBuf.front()->header.stamp.toSec();
                timeSurfPointsLessFlat = surfLessFlatBuf.front()->header.stamp.toSec();
                timeLaserCloudFullRes = fullPointsBuf.front()->header.stamp.toSec();

                if (timeCornerPointsSharp != timeLaserCloudFullRes ||
                    timeCornerPointsLessSharp != timeLaserCloudFullRes ||
                    timeSurfPointsFlat != timeLaserCloudFullRes ||
                    timeSurfPointsLessFlat != timeLaserCloudFullRes ) {

                    printf("unsync messeage!");
                    ROS_BREAK();
                }

                {
                    std::lock_guard lockGuard(mBuf);

                    resetPCLPtr();
//                    cornerPointsSharp->clear();
                    pcl::fromROSMsg(*cornerSharpBuf.front(), *cornerPointsSharp);
                    cornerSharpBuf.pop();

//                    cornerPointsLessSharp->clear();
                    pcl::fromROSMsg(*cornerLessSharpBuf.front(), *cornerPointsLessSharp);
                    cornerLessSharpBuf.pop();

//                    surfPointsFlat->clear();
                    pcl::fromROSMsg(*surfFlatBuf.front(), *surfPointsFlat);
                    surfFlatBuf.pop();

//                    surfPointsLessFlat->clear();
                    pcl::fromROSMsg(*surfLessFlatBuf.front(), *surfPointsLessFlat);
                    surfLessFlatBuf.pop();

//                    laserCloudFullRes->clear();
                    pcl::fromROSMsg(*fullPointsBuf.front(), *laserCloudFullRes);
                    fullPointsBuf.pop();
                }

                Timer t_whole("running");

                auto newFrame = boost::make_shared<Frame>(cornerPointsSharp, surfPointsFlat, cornerPointsLessSharp, surfPointsLessFlat);
                currFrame = newFrame;

                if (!is_init) {
                    is_init = true;
                    std::cout << "Initialization finished \n";

                    lastFrame = lastKeyframe = currFrame;
                    currFrame->set_keyframe();
                    keyframeVec.push_back(frameCount);

                } else {

                    OPTIMIZATION_TIMES = 2;

                    for (int opti_counter = 0; opti_counter < OPTIMIZATION_TIMES; opti_counter++) {
                        getPose3BetweenFrames(lastFrame, currFrame, false);
                    }
                    currFrame->setTrans2LastFrame(q_curr2last, t_curr2last);
                    lastFrame = currFrame;

                    // accumulate △t_curr2last (between two close frame) to t_delta2key
                    pose_delta2key.multiply(q_curr2last, t_curr2last);

                    if (frameToBeKeyframe()) { // currFrame is key
                        currFrame->set_keyframe();
                        keyframeVec.push_back(frameCount);
                        OPTIMIZATION_TIMES = 4;

                        cout << "pose_delta2key: \n" << pose_delta2key.q.matrix() << "\n" << pose_delta2key.t << endl;
                        pose_delta2key.reset();
                    }

                    for (int opti_counter = 0; opti_counter < OPTIMIZATION_TIMES; opti_counter++) {
                        getPose3BetweenFrames(lastKeyframe, currFrame, true);
                    }
                    currFrame->setTrans2LastKeyframe(q_curr2key, t_curr2key);

                    if (currFrame->is_keyframe()) {
                        cout << "pose_curr2key: \n" << q_curr2key.matrix() << "\n" << t_curr2key << endl;
                        pose_key2world.multiply(q_curr2key, t_curr2key);
                        lastKeyframe = currFrame;

                        pose_curr2key2world = pose_key2world;
                    } else {
                        pose_curr2key2world = pose_key2world.dot(q_curr2key, t_curr2key);
                    }

                    pose_curr2world.multiply(q_curr2last, t_curr2last);

                }

                toBeKeyframeInterval++;

                if (currFrame->is_keyframe())
                    pWKeyframeToLastKeyframe.write(pose_key2world, true);
                pWFrameToLastFrame.write(pose_curr2world, currFrame->is_keyframe());
                pWFrameToLastKeyframe.write(pose_curr2key2world, currFrame->is_keyframe());

                frameMap[frameCount] = currFrame;

                // publish odometry
                odomWithoutKF.header.frame_id = "/camera_init";
                odomWithoutKF.child_frame_id = "/laser_odom";
                odomWithoutKF.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                odomWithoutKF.pose.pose.orientation.x = pose_curr2world.q.x();
                odomWithoutKF.pose.pose.orientation.y = pose_curr2world.q.y();
                odomWithoutKF.pose.pose.orientation.z = pose_curr2world.q.z();
                odomWithoutKF.pose.pose.orientation.w = pose_curr2world.q.w();
                odomWithoutKF.pose.pose.position.x = pose_curr2world.t.x();
                odomWithoutKF.pose.pose.position.y = pose_curr2world.t.y();
                odomWithoutKF.pose.pose.position.z = pose_curr2world.t.z();
                pubOdomWithoutKeyframe.publish(odomWithoutKF);

                odomWithKF.header.frame_id = "/camera_init";
                odomWithKF.child_frame_id = "/laser_odom";
                odomWithKF.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                odomWithKF.pose.pose.orientation.x = pose_curr2key2world.q.x();
                odomWithKF.pose.pose.orientation.y = pose_curr2key2world.q.y();
                odomWithKF.pose.pose.orientation.z = pose_curr2key2world.q.z();
                odomWithKF.pose.pose.orientation.w = pose_curr2key2world.q.w();
                odomWithKF.pose.pose.position.x = pose_curr2key2world.t.x();
                odomWithKF.pose.pose.position.y = pose_curr2key2world.t.y();
                odomWithKF.pose.pose.position.z = pose_curr2key2world.t.z();
                pubOdomWithKeyframe.publish(odomWithKF);

                geometry_msgs::PoseStamped laserPose;
                laserPose.header = odomWithoutKF.header;
                laserPose.pose = odomWithoutKF.pose.pose;
                pathWithoutKF.header.stamp = odomWithoutKF.header.stamp;
                pathWithoutKF.poses.push_back(laserPose);
                pathWithoutKF.header.frame_id = "/camera_init";
                pubPathWithoutKeyframe.publish(pathWithoutKF);

                laserPose.header = odomWithKF.header;
                laserPose.pose = odomWithKF.pose.pose;
                pathWithKF.header.stamp = odomWithKF.header.stamp;
                pathWithKF.poses.push_back(laserPose);
                pathWithKF.header.frame_id = "/camera_init";
                pubPathWithKeyframe.publish(pathWithKF);

                t_whole.count("whole laserOdometry time");
                frameCount++;
            }
            rate.sleep();
        }

    }

    void resetPCLPtr() {
        cornerPointsSharp.reset(new pcl::PointCloud<PointT>());
        cornerPointsLessSharp.reset(new pcl::PointCloud<PointT>());
        surfPointsFlat.reset(new pcl::PointCloud<PointT>());
        surfPointsLessFlat.reset(new pcl::PointCloud<PointT>());
        laserCloudFullRes.reset(new pcl::PointCloud<PointT>());
    }

    void allocateMemory() {
//        resetPCLPtr();

        kdtreeEdgeFeature.reset(new pcl::KdTreeFLANN<PointT>());
        kdtreePlaneFeature.reset(new pcl::KdTreeFLANN<PointT>());

        downsample_filter = Filter::getDownsampleFilter("VOXELGRID");
    }

    void setROSMesssageHandler() {
        subCornerPointsSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100, &Odometry::laserCloudSharpHandler, this);

        subCornerPointsLessSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100, &Odometry::laserCloudLessSharpHandler, this);

        subSurfPointsFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100, &Odometry::laserCloudFlatHandler, this);

        subSurfPointsLessFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100, &Odometry::laserCloudLessFlatHandler, this);

        subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100, &Odometry::laserCloudFullResHandler, this);

        pubPathWithKeyframe = nh.advertise<nav_msgs::Path>("/path_with_keyframe", 100);

        pubPathWithoutKeyframe = nh.advertise<nav_msgs::Path>("/path_without_keyframe", 100);

        pubOdomWithKeyframe = nh.advertise<nav_msgs::Odometry>("/odom_with_keyframe", 100);

        pubOdomWithoutKeyframe = nh.advertise<nav_msgs::Odometry>("/odom_without_keyframe", 100);

        pubLaserCloudCornerLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 100);

        pubLaserCloudSurfLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 100);

        pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_3", 100);
    }


    Odometry() :
        nh("~"),
        q_curr2last(para_q_2last),
        t_curr2last(para_t_2last),
        q_curr2key(para_q_2key),
        t_curr2key(para_t_2key),
        pWFrameToLastFrame("frame_to_last_frame.txt"),
        pWFrameToLastKeyframe("frame_to_last_keyframe.txt"),
        pWKeyframeToLastKeyframe("keyframe_to_last_keyframe.txt")
        {

        // init all T to identity
//        q_curr2key2world = q_key2world = q_delta2key = q_curr2world = Eigen::Quaterniond(1, 0, 0, 0);
//        t_curr2key2world = t_key2world = t_delta2key = t_curr2world = Eigen::Vector3d(0, 0, 0);

        need_undisortion = true;

        allocateMemory();

        setROSMesssageHandler();

        run();

    }

private:

    ros::NodeHandle nh;

    PoseWriter pWFrameToLastFrame;
    PoseWriter pWFrameToLastKeyframe;
    PoseWriter pWKeyframeToLastKeyframe;

    // current frame to world frame, accumulation of current frame to last frame
//    Eigen::Quaterniond q_curr2world;
//    Eigen::Vector3d t_curr2world;
    Pose3 pose_curr2world;

    // current frame to world frame, accumulation of current frame to last keyframe and last keyframe to world frame
//    Eigen::Quaterniond q_curr2key2world;
//    Eigen::Vector3d t_curr2key2world;
    Pose3 pose_curr2key2world;

    // last keyframe to world frame, accumulation of every keyframe
//    Eigen::Quaterniond q_key2world;
//    Eigen::Vector3d t_key2world;
    Pose3 pose_key2world;

    // current frame to last keyframe, accumulation of current frame to last frame
//    Eigen::Quaterniond q_delta2key;
//    Eigen::Vector3d t_delta2key;
    Pose3 pose_delta2key;

    // current frame to last frame
    double para_q_2last[4] = {0, 0, 0, 1};
    double para_t_2last[3] = {0, 0, 0};
    // current frame to last keyframe
    double para_q_2key[4] = {0, 0, 0, 1};
    double para_t_2key[3] = {0, 0, 0};
    // and their ref
    Eigen::Map<Eigen::Quaterniond> q_curr2last, q_curr2key;
    Eigen::Map<Eigen::Vector3d> t_curr2last, t_curr2key;

    std::queue<sensor_msgs::PointCloud2ConstPtr> cornerSharpBuf;
    std::queue<sensor_msgs::PointCloud2ConstPtr> cornerLessSharpBuf;
    std::queue<sensor_msgs::PointCloud2ConstPtr> surfFlatBuf;
    std::queue<sensor_msgs::PointCloud2ConstPtr> surfLessFlatBuf;
    std::queue<sensor_msgs::PointCloud2ConstPtr> fullPointsBuf;
    std::mutex mBuf;

    const int N_SCANS = 16;
    const int SCAN_FREQUENCY = 10;
    const int DISTANCE_SQ_THRESHOLD = 10;
    const double NEARBY_SCAN = 2.5;
    const int MAX_KEYFRAME_INTERVAL = 2;
    const int MIN_KEYFRAME_INTERVAL = 1;

    double keyframeDistThreshold = 1.0;
    double keyframeAngleThreshold = 0.2;

    bool is_init = false;
    int frameCount = 0;
    bool need_undisortion = false;
    int OPTIMIZATION_TIMES = 2;
    int toBeKeyframeInterval = 0;

    // subscriber topic from preprocessing
    ros::Subscriber subCornerPointsSharp;
    ros::Subscriber subCornerPointsLessSharp;
    ros::Subscriber subSurfPointsFlat;
    ros::Subscriber subSurfPointsLessFlat;
    ros::Subscriber subLaserCloudFullRes;


    ros::Publisher pubLaserCloudCornerLast;
    ros::Publisher pubLaserCloudSurfLast;
    ros::Publisher pubLaserCloudFullRes;
    ros::Publisher pubOdomWithoutKeyframe;
    ros::Publisher pubOdomWithKeyframe;
    ros::Publisher pubPathWithoutKeyframe;
    ros::Publisher pubPathWithKeyframe;

    nav_msgs::Path pathWithKF;
    nav_msgs::Path pathWithoutKF;
    nav_msgs::Odometry odomWithKF;
    nav_msgs::Odometry odomWithoutKF;

    double timeCornerPointsSharp = 0;
    double timeCornerPointsLessSharp = 0;
    double timeSurfPointsFlat = 0;
    double timeSurfPointsLessFlat = 0;
    double timeLaserCloudFullRes = 0;

    Frame::Ptr currFrame;
    Frame::Ptr lastKeyframe;
    Frame::Ptr lastFrame;

    std::unordered_map<int, Frame::Ptr> frameMap;
    std::vector<int> keyframeVec;

    // point_cloud current
    pcl::PointCloud<PointT>::Ptr cornerPointsSharp;
    pcl::PointCloud<PointT>::Ptr cornerPointsLessSharp;
    pcl::PointCloud<PointT>::Ptr surfPointsFlat;
    pcl::PointCloud<PointT>::Ptr surfPointsLessFlat;
    pcl::PointCloud<PointT>::Ptr laserCloudFullRes;

    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeEdgeFeature;
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreePlaneFeature;

    // pcl_utils
    pcl::Registration<PointT, PointT>::Ptr registration;
    pcl::Filter<PointT>::Ptr downsample_filter;
    pcl::Filter<PointT>::Ptr outlier_removal_filter;

};

int main(int argc, char **argv) {

    ros::init(argc, argv, "laserOdometry");

    Odometry odometry;

}
