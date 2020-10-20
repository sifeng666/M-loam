//
// Created by ziv on 2020/10/9.
//

#include "helper.h"
#include "pcl_utils.h"


class PreProcessing {

public:

    void pclFiltersInit() {
        std::string downsample_method = nh.param<std::string>("downsample_method", "VOXELGRID");
        double downsample_resolution = nh.param<double>("downsample_resolution", 0.1);
        downsample_filter = filter.getDownsampleFilter(downsample_method, downsample_resolution);

        std::string outlier_removal_method = nh.param<std::string>("outlier_removal_method", "RADIUS");
        outlier_removal_filter = filter.getOutlierRemovalFilter(outlier_removal_method);
    }


    void restrictPointDistance(const pcl::PointCloud<PointT>::Ptr &cloud_in, pcl::PointCloud<PointT>::Ptr& cloud_out,
                               double distance_near_thresh = 0.0, double distance_far_thresh = 250.0) {

        cloud_out->resize(cloud_in->size());
        size_t j = 0;

        for (size_t i = 0; i < cloud_in->points.size(); ++i) {
            auto dis = cloud_in->points[i].x * cloud_in->points[i].x + cloud_in->points[i].y * cloud_in->points[i].y + cloud_in->points[i].z * cloud_in->points[i].z;

            if (dis < distance_near_thresh * distance_near_thresh || dis > distance_far_thresh * distance_far_thresh)
                continue;
            cloud_out->points[j++] = cloud_in->points[i];
        }

        if (j != cloud_in->points.size()) {
            cloud_out->points.resize(j);
        }

        cloud_out->height = 1;
        cloud_out->width = static_cast<uint32_t>(j);
        cloud_out->is_dense = true;
        cloud_out->header = cloud_in->header;

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


//    void pointCloudHandler2(const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg) {
//
//        Timer t_whole("handler2");
//
//        pcl::PointCloud<PointT>::Ptr cloud_in(new pcl::PointCloud<PointT>());
//        pcl::fromROSMsg(*point_cloud_msg, *cloud_in);
//
//        // remove nan points and points that too close and too far
//        std::vector<int> dummies;
//
//        printf("size of cloud origin: %d \n", cloud_in->size());
//
//        pcl::PointCloud<PointT>::Ptr cloud_out(new pcl::PointCloud<PointT>());
//        pcl::removeNaNFromPointCloud(*cloud_in, *cloud_out, dummies);
//        restrictPointDistance(cloud_out, cloud_out, distance_near_thresh, distance_far_thresh);
//        downsample(cloud_out, cloud_out);
//        outlier_removal(cloud_out, cloud_out);
//
//        sensor_msgs::PointCloud2 laserCloudFilteredMsg;
//        pcl::toROSMsg(*cloud_out, laserCloudFilteredMsg);
//        laserCloudFilteredMsg.header.stamp = point_cloud_msg->header.stamp;
//        laserCloudFilteredMsg.header.frame_id = "/camera_init";
//        laserCloudFilteredMsg.header.frame_id = point_cloud_msg->header.frame_id;
//        pubLaserCloudFiltered.publish(laserCloudFilteredMsg);
//
////        t_whole.count_from_last("remove unused points");
////        printf("size of cloud remove unused points: %d \n", cloud_out->size());
////
////        pcl::PointCloud<PointT>::Ptr cloud_out_ds(new pcl::PointCloud<PointT>());
////        downsample(cloud_out, cloud_out_ds);
////        t_whole.count_from_last("downsample");
////        printf("size of cloud downsample: %d \n", cloud_out_ds->size());
////
////        pcl::PointCloud<PointT>::Ptr cloud_out_ro(new pcl::PointCloud<PointT>());
////        outlier_removal(cloud_out_ds, cloud_out_ro);
////        t_whole.count_from_last("outlier_removal");
////        printf("size of cloud outlier_removal: %d \n", cloud_out_ro->size());
////
////        sensor_msgs::PointCloud2 laserCloudOutRUMsg;
////        pcl::toROSMsg(*cloud_out, laserCloudOutRUMsg);
////        laserCloudOutRUMsg.header.stamp = point_cloud_msg->header.stamp;
////        laserCloudOutRUMsg.header.frame_id = "/camera_init";
////        laserCloudOutRUMsg.header.frame_id = point_cloud_msg->header.frame_id;
////        pubLaserCloudRU.publish(laserCloudOutRUMsg); // /remove_unused_points
////
////        sensor_msgs::PointCloud2 laserCloudOutDSMsg;
////        pcl::toROSMsg(*cloud_out_ds, laserCloudOutDSMsg);
////        laserCloudOutDSMsg.header.stamp = point_cloud_msg->header.stamp;
////        laserCloudOutDSMsg.header.frame_id = "/camera_init";
////        laserCloudOutDSMsg.header.frame_id = point_cloud_msg->header.frame_id;
////        pubLaserCloudDS.publish(laserCloudOutDSMsg); // /downsample_points
////
////        sensor_msgs::PointCloud2 laserCloudOutROMsg;
////        pcl::toROSMsg(*cloud_out_ro, laserCloudOutROMsg);
////        laserCloudOutROMsg.header.stamp = point_cloud_msg->header.stamp;
////        laserCloudOutROMsg.header.frame_id = "/camera_init";
////        laserCloudOutROMsg.header.frame_id = point_cloud_msg->header.frame_id;
////        pubLaserCloudRO.publish(laserCloudOutROMsg); // /remove_outlier_points
//
//    }

    double getTwoPointsAngle(const PointT& p1, const PointT& p2) {
        Eigen::Vector3d v1(p1.x, p1.y, p1.z);
        Eigen::Vector3d v2(p2.x, p2.y, p2.z);
        double radian_angle = std::atan2(v1.cross(v2).norm(), v1.transpose() * v2);
        if (v1.cross(v2).z() < 0) {
            radian_angle = 2 * M_PI - radian_angle;
        }
        return radian_angle;
    }

    void pointCloudHandler(const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg) {

//        pointCloudHandler2(point_cloud_msg);

        Timer t_whole("handler1");

        std::vector<int> scanStartInd(N_SCANS, 0);
        std::vector<int> scanEndInd(N_SCANS, 0);
        std::vector<int> dummies;

        pcl::PointCloud<PointT>::Ptr cloud_in(new pcl::PointCloud<PointT>());
        pcl::fromROSMsg(*point_cloud_msg, *cloud_in);

        // remove nan points and points that too close and too far
        pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, dummies);
        restrictPointDistance(cloud_in, cloud_in, distance_near_thresh, distance_far_thresh);
//        t_whole.count_from_last("remove unused points");

        // split into 16 scans (from loam)
        int cloudSize = cloud_in->points.size();
        float startOri = -atan2(cloud_in->points[0].y, cloud_in->points[0].x);
        float endOri = -atan2(cloud_in->points[cloudSize - 1].y,
                              cloud_in->points[cloudSize - 1].x) + 2 * M_PI;

        if (endOri - startOri > 3 * M_PI) {
            endOri -= 2 * M_PI;
        } else if (endOri - startOri < M_PI) {
            endOri += 2 * M_PI;
        }
        bool halfPassed = false;
        int count = cloudSize;
        PointT point;
        std::vector<pcl::PointCloud<PointT>> pointCloudScans(N_SCANS);
        for (int i = 0; i < cloudSize; ++i) {
            point.x = cloud_in->points[i].x;
            point.y = cloud_in->points[i].y;
            point.z = cloud_in->points[i].z;

            float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
            int scanID = 0;

            scanID = int((angle + 15) / 2 + 0.5);
            // 去除 <-16度与 > 16度 的离群点
            if (scanID > (N_SCANS - 1) || scanID < 0) {
                count--;
                continue;
            }

            float ori = -atan2(point.y, point.x);
            if (!halfPassed) {
                if (ori < startOri - M_PI / 2) {
                    ori += 2 * M_PI;
                } else if (ori > startOri + M_PI * 3 / 2) {
                    ori -= 2 * M_PI;
                }

                if (ori - startOri > M_PI) {
                    halfPassed = true;
                }
            } else {
                ori += 2 * M_PI;

                if (ori < endOri - M_PI * 3 / 2) {
                    ori += 2 * M_PI;
                } else if (ori > endOri + M_PI / 2) {
                    ori -= 2 * M_PI;
                }
            }

            float relTime = (ori - startOri) / (endOri - startOri);
            point.intensity = scanID + 1.0 / SCAN_FREQUENCY * relTime;
            pointCloudScans[scanID].push_back(point);
        }
//        t_whole.count_from_last("split into 16 scans");

        cloudSize = count;
//        printf("points size %d \n", cloudSize);

        pcl::PointCloud<PointT>::Ptr laserCloud(new pcl::PointCloud<PointT>());
        for (int i = 0; i < N_SCANS; i++) {
            scanStartInd[i] = laserCloud->size() + 5;// 记录每个scan的开始index，忽略前5个点
            *laserCloud += pointCloudScans[i];
            scanEndInd[i] = laserCloud->size() - 6;// 记录每个scan的结束index，忽略后5个点，开始和结束处的点云scan容易产生不闭合的“接缝”，对提取edge feature不利
        }
//        t_whole.count_from_last("points prepare");

        for (int i = 5; i < cloudSize - 5; i++) {
            float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x + laserCloud->points[i + 5].x;
            float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y + laserCloud->points[i + 5].y;
            float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z + laserCloud->points[i + 5].z;

            cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
            cloudSortInd[i] = i;
            cloudNeighborPicked[i] = 0;// 点有没有被选选择为feature点
            cloudLabel[i] = 0;
            // Label 2: corner_sharp
            // Label 1: corner_less_sharp, 包含Label 2
            // Label -1: surf_flat
            // Label 0: surf_less_flat， 包含Label -1，因为点太多，最后会降采样
        }
//        t_whole.count_from_last("cal all points' curvature");


        //挑选点，排除容易被斜面挡住的点以及离群点，有些点容易被斜面挡住，而离群点可能出现带有偶然性，这些情况都可能导致前后两次扫描不能被同时看到
//        for (int i = 5; i < cloudSize - 6; i++) { //与后一个点差值，所以减6
//            float diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x;
//            float diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y;
//            float diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z;
//            //计算有效曲率点与后一个点之间的距离平方和
//            float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;
//
//            if (diff > 0.1) {//前提:两个点之间距离要大于0.1
//
//                //点的深度
//                float depth1 = sqrt(laserCloud->points[i].x * laserCloud->points[i].x +
//                                    laserCloud->points[i].y * laserCloud->points[i].y +
//                                    laserCloud->points[i].z * laserCloud->points[i].z);
//
//                //后一个点的深度
//                float depth2 = sqrt(laserCloud->points[i + 1].x * laserCloud->points[i + 1].x +
//                                    laserCloud->points[i + 1].y * laserCloud->points[i + 1].y +
//                                    laserCloud->points[i + 1].z * laserCloud->points[i + 1].z);
//
//                //按照两点的深度的比例，将深度较大的点拉回后计算距离
//                if (depth1 > depth2) {
//                    diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x * depth2 / depth1;
//                    diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y * depth2 / depth1;
//                    diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z * depth2 / depth1;
//
//                    //边长比也即是弧度值，若小于0.1，说明夹角比较小，斜面比较陡峭,点深度变化比较剧烈,点处在近似与激光束平行的斜面上
//                    if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth2 < 0.1) {//排除容易被斜面挡住的点
//                        //该点及前面五个点（大致都在斜面上）全部置为筛选过
//                        cloudNeighborPicked[i - 5] = 1;
//                        cloudNeighborPicked[i - 4] = 1;
//                        cloudNeighborPicked[i - 3] = 1;
//                        cloudNeighborPicked[i - 2] = 1;
//                        cloudNeighborPicked[i - 1] = 1;
//                        cloudNeighborPicked[i] = 1;
//                    }
//                } else {
//                    diffX = laserCloud->points[i + 1].x * depth1 / depth2 - laserCloud->points[i].x;
//                    diffY = laserCloud->points[i + 1].y * depth1 / depth2 - laserCloud->points[i].y;
//                    diffZ = laserCloud->points[i + 1].z * depth1 / depth2 - laserCloud->points[i].z;
//
//                    if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth1 < 0.1) {
//                        cloudNeighborPicked[i + 1] = 1;
//                        cloudNeighborPicked[i + 2] = 1;
//                        cloudNeighborPicked[i + 3] = 1;
//                        cloudNeighborPicked[i + 4] = 1;
//                        cloudNeighborPicked[i + 5] = 1;
//                        cloudNeighborPicked[i + 6] = 1;
//                    }
//                }
//            }
//
//            float diffX2 = laserCloud->points[i].x - laserCloud->points[i - 1].x;
//            float diffY2 = laserCloud->points[i].y - laserCloud->points[i - 1].y;
//            float diffZ2 = laserCloud->points[i].z - laserCloud->points[i - 1].z;
//            //与前一个点的距离平方和
//            float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;
//
//            //点深度的平方和
//            float dis = laserCloud->points[i].x * laserCloud->points[i].x
//                        + laserCloud->points[i].y * laserCloud->points[i].y
//                        + laserCloud->points[i].z * laserCloud->points[i].z;
//
//            //与前后点的平方和都大于深度平方和的万分之二，这些点视为离群点，包括陡斜面上的点，强烈凸凹点和空旷区域中的某些点，置为筛选过，弃用
//            if (diff > 0.0002 * dis && diff2 > 0.0002 * dis) {
//                cloudNeighborPicked[i] = 1;
//            }
//        }
//
//        t_whole.count_from_last("loam's outlier removal");

        for (int i = 5; i < cloudSize - 6; i++) { //与后一个点差值，所以减6
            float diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x;
            float diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y;
            float diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z;
            //计算有效曲率点与后一个点之间的距离平方和
            float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;

            if (diff > 0.1) {//前提:两个点之间距离要大于0.1

                double angle = getTwoPointsAngle(laserCloud->points[i], laserCloud->points[i + 1]);
                //点的深度
                float depth1 = sqrt(laserCloud->points[i].x * laserCloud->points[i].x +
                                    laserCloud->points[i].y * laserCloud->points[i].y +
                                    laserCloud->points[i].z * laserCloud->points[i].z);

                //后一个点的深度
                float depth2 = sqrt(laserCloud->points[i + 1].x * laserCloud->points[i + 1].x +
                                    laserCloud->points[i + 1].y * laserCloud->points[i + 1].y +
                                    laserCloud->points[i + 1].z * laserCloud->points[i + 1].z);

                if (angle < 0.1) {
                    if (depth1 > depth2) {
                        cloudNeighborPicked[i - 5] = 1;
                        cloudNeighborPicked[i - 4] = 1;
                        cloudNeighborPicked[i - 3] = 1;
                        cloudNeighborPicked[i - 2] = 1;
                        cloudNeighborPicked[i - 1] = 1;
                        cloudNeighborPicked[i] = 1;
                    } else {
                        cloudNeighborPicked[i + 1] = 1;
                        cloudNeighborPicked[i + 2] = 1;
                        cloudNeighborPicked[i + 3] = 1;
                        cloudNeighborPicked[i + 4] = 1;
                        cloudNeighborPicked[i + 5] = 1;
                        cloudNeighborPicked[i + 6] = 1;
                    }
                }
            }

            float diffX2 = laserCloud->points[i].x - laserCloud->points[i - 1].x;
            float diffY2 = laserCloud->points[i].y - laserCloud->points[i - 1].y;
            float diffZ2 = laserCloud->points[i].z - laserCloud->points[i - 1].z;
            //与前一个点的距离平方和
            float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;

            //点深度的平方和
            float dis = laserCloud->points[i].x * laserCloud->points[i].x
                        + laserCloud->points[i].y * laserCloud->points[i].y
                        + laserCloud->points[i].z * laserCloud->points[i].z;

            //与前后点的平方和都大于深度平方和的万分之二，这些点视为离群点，包括陡斜面上的点，强烈凸凹点和空旷区域中的某些点，置为筛选过，弃用
            if (diff > 0.0002 * dis && diff2 > 0.0002 * dis) {
                cloudNeighborPicked[i] = 1;
            }
        }

//        t_whole.count_from_last("my outlier removal");

        pcl::PointCloud<PointT> cornerPointsSharp;
        pcl::PointCloud<PointT> cornerPointsLessSharp;
        pcl::PointCloud<PointT> surfPointsFlat;
        pcl::PointCloud<PointT> surfPointsLessFlat;

//        size_t t_q_sort = 0;
        for (int i = 0; i < N_SCANS; i++) {
            if( scanEndInd[i] - scanStartInd[i] < 6) // 如果该scan的点数少于7个点，就跳过
                continue;
            pcl::PointCloud<PointT>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointT>);
            for (int j = 0; j < 6; j++) {// 将该scan分成6小段执行特征检测

                int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6;// subscan的起始index
                int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;// subscan的结束index

//                TimeCounter t_sorter;
                std::sort (cloudSortInd + sp, cloudSortInd + ep + 1, [&](int i, int j) { return (cloudCurvature[i]<cloudCurvature[j]); }); // 根据曲率有小到大对subscan的点进行sort
//                t_q_sort += t_sorter.count();

                int largestPickedNum = 0;
                for (int k = ep; k >= sp; k--) {
                    int ind = cloudSortInd[k];

                    if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > 0.1)  {
                        largestPickedNum++;
                        if (largestPickedNum <= 2) { // 该subscan中曲率最大的前2个点认为是corner_sharp特征点
                            cloudLabel[ind] = 2;
                            cornerPointsSharp.push_back(laserCloud->points[ind]);
                            cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                        }
                        else if (largestPickedNum <= 20) { // 该subscan中曲率最大的前20个点认为是corner_less_sharp特征点
                            cloudLabel[ind] = 1;
                            cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                        }
                        else break;

                        cloudNeighborPicked[ind] = 1;// 标记该点被选择过了

                        // 与当前点距离的平方 <= 0.05的点标记为选择过，避免特征点密集分布
                        for (int l = 1; l <= 5; l++) {
                            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                                break;
                            }

                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--)
                        {
                            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                                break;
                            }

                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                // 提取surf平面feature，与上述类似，选取该subscan曲率最小的前4个点为surf_flat
                int smallestPickedNum = 0;
                for (int k = sp; k <= ep; k++) {
                    int ind = cloudSortInd[k];

                    if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < 0.1) {

                        cloudLabel[ind] = -1;
                        surfPointsFlat.push_back(laserCloud->points[ind]);

                        smallestPickedNum++;
                        if (smallestPickedNum >= 4) {
                            break;
                        }

                        cloudNeighborPicked[ind] = 1;
                        for (int l = 1; l <= 5; l++) {
                            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                                break;
                            }

                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--) {
                            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                                break;
                            }

                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                // 其他的非corner特征点与surf_flat特征点一起组成surf_less_flat特征点
                for (int k = sp; k <= ep; k++) {
                    if (cloudLabel[k] <= 0) {
                        surfPointsLessFlatScan->push_back(laserCloud->points[k]);
                    }
                }
            }

            // 最后对该scan点云中提取的所有surf_less_flat特征点进行降采样，因为点太多了
            pcl::PointCloud<PointT> surfPointsLessFlatScanDS;
            pcl::VoxelGrid<PointT> downSizeFilter;
            downSizeFilter.setInputCloud(surfPointsLessFlatScan);
            downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
            downSizeFilter.filter(surfPointsLessFlatScanDS);

            surfPointsLessFlat += surfPointsLessFlatScanDS;
        }

//        printf("sort q time %lld \n", t_q_sort);
//        t_whole.count_from_last("points seperated");

        sensor_msgs::PointCloud2 laserCloudOutMsg;
        pcl::toROSMsg(*laserCloud, laserCloudOutMsg);
        laserCloudOutMsg.header.stamp = point_cloud_msg->header.stamp;
        laserCloudOutMsg.header.frame_id = "/camera_init";
        laserCloudOutMsg.header.frame_id = point_cloud_msg->header.frame_id;
        pubLaserCloud.publish(laserCloudOutMsg); // /velodyne_cloud_2

        sensor_msgs::PointCloud2 cornerPointsSharpMsg;
        pcl::toROSMsg(cornerPointsSharp, cornerPointsSharpMsg);
        cornerPointsSharpMsg.header.stamp = point_cloud_msg->header.stamp;
        cornerPointsSharpMsg.header.frame_id = "/camera_init";
        cornerPointsSharpMsg.header.frame_id = point_cloud_msg->header.frame_id;
        pubCornerPointsSharp.publish(cornerPointsSharpMsg); // /laser_cloud_sharp

        sensor_msgs::PointCloud2 cornerPointsLessSharpMsg;
        pcl::toROSMsg(cornerPointsLessSharp, cornerPointsLessSharpMsg);
        cornerPointsLessSharpMsg.header.stamp = point_cloud_msg->header.stamp;
        cornerPointsLessSharpMsg.header.frame_id = "/camera_init";
        cornerPointsLessSharpMsg.header.frame_id = point_cloud_msg->header.frame_id;
        pubCornerPointsLessSharp.publish(cornerPointsLessSharpMsg); // /laser_cloud_less_sharp

        sensor_msgs::PointCloud2 surfPointsFlat2;
        pcl::toROSMsg(surfPointsFlat, surfPointsFlat2);
        surfPointsFlat2.header.stamp = point_cloud_msg->header.stamp;
        surfPointsFlat2.header.frame_id = "/camera_init";
        surfPointsFlat2.header.frame_id = point_cloud_msg->header.frame_id;
        pubSurfPointsFlat.publish(surfPointsFlat2); // /laser_cloud_flat

        sensor_msgs::PointCloud2 surfPointsLessFlat2;
        pcl::toROSMsg(surfPointsLessFlat, surfPointsLessFlat2);
        surfPointsLessFlat2.header.stamp = point_cloud_msg->header.stamp;
        surfPointsLessFlat2.header.frame_id = "/camera_init";
        surfPointsLessFlat2.header.frame_id = point_cloud_msg->header.frame_id;
        pubSurfPointsLessFlat.publish(surfPointsLessFlat2); // /laser_cloud_less_flat

        t_whole.count("scan registration time");
    }

    PreProcessing() : nh("~") {

        use_distance_filter = nh.param<bool>("use_distance_filter", true);
        distance_near_thresh = nh.param<double>("distance_near_thresh", 0.1);
        distance_far_thresh = nh.param<double>("distance_far_thresh", 100.0);
        base_link_frame = nh.param<std::string>("base_link_frame", "");

        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, &PreProcessing::pointCloudHandler, this);

        pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100);

        pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100);

        pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100);

        pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100);

        pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100);

//        pubLaserCloudRU = nh.advertise<sensor_msgs::PointCloud2>("/remove_unused_points", 100);
//
//        pubLaserCloudDS = nh.advertise<sensor_msgs::PointCloud2>("/downsample_points", 100);
//
//        pubLaserCloudRO = nh.advertise<sensor_msgs::PointCloud2>("/remove_outlier_points", 100);
//
//        pubLaserCloudFiltered = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 100);

//        pubRemovePoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_remove_points", 100);

        pclFiltersInit();
    }

private:
    ros::NodeHandle nh;

    ros::Subscriber subLaserCloud; // origin points sub

    ros::Publisher  pubLaserCloud;
    ros::Publisher  pubCornerPointsSharp;
    ros::Publisher  pubCornerPointsLessSharp;
    ros::Publisher  pubSurfPointsFlat;
    ros::Publisher  pubSurfPointsLessFlat;

    ros::Publisher  pubLaserCloudRU;
    ros::Publisher  pubLaserCloudDS;
    ros::Publisher  pubLaserCloudRO;
    ros::Publisher  pubLaserCloudFiltered;

    tf::TransformListener tf_listener;

    std::string base_link_frame;

    bool use_distance_filter;
    double distance_near_thresh;
    double distance_far_thresh;

    const int N_SCANS = 16;
    const int SCAN_FREQUENCY = 10;

    Filter filter;
    pcl::Filter<PointT>::Ptr downsample_filter;
    pcl::Filter<PointT>::Ptr outlier_removal_filter;

    float cloudCurvature[100000];
    int cloudSortInd[100000];
    int cloudNeighborPicked[100000];
    int cloudLabel[100000];
};

int main(int argc, char **argv) {

    ros::init(argc, argv, "preProcessing");

    PreProcessing preProcessing;
    ros::spin();

}