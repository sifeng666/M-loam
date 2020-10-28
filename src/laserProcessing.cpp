//
// Created by ziv on 2020/10/26.
//

#include "helper.h"

#include <pcl/filters/radius_outlier_removal.h>

class LaserProcessing {

public:
    using IndexVal = std::pair<int, double>; // first: index, second: value
public:

    void initROSParam() {

        scan_period     = nh.param<double>("scan_period", 0.1);

        vertical_angle  = nh.param<double>("vertical_angle", 2.0);

        max_dis         = nh.param<double>("max_dis", 90.0);

        min_dis         = nh.param<double>("min_dis", 2.0);

        scan_line       = nh.param<int>("scan_line", 16);

    }

    void initROSHandler() {

        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, &LaserProcessing::pointCloudHandler, this);

        pubLaserCloudFiltered = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_2", 100);

        pubEdgePoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_edge", 100);

        pubSurfPoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf", 100);

    }

    void pointCloudHandler(const sensor_msgs::PointCloud2ConstPtr &pointCloudMsg) {
        std::lock_guard lg(mtx);
        pointCloudBuf.push(pointCloudMsg);
    }

    void featureExtractionFromSector(const pcl::PointCloud<PointT>::Ptr& cloud_in, std::vector<IndexVal>& cloudCurvature, pcl::PointCloud<PointT>::Ptr& cloud_out_edge, pcl::PointCloud<PointT>::Ptr& cloud_out_surf) {

        std::sort(cloudCurvature.begin(), cloudCurvature.end(), [](const IndexVal& a, const IndexVal& b) {
            return a.second < b.second;
        });


        int largestPickedNum = 0;
        std::vector<int> picked_points;
        int point_info_count =0;
        for (int i = cloudCurvature.size()-1; i >= 0; i--) {
            int ind = cloudCurvature[i].first;
            if (std::find(picked_points.begin(), picked_points.end(), ind) == picked_points.end()) {
                if(cloudCurvature[i].second <= 0.1) {
                    break;
                }

                largestPickedNum++;
                picked_points.push_back(ind);

                if (largestPickedNum <= 20) {
                    cloud_out_edge->push_back(cloud_in->points[ind]);
                    point_info_count++;
                } else {
                    break;
                }

                for (int k=1; k<=5; k++) {
                    double diffX = cloud_in->points[ind + k].x - cloud_in->points[ind + k - 1].x;
                    double diffY = cloud_in->points[ind + k].y - cloud_in->points[ind + k - 1].y;
                    double diffZ = cloud_in->points[ind + k].z - cloud_in->points[ind + k - 1].z;
                    if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                        break;
                    }
                    picked_points.push_back(ind+k);
                }
                for (int k=-1; k>=-5; k--) {
                    double diffX = cloud_in->points[ind + k].x - cloud_in->points[ind + k + 1].x;
                    double diffY = cloud_in->points[ind + k].y - cloud_in->points[ind + k + 1].y;
                    double diffZ = cloud_in->points[ind + k].z - cloud_in->points[ind + k + 1].z;
                    if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                        break;
                    }
                    picked_points.push_back(ind+k);
                }

            }
        }

        //find flat points
        // point_info_count =0;
        // int smallestPickedNum = 0;

        // for (int i = 0; i <= (int)cloudCurvature.size()-1; i++)
        // {
        //     int ind = cloudCurvature[i].id;

        //     if( std::find(picked_points.begin(), picked_points.end(), ind)==picked_points.end()){
        //         if(cloudCurvature[i].value > 0.1){
        //             //ROS_WARN("extracted feature not qualified, please check lidar");
        //             break;
        //         }
        //         smallestPickedNum++;
        //         picked_points.push_back(ind);

        //         if(smallestPickedNum <= 4){
        //             //find all points
        //             pc_surf_flat->push_back(pc_in->points[ind]);
        //             pc_surf_lessFlat->push_back(pc_in->points[ind]);
        //             point_info_count++;
        //         }
        //         else{
        //             break;
        //         }

        //         for(int k=1;k<=5;k++){
        //             double diffX = pc_in->points[ind + k].x - pc_in->points[ind + k - 1].x;
        //             double diffY = pc_in->points[ind + k].y - pc_in->points[ind + k - 1].y;
        //             double diffZ = pc_in->points[ind + k].z - pc_in->points[ind + k - 1].z;
        //             if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05){
        //                 break;
        //             }
        //             picked_points.push_back(ind+k);
        //         }
        //         for(int k=-1;k>=-5;k--){
        //             double diffX = pc_in->points[ind + k].x - pc_in->points[ind + k + 1].x;
        //             double diffY = pc_in->points[ind + k].y - pc_in->points[ind + k + 1].y;
        //             double diffZ = pc_in->points[ind + k].z - pc_in->points[ind + k + 1].z;
        //             if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05){
        //                 break;
        //             }
        //             picked_points.push_back(ind+k);
        //         }

        //     }
        // }

        for (int i = 0; i <= (int)cloudCurvature.size()-1; i++) {
            int ind = cloudCurvature[i].first;
            if(std::find(picked_points.begin(), picked_points.end(), ind) == picked_points.end()) {
                cloud_out_surf->push_back(cloud_in->points[ind]);
            }
        }
    }

    void featureExtraction(const pcl::PointCloud<PointT>::Ptr& cloud_in, pcl::PointCloud<PointT>::Ptr& cloud_out_edge, pcl::PointCloud<PointT>::Ptr& cloud_out_surf) {
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud_in, indices);

        int N_SCANS = scan_line;
        std::vector<pcl::PointCloud<PointT>::Ptr> laserCloudScans;
        for(int i = 0; i < N_SCANS; i++){
            laserCloudScans.push_back(pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>()));
        }

        for (int i = 0; i < (int)cloud_in->points.size(); i++) {
            int scanID = 0;
            double dis_XY = cloud_in->points[i].x * cloud_in->points[i].x + cloud_in->points[i].y * cloud_in->points[i].y;
            double dis = dis_XY + cloud_in->points[i].z * cloud_in->points[i].z;
            if (dis < min_dis * min_dis || dis > max_dis * max_dis) {
                continue;
            }
            double angle = atan(cloud_in->points[i].z / sqrt(dis_XY)) * 180 / M_PI;

            if (N_SCANS == 16) {
                scanID = int((angle + 15) / 2 + 0.5);
                if (scanID > (N_SCANS - 1) || scanID < 0) {
                    continue;
                }
            }
            else if (N_SCANS == 32) {
                scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
                if (scanID > (N_SCANS - 1) || scanID < 0) {
                    continue;
                }
            }
            else if (N_SCANS == 64) {
                if (angle >= -8.83) {
                    scanID = int((2 - angle) * 3.0 + 0.5);
                } else {
                    scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);
                }
                if (angle > 2 || angle < -24.33 || scanID > 63 || scanID < 0) {
                    continue;
                }
            }
            else {
                printf("wrong scan number\n");
            }
            laserCloudScans[scanID]->push_back(cloud_in->points[i]);

        }

        for (int i = 0; i < N_SCANS; i++) {
            if (laserCloudScans[i]->points.size() < 131) {
                continue;
            }

            std::vector<IndexVal> cloudCurvature;
            int total_points = laserCloudScans[i]->points.size() - 10;

            for (int j = 5; j < (int)laserCloudScans[i]->points.size() - 5; j++) {
                double diffX = laserCloudScans[i]->points[j - 5].x + laserCloudScans[i]->points[j - 4].x + laserCloudScans[i]->points[j - 3].x + laserCloudScans[i]->points[j - 2].x + laserCloudScans[i]->points[j - 1].x - 10 * laserCloudScans[i]->points[j].x + laserCloudScans[i]->points[j + 1].x + laserCloudScans[i]->points[j + 2].x + laserCloudScans[i]->points[j + 3].x + laserCloudScans[i]->points[j + 4].x + laserCloudScans[i]->points[j + 5].x;
                double diffY = laserCloudScans[i]->points[j - 5].y + laserCloudScans[i]->points[j - 4].y + laserCloudScans[i]->points[j - 3].y + laserCloudScans[i]->points[j - 2].y + laserCloudScans[i]->points[j - 1].y - 10 * laserCloudScans[i]->points[j].y + laserCloudScans[i]->points[j + 1].y + laserCloudScans[i]->points[j + 2].y + laserCloudScans[i]->points[j + 3].y + laserCloudScans[i]->points[j + 4].y + laserCloudScans[i]->points[j + 5].y;
                double diffZ = laserCloudScans[i]->points[j - 5].z + laserCloudScans[i]->points[j - 4].z + laserCloudScans[i]->points[j - 3].z + laserCloudScans[i]->points[j - 2].z + laserCloudScans[i]->points[j - 1].z - 10 * laserCloudScans[i]->points[j].z + laserCloudScans[i]->points[j + 1].z + laserCloudScans[i]->points[j + 2].z + laserCloudScans[i]->points[j + 3].z + laserCloudScans[i]->points[j + 4].z + laserCloudScans[i]->points[j + 5].z;

                double curvature = diffX * diffX + diffY * diffY + diffZ * diffZ;
                cloudCurvature.emplace_back(j, curvature);
            }
            for (int j = 0; j < 6; j++) {
                int sector_length = (int)(total_points/6);
                int sector_start = sector_length * j;
                int sector_end = sector_length * (j+1)-1;
                if (j == 5) {
                    sector_end = total_points - 1;
                }
                std::vector<IndexVal> subCloudCurvature(cloudCurvature.begin()+sector_start, cloudCurvature.begin()+sector_end);

                featureExtractionFromSector(laserCloudScans[i], subCloudCurvature, cloud_out_edge, cloud_out_surf);

            }

        }
    }

    void laser_processing() {

        while (ros::ok()) {

            if (!pointCloudBuf.empty()) {
                pcl::PointCloud<PointT>::Ptr cloud_in(new pcl::PointCloud<PointT>());
                pcl::PointCloud<PointT>::Ptr cloud_edge(new pcl::PointCloud<PointT>());
                pcl::PointCloud<PointT>::Ptr cloud_surf(new pcl::PointCloud<PointT>());

                mtx.lock();
                pcl::fromROSMsg(*pointCloudBuf.front(), *cloud_in);
                ros::Time cloud_in_time = (pointCloudBuf.front())->header.stamp;
                pointCloudBuf.pop();
                mtx.unlock();


                Timer feat_extra("featureExtraction");
                featureExtraction(cloud_in, cloud_edge, cloud_surf);
                feat_extra.count();

                sensor_msgs::PointCloud2 laserCloudFilteredMsg;
                pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
                *cloud_filtered += *cloud_edge;
                *cloud_filtered += *cloud_surf;
                pcl::toROSMsg(*cloud_filtered, laserCloudFilteredMsg);
                laserCloudFilteredMsg.header.stamp = cloud_in_time;
                laserCloudFilteredMsg.header.frame_id = "/base_link";
                pubLaserCloudFiltered.publish(laserCloudFilteredMsg);

                sensor_msgs::PointCloud2 edgePointsMsg;
                pcl::toROSMsg(*cloud_edge, edgePointsMsg);
                edgePointsMsg.header.stamp = cloud_in_time;
                edgePointsMsg.header.frame_id = "/base_link";
                pubEdgePoints.publish(edgePointsMsg);

                sensor_msgs::PointCloud2 surfPointsMsg;
                pcl::toROSMsg(*cloud_surf, surfPointsMsg);
                surfPointsMsg.header.stamp = cloud_in_time;
                surfPointsMsg.header.frame_id = "/base_link";
                pubSurfPoints.publish(surfPointsMsg);

            }
            //sleep 2 ms every time
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    LaserProcessing() {

        initROSParam();

        initROSHandler();

    }

private:

    ros::NodeHandle nh;

    std::mutex mtx;
    std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudBuf;

    ros::Subscriber subLaserCloud;
    ros::Publisher pubEdgePoints;
    ros::Publisher pubSurfPoints;
    ros::Publisher pubLaserCloudFiltered;

    int scan_line;
    double vertical_angle, scan_period, max_dis, min_dis;

};

int main(int argc, char **argv) {

    ros::init(argc, argv, "laserProcessing");

    LaserProcessing laserProcessing;

    std::thread laser_processing_thread{&LaserProcessing::laser_processing, &laserProcessing};

    ros::spin();

}