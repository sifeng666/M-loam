//
// Created by ziv on 2020/10/26.
//

#include "helper.h"

#include <pcl/filters/radius_outlier_removal.h>

class LaserProcessor {
public:
    using Ptr = std::shared_ptr<LaserProcessor>;

    int i;
    std::mutex mtx;
    std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudBuf;

    ros::Subscriber subLaserCloud;
    ros::Publisher  pubEdgePoints;
    ros::Publisher  pubSurfPoints;
    ros::Publisher  pubLessEdgePoints;
    ros::Publisher  pubLessSurfPoints;
    void lock()   { mtx.lock(); }
    void unlock() { mtx.unlock(); }

    void pointCloudHandler(const sensor_msgs::PointCloud2ConstPtr &pointCloudMsg) {
        std::lock_guard lg(mtx);
        pointCloudBuf.push(pointCloudMsg);
    }

    LaserProcessor(int i_) : i(i_) {}
};

class LaserProcessing {

public:
    using IndexVal = std::pair<int, double>; // first: index, second: value
public:

    void featureExtractionFromSector(const pcl::PointCloud<PointT>::Ptr& cloud_in,
                                     std::vector<IndexVal>& cloudCurvature,
                                     pcl::PointCloud<PointT>::Ptr& cloud_out_edge,
                                     pcl::PointCloud<PointT>::Ptr& cloud_out_surf,
                                     pcl::PointCloud<PointT>::Ptr& cloud_out_less_edge,
                                     pcl::PointCloud<PointT>::Ptr& cloud_out_less_surf)
    {
        std::sort(cloudCurvature.begin(), cloudCurvature.end(), [](const IndexVal& a, const IndexVal& b) {
            return a.second < b.second;
        });


        int largestPickedNum = 0;
        std::vector<int> picked_points;
        for (int i = cloudCurvature.size()-1; i >= 0; i--) {
            int ind = cloudCurvature[i].first;
            if (std::find(picked_points.begin(), picked_points.end(), ind) == picked_points.end()) {
                if(cloudCurvature[i].second <= 0.1) {
                    break;
                }

                largestPickedNum++;
                picked_points.push_back(ind);

                if (largestPickedNum <= 2) {
                    cloud_out_edge->push_back(cloud_in->points[ind]);
                    cloud_out_less_edge->push_back(cloud_in->points[ind]);
                }
                else if (largestPickedNum <= 20) {
                    cloud_out_less_edge->push_back(cloud_in->points[ind]);
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
         int smallestPickedNum = 0;

         for (int i = 0; i <= (int)cloudCurvature.size()-1; i++) {
             int ind = cloudCurvature[i].first;

             if( std::find(picked_points.begin(), picked_points.end(), ind)==picked_points.end()){
                 if(cloudCurvature[i].second > 0.1){
                     break;
                 }
                 smallestPickedNum++;
                 picked_points.push_back(ind);

                 if(smallestPickedNum <= 4){
                     //find all points
                     cloud_out_surf->push_back(cloud_in->points[ind]);
                     cloud_out_less_surf->push_back(cloud_in->points[ind]);
                 }
                 else{
                     break;
                 }

                 for(int k=1;k<=5;k++){
                     double diffX = cloud_in->points[ind + k].x - cloud_in->points[ind + k - 1].x;
                     double diffY = cloud_in->points[ind + k].y - cloud_in->points[ind + k - 1].y;
                     double diffZ = cloud_in->points[ind + k].z - cloud_in->points[ind + k - 1].z;
                     if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05){
                         break;
                     }
                     picked_points.push_back(ind+k);
                 }
                 for(int k=-1;k>=-5;k--){
                     double diffX = cloud_in->points[ind + k].x - cloud_in->points[ind + k + 1].x;
                     double diffY = cloud_in->points[ind + k].y - cloud_in->points[ind + k + 1].y;
                     double diffZ = cloud_in->points[ind + k].z - cloud_in->points[ind + k + 1].z;
                     if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05){
                         break;
                     }
                     picked_points.push_back(ind+k);
                 }

             }
         }

        for (int i = 0; i <= (int)cloudCurvature.size()-1; i++) {
            int ind = cloudCurvature[i].first;
            if(std::find(picked_points.begin(), picked_points.end(), ind) == picked_points.end()) {
                cloud_out_less_surf->push_back(cloud_in->points[ind]);
            }
        }
    }

    void featureExtraction(const pcl::PointCloud<PointT>::Ptr& cloud_in,
                           pcl::PointCloud<PointT>::Ptr& cloud_out_edge,
                           pcl::PointCloud<PointT>::Ptr& cloud_out_surf,
                           pcl::PointCloud<PointT>::Ptr& cloud_out_less_edge,
                           pcl::PointCloud<PointT>::Ptr& cloud_out_less_surf)
    {
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

                featureExtractionFromSector(laserCloudScans[i], subCloudCurvature, cloud_out_edge, cloud_out_surf, cloud_out_less_edge, cloud_out_less_surf);

            }

        }
    }

    void process(LaserProcessor::Ptr lidar) {

        if (!lidar->pointCloudBuf.empty()) {

            pcl::PointCloud<PointT>::Ptr cloud_in(new pcl::PointCloud<PointT>());
            pcl::PointCloud<PointT>::Ptr cloud_edge(new pcl::PointCloud<PointT>());
            pcl::PointCloud<PointT>::Ptr cloud_surf(new pcl::PointCloud<PointT>());
            pcl::PointCloud<PointT>::Ptr cloud_less_edge(new pcl::PointCloud<PointT>());
            pcl::PointCloud<PointT>::Ptr cloud_less_surf(new pcl::PointCloud<PointT>());

            lidar->lock();
            pcl::fromROSMsg(*lidar->pointCloudBuf.front(), *cloud_in);
            ros::Time cloud_in_time = (lidar->pointCloudBuf.front())->header.stamp;
            lidar->pointCloudBuf.pop();
            lidar->unlock();

            featureExtraction(cloud_in, cloud_edge, cloud_surf, cloud_less_edge, cloud_less_surf);

            std::string frame_id = "frame" + std::to_string(lidar->i);

            sensor_msgs::PointCloud2 edgePointsMsg;
            pcl::toROSMsg(*cloud_edge, edgePointsMsg);
            edgePointsMsg.header.stamp = cloud_in_time;
            edgePointsMsg.header.frame_id = frame_id;
            lidar->pubEdgePoints.publish(edgePointsMsg);

            sensor_msgs::PointCloud2 surfPointsMsg;
            pcl::toROSMsg(*cloud_surf, surfPointsMsg);
            surfPointsMsg.header.stamp = cloud_in_time;
            surfPointsMsg.header.frame_id = frame_id;
            lidar->pubSurfPoints.publish(surfPointsMsg);

            sensor_msgs::PointCloud2 edgeLessPointsMsg;
            pcl::toROSMsg(*cloud_less_edge, edgeLessPointsMsg);
            edgeLessPointsMsg.header.stamp = cloud_in_time;
            edgeLessPointsMsg.header.frame_id = frame_id;
            lidar->pubLessEdgePoints.publish(edgeLessPointsMsg);

            sensor_msgs::PointCloud2 surfLessPointsMsg;
            pcl::toROSMsg(*cloud_less_surf, surfLessPointsMsg);
            surfLessPointsMsg.header.stamp = cloud_in_time;
            surfLessPointsMsg.header.frame_id = frame_id;
            lidar->pubLessSurfPoints.publish(surfLessPointsMsg);

        }
    }

    void laser_processing() {

        while (ros::ok()) {

            process(lidar0);

            process(lidar1);
            //sleep 2 ms every time
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    LaserProcessing() {

        lidar0 = std::make_shared<LaserProcessor>(0);
        lidar1 = std::make_shared<LaserProcessor>(1);

        scan_period     = nh.param<double>("scan_period", 0.1);
        vertical_angle  = nh.param<double>("vertical_angle", 2.0);
        max_dis         = nh.param<double>("max_dis", 90.0);
        min_dis         = nh.param<double>("min_dis", 2.0);
        scan_line       = nh.param<int>("scan_line", 16);

        // lidar 0
        lidar0->subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/left/velodyne_points", 100, &LaserProcessor::pointCloudHandler, &(*lidar0));
        lidar0->pubEdgePoints = nh.advertise<sensor_msgs::PointCloud2>("/left/laser_cloud_edge", 100);
        lidar0->pubSurfPoints = nh.advertise<sensor_msgs::PointCloud2>("/left/laser_cloud_surf", 100);
        lidar0->pubLessEdgePoints = nh.advertise<sensor_msgs::PointCloud2>("/left/laser_cloud_less_edge", 100);
        lidar0->pubLessSurfPoints = nh.advertise<sensor_msgs::PointCloud2>("/left/laser_cloud_less_surf", 100);

        // lidar 1
        lidar1->subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/right/velodyne_points", 100, &LaserProcessor::pointCloudHandler, &(*lidar1));
        lidar1->pubEdgePoints = nh.advertise<sensor_msgs::PointCloud2>("/right/laser_cloud_edge", 100);
        lidar1->pubSurfPoints = nh.advertise<sensor_msgs::PointCloud2>("/right/laser_cloud_surf", 100);
        lidar1->pubLessEdgePoints = nh.advertise<sensor_msgs::PointCloud2>("/right/laser_cloud_less_edge", 100);
        lidar1->pubLessSurfPoints = nh.advertise<sensor_msgs::PointCloud2>("/right/laser_cloud_less_surf", 100);

    }

private:

    ros::NodeHandle nh;

    LaserProcessor::Ptr lidar0;
    LaserProcessor::Ptr lidar1;

    int scan_line;
    double vertical_angle, scan_period, max_dis, min_dis;

};

int main(int argc, char **argv) {

    ros::init(argc, argv, "laserProcessing");

    LaserProcessing laserProcessing;

    std::thread laser_processing_thread{&LaserProcessing::laser_processing, &laserProcessing};

    ros::spin();

}