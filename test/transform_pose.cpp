//
// Created by ziv on 2021/3/23.
//

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <Eigen/Core>
#include <iomanip>

#include <gtsam/geometry/Pose3.h>

using namespace std;

string path = "/home/ziv/mloam/result/0309-circle02/compare/";
//vector<string> methods{"aloam", "balm", "hdl", "lego-loam", "mloam"};
vector<string> methods{"mloam"};
//vector<string> methods{"lego-loam"};


int main() {

    ifstream f;
    ofstream f_out;

    Eigen::Matrix3d R;
    R << 0, 1, 0, 0, 0, 1, 1, 0, 0;
    cout << R << endl;

    Eigen::Matrix4d T1(Eigen::Matrix4d::Identity());
    T1.block<3, 3>(0, 0) = R;
    cout << T1 << endl;
    Eigen::Isometry3d T(T1);
    T = T.inverse();

    // odom
    for (auto m : methods) {
        string filename = path + m + "/" + "odom_pose.txt";
        string filename_out = path + m + "/" + "odom_pose_out.txt";
        cout << filename_out << endl;
        f.open(filename);
        f_out.open(filename_out);
        Eigen::Isometry3d pose(Eigen::Matrix4d::Identity());

        //timestamp x y z q_x q_y q_z q_w
        //1607242378413432000
        while (!f.eof()) {
            string k;
            f >> k;
            double time_stamp;
            f >> time_stamp;

            // Read ceres result
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    f >> *(pose.data() + i+j*4);
                }
            }
            if (m == "lego-loam")
                pose = T * pose;

            Eigen::Quaterniond q(pose.rotation());
            f_out << setprecision(20) << time_stamp/1e9 << " " << pose.translation().x() << " "
                << pose.translation().y() << " " << pose.translation().z() << " "
                << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }
        f.close();
        cout << "hello";
        f_out.close();
    }
    // mapping
    for (auto m : methods) {
        string filename = path + m + "/" + "mapping_pose.txt";
        string filename_out = path + m + "/" + "mapping_pose_out.txt";
        f.open(filename);
        f_out.open(filename_out);
        Eigen::Isometry3d pose(Eigen::Matrix4d::Identity());

        //timestamp x y z q_x q_y q_z q_w
        //1607242378413432000
        while (!f.eof()) {
            string k;
            f >> k;
            double time_stamp;
            f >> time_stamp;
            // Read ceres result
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    f >> *(pose.data() + i+j*4);
                }
            }
            if (m == "lego-loam")
                pose = T * pose;

            Eigen::Quaterniond q(pose.rotation());
            f_out << setprecision(20) <<  time_stamp/1e9 << " " << pose.translation().x() << " "
                  << pose.translation().y() << " " << pose.translation().z() << " "
                  << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }
        f.close();
        f_out.close();
    }

}

