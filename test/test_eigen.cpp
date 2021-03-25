//
// Created by ziv on 2021/3/9.
//

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <string>
#include <vector>
#include <gtsam/geometry/Pose3.h>

using namespace std;
using namespace Eigen;
using namespace gtsam;


Eigen::Matrix3d eularAngleToMatrix(double yaw, double pitch, double roll) {
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());

    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;

    return q.matrix();
}


Eigen::Matrix4d fromRotationAndTranslation(const Eigen::Matrix3d& R, const Eigen::Vector3d& t) {
    Eigen::Matrix4d T;
    T.setIdentity();
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = t;
    return T;
}

int main() {

    Eigen::Vector3d t(0, -0.477, -0.220);
    Eigen::Matrix3d R = eularAngleToMatrix(0*M_PI/180, 0*M_PI/180, 40*M_PI/180);
    cout << fromRotationAndTranslation(R, t) << endl;
//
//    Eigen::Matrix4d T;
//    T <<   -0.999998,  0.00150658,  0.00153659,   -0.845645,
//    0.00149484,    -0.99997, -0.00760869,   0.0114515,
//    0.001548, -0.00760638,     0.99997,  -0.0350387,
//    0,           0,           0,           1;
//
//
//
//    Eigen::Matrix4d TT;
//    TT << 1.000,	-0.003,	-0.002,	0.007,
//    0.003,	1.000,	0.001,	-0.020,
//    0.002,	-0.001,	1.000,	-0.001,
//    0.000,	0.000,	0.000,	1.000;
//
//   cout << TT * T << endl;







}