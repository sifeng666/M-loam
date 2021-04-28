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
    Eigen::Matrix4d T;
    T << 0.15899,    0.986373,  -0.0423144,    -1.22511,
         -0.987278,    0.158747, -0.00906268,     7.94935,
         -0.00222191,   0.0432169,    0.999063,           0,
    0,           0,           0,           1;



    Eigen::Matrix4d TT;
    TT << 0.995941579342, 0.089642129838, 0.008032932878 ,-0.598650217056,
                                                   -0.089658133686, 0.995970666409, 0.001663845731, 9.558296203613,
                                                                                                 -0.007851422764, -0.002377322642, 0.999966681004, -0.461331695318,
    0.000000000000, 0.000000000000, 0.000000000000, 1.000000000000;

   cout << TT * T << endl;


}