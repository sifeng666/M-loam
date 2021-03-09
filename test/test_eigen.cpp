//
// Created by ziv on 2021/3/9.
//

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <string>
#include <vector>

using namespace std;
using namespace Eigen;


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

    Eigen::Vector3d t(-0.825, -0.02, -0.1);
    Eigen::Matrix3d R = eularAngleToMatrix(-179.5*M_PI/180, 0*M_PI/180, 0*M_PI/180);
    cout << fromRotationAndTranslation(R, t) << endl;

}