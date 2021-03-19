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

//    Eigen::Vector3d t(0.21, 0.01, -0.1);
//    Eigen::Matrix3d R = eularAngleToMatrix(-2.3*M_PI/180, 0*M_PI/180, 0*M_PI/180);
//    cout << fromRotationAndTranslation(R, t) << endl;


    Eigen::Matrix4d TT;
    TT << -0.999997735023, 0.001494844211, 0.00154800422, -0.845576286316,
        0.001506577595, -0.999970078468, -0.007606379222, 0.009910655208,
        0.001536586904, -0.007608694024 ,0.999969959259, 0.036424160004,
        0.0, 0.0, 0.0, 1.0;

    Eigen::Isometry3d T(TT);
    cout << T.matrix() << endl;
    cout << T.inverse().matrix() << endl;

}