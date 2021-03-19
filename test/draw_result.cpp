//
// Created by ziv on 2021/3/18.
//

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <string>
#include <vector>
#include <gtsam/geometry/Pose3.h>

#include "matplotlibcpp/matplotlibcpp.h"

namespace plt = matplotlibcpp;
using namespace std;
using namespace gtsam;

void print_pose(Pose3 p) {
    auto q = p.rotation().toQuaternion();
    cout << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << ", " << p.x() << ", " << p.y() << ", " << p.z() << endl;
}

void draw_interpolate() {

    Pose3 p1 = Pose3::identity();

    Pose3 p2(Rot3::RzRyRx(0*M_PI/180, 90*M_PI/180, 0*M_PI/180), Point3(8, 3, 0));

    Pose3 p3(Rot3::RzRyRx(0*M_PI/180, 0*M_PI/180, 90*M_PI/180), Point3(10, 10, 0));

    for (int i = 0; i < 10; i++) {
        auto p_ = p1.interpolateRt(p2, i / (10.0));
        print_pose(p_);
    }
    for (int i = 0; i < 11; i++) {
        auto p_ = p2.interpolateRt(p3, i / (10.0));
        print_pose(p_);
    }
    cout << "tangent space:" << endl;
    for (int i = 0; i < 10; i++) {
        print_pose(gtsam::interpolate(p1, p2, i / (10.0)));
    }
    for (int i = 0; i < 11; i++) {
        print_pose(gtsam::interpolate(p2, p3, i / (10.0)));
    }
}

int main() {
    draw_interpolate();
}
