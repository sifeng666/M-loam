#include <iostream>
#include <random>
#include <cstdlib>
#include <fstream>
#include <gtsam/geometry/Pose3.h>
#include "camodocal/EigenUtils.h"
#include "camodocal/PlanarHandEyeCalibration.h"
#include "camodocal/HandEyeCalibration.h"

using namespace std;

template<class T>
const T random(const T& a, const T& b)
{
    return static_cast<double>(rand()) / RAND_MAX * (b - a) + a;
}

double d2r(double deg)
{
    return deg / 180.0 * M_PI;
}

int main()
{

    Eigen::Matrix4d T;
    T << 0.999754, -0.00692039,  -0.0210955,   0.0153439,
         -0.00685725,     0.80747,   -0.589869,   -0.387074,
        0.0211161,    0.589868,    0.807224,   -0.229381,
        0,           0,           0,           1;
    cout << T << endl;

//    Eigen::Quaterniond q(0.94677, 0.32182, -0.00094, 0.00744);
//    Eigen::Isometry3d T;
//    T.matrix().block<3, 3>(0, 0) = q.toRotationMatrix();
//    T.translation() = Eigen::Vector3d(0.06641, -0.49424, -0.11251);
//
//    cout << Eigen::Quaterniond(T.rotation()).coeffs() << endl;
//    cout << T.translation() << endl;
//
    cout << T.inverse() << endl;



    return 1;
    cout << "PlanarHandEyeCalibration: " << endl;
    camodocal::PlanarHandEyeCalibration calib;

    calib.setVerbose(true);

    Eigen::Matrix4d H_12_expected = Eigen::Matrix4d::Identity();

    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > H1, H2, H1_pose, H2_pose;

    double r, p, y, d, e, f;
    fstream f0("/home/ziv/mloam/interpolate/228/PTS0_opti.txt");
    bool is_first = true;
    gtsam::Pose3 last_pose;
    while (!f0.eof()) {
        f0 >> r >> p >> y >> d >> e >> f;
        gtsam::Pose3 pose(gtsam::Rot3::Ypr(y, p, r), gtsam::Point3(d, e, f));
        H1_pose.push_back(pose.matrix());
        if (is_first) {
            last_pose = pose;
            is_first = false;
            continue;
        }
        H1.push_back(last_pose.between(pose).matrix());
        last_pose = pose;
    }
    f0.close();

    fstream f1("/home/ziv/mloam/interpolate/228/PTS1_opti.txt");
    is_first = true;
    while (!f1.eof()) {
        f1 >> r >> p >> y >> d >> e >> f;
        gtsam::Pose3 pose(gtsam::Rot3::Ypr(y, p, r), gtsam::Point3(d, e, f));
        H2_pose.push_back(pose.matrix());
        if (is_first) {
            last_pose = pose;
            is_first = false;
            continue;
        }
        H2.push_back(last_pose.between(pose).matrix());
        last_pose = pose;
    }
    f1.close();


    std::cout << "add motions: " << calib.addMotions(H2, H1) << std::endl;

    Eigen::Matrix4d H_21;
    std::cout << "calibrate: " << calib.calibrate(H_21) << std::endl;

    Eigen::Matrix3d R1 = H_21.block<3,3>(0,0);
    std::cout << Eigen::Quaterniond(R1).coeffs() << endl;
    Eigen::Vector3d t1 = H_21.block<3,1>(0,3);
    std::cout << t1[0] << " " << t1[1] << " " << t1[2] << endl;

    cout << "HandEyeCalibration: full" << endl;

    camodocal::HandEyeCalibration handEyeCalibration;
    handEyeCalibration.setVerbose(true);

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > rvecs1, tvecs1, rvecs2, tvecs2;
    Eigen::Vector3d rvec1, tvec1, rvec2, tvec2;
    for (size_t i = 0; i < H1_pose.size(); i++) {
        Eigen::AngleAxisd angleAxis1(H1_pose[i].block<3,3>(0,0));
        rvec1 = angleAxis1.angle() * angleAxis1.axis();
        Eigen::AngleAxisd angleAxis2(H2_pose[i].block<3,3>(0,0));
        rvec2 = angleAxis2.angle() * angleAxis2.axis();

        rvecs1.push_back(rvec1);
        tvecs1.push_back(H1_pose[i].block<3,1>(0,3));
        rvecs2.push_back(rvec2);
        tvecs2.push_back(H2_pose[i].block<3,1>(0,3));
    }
    Eigen::Matrix4d H_21_full;
    handEyeCalibration.estimateHandEyeScrew(rvecs1, tvecs1, rvecs2, tvecs2, H_21_full);

    R1 = H_21_full.block<3,3>(0,0);
    std::cout << Eigen::Quaterniond(R1).coeffs() << endl;
    t1 = H_21_full.block<3,1>(0,3);
    std::cout << t1[0] << " " << t1[1] << " " << t1[2] << endl;


    cout << "HandEyeCalibration: planar" << endl;

    handEyeCalibration.estimateHandEyeScrew(rvecs1, tvecs1, rvecs2, tvecs2, H_21_full, true);

    R1 = H_21_full.block<3,3>(0,0);
    std::cout << Eigen::Quaterniond(R1).coeffs() << endl;
    t1 = H_21_full.block<3,1>(0,3);
    std::cout << t1[0] << " " << t1[1] << " " << t1[2] << endl;

}

