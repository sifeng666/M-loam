//
// Created by ziv on 2021/1/11.
//

#include "calibration.h"

static int file_count = 1;

void write_to_file(std::string filename, const std::vector<PoseTimeStamp>& PTS) {
    boost::filesystem::path save_path(filename);
    auto folder_path = save_path.parent_path();
    if (!boost::filesystem::exists(folder_path)) {
        boost::filesystem::create_directories(folder_path);
    }
    std::ofstream f(filename);
    for (size_t i = 0; i < PTS.size(); i++) {
        char ret[200];
        Eigen::Quaterniond q(PTS[i].pose.rotation().matrix());
        sprintf(ret, "%f %f %f %f %f %f %f\n", q.x(), q.y(), q.z(), q.w(), PTS[i].pose.translation().x(), PTS[i].pose.translation().y(), PTS[i].pose.translation().z());
        f << ret;
    }
}

gtsam::Point3 linear_interpolation(double t, gtsam::Point3 start, gtsam::Point3 end) {
    end[0] = start[0] * (1 - t) + end[0] * t;
    end[1] = start[1] * (1 - t) + end[1] * t;
    end[2] = start[2] * (1 - t) + end[2] * t;
    return end;
}

gtsam::Pose3 interpolate(double t, const gtsam::Pose3& start, const gtsam::Pose3& end) {
    return gtsam::Pose3(start.rotation().slerp(t, end.rotation()), linear_interpolation(t, start.translation(), end.translation()));
}

bool HandEyeCalibrator::calibrate(KeyframeVec::Ptr keyframeVec_0, KeyframeVec::Ptr keyframeVec_i, gtsam::Pose3& T_0_i) {

    bool success;
    std::vector<gtsam::Pose3> poses_0 = keyframeVec_0->read_poses(0, keyframeVec_0->keyframes.size(), true);
    std::vector<gtsam::Pose3> poses_i = keyframeVec_i->read_poses(0, keyframeVec_i->keyframes.size(), true);

    std::vector<PoseTimeStamp> PTS_0, PTS_i;
    for (size_t i = 0; i < poses_0.size(); i++) PTS_0.emplace_back(keyframeVec_0->keyframes[i]->cloud_in_time, poses_0[i]);
    for (size_t i = 0; i < poses_i.size(); i++) PTS_i.emplace_back(keyframeVec_i->keyframes[i]->cloud_in_time, poses_i[i]);

    success = sync_timestamp(PTS_0, PTS_i);
    if (!success) {
        return false;
    }

    camodocal::PlanarHandEyeCalibration calib;
    calib.setVerbose(false);

    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > H1, H2;
    bool is_first = true;
    gtsam::Pose3 last_pose_0, last_pose_i;
    for (size_t j = 0; j < PTS_0.size(); ++j) {
        if (is_first) {
            is_first = false;
            last_pose_0 = PTS_0[j].pose;
            last_pose_i = PTS_i[j].pose;
            continue;
        }
        H1.push_back(last_pose_0.between(PTS_0[j].pose).matrix());
        H2.push_back(last_pose_i.between(PTS_i[j].pose).matrix());
    }
    success = calib.addMotions(H1, H2);
    if (!success) {
        return false;
    }
    Eigen::Matrix4d H_12;
    success = calib.calibrate(H_12);
    if (!success) {
        return false;
    }
    T_0_i = gtsam::Pose3(H_12);
    return true;
}

bool HandEyeCalibrator::sync_timestamp(std::vector<PoseTimeStamp>& PTS_0, std::vector<PoseTimeStamp>& PTS_i) {

    if (PTS_0.size() < 30 && PTS_i.size() < 30) {
//        std::cout << "too less trajectories, exit" << std::endl;
        return false;
    }

//    write_to_file("/home/ziv/mloam/interpolate/" + std::to_string(file_count) + "/PTS0.txt", PTS_0);
//    write_to_file("/home/ziv/mloam/interpolate/" + std::to_string(file_count) + "/PTS1.txt", PTS_i);

    std::vector<PoseTimeStamp> PTS_0_opti, PTS_i_opti;

    const int SKIP = 5;

    size_t k = 0;
    for (size_t i = SKIP; i < PTS_0.size() - SKIP; i++) {
        PTS_0_opti.push_back(PTS_0[i]);
        ros::Time currTime = PTS_0[i].timestamp;
        while (k < PTS_i.size() && PTS_i[k].timestamp < currTime) k++;
        if (k == PTS_i.size()) {
//            std::cout << "range err" << std::endl;
            return false;
        }
        size_t last_k = k;
        while (last_k >= 0 && PTS_i[last_k].timestamp >= currTime) last_k--;
        if (last_k < 0) {
//            std::cout << "range err" << std::endl;
            return false;
        }

        double t = (currTime - PTS_i[last_k].timestamp).toSec() /
                (PTS_i[k].timestamp - PTS_i[last_k].timestamp).toSec();

        PTS_i_opti.emplace_back(currTime, gtsam::interpolate(PTS_i[last_k].pose, PTS_i[k].pose, t));
    }

//    write_to_file("/home/ziv/mloam/interpolate/" + std::to_string(file_count) + "/PTS0_opti.txt", PTS_0_opti);
//    write_to_file("/home/ziv/mloam/interpolate/" + std::to_string(file_count) + "/PTS1_opti.txt", PTS_i_opti);
//    write_to_file("/home/ziv/mloam/interpolate/" + std::to_string(file_count) + "/PTS1_gtsam.txt", PTS_i_gtsam);
//    write_to_file("/home/ziv/mloam/interpolate/" + std::to_string(file_count) + "/PTS1_RT.txt", PTS_i_RT);

    PTS_0 = PTS_0_opti;
    PTS_i = PTS_i_opti;
    return true;
}