//
// Created by ziv on 2020/10/16.
//

#include "helper.h"
#include <ceres/ceres.h>
#include <ceres/rotation.h>


struct LidarDistanceFactor
{

    LidarDistanceFactor(const Eigen::Vector3d& curr_point_, const Eigen::Vector3d& closed_point_)
            : curr_point(curr_point_), closed_point(closed_point_){}

    template <typename T>
    bool operator()(const T *q, const T *t, T *residual) const
    {
        Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
        Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};
        Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
        Eigen::Matrix<T, 3, 1> point_w;
        point_w = q_w_curr * cp + t_w_curr;


        residual[0] = point_w.x() - T(closed_point.x());
        residual[1] = point_w.y() - T(closed_point.y());
        residual[2] = point_w.z() - T(closed_point.z());
        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Vector3d& curr_point_, const Eigen::Vector3d& closed_point_)
    {
        return (new ceres::AutoDiffCostFunction<
                LidarDistanceFactor, 3, 4, 3>(
                new LidarDistanceFactor(curr_point_, closed_point_)));
    }

    Eigen::Vector3d curr_point;
    Eigen::Vector3d closed_point;
};


