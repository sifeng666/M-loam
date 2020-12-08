#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam
{

/**
 * Factor to implement error between a point landmark and a 3D line constructed by two points.
 */
class PointToEdgeFactor: public NoiseModelFactor1<Pose3>
{
protected:

    Key pose_key_;

    Point3 p_s_;
    Point3 p_a_;
    Point3 p_b_;

    double p_a_minus_p_b_norm;
    Matrix33 skew_p_b_minus_p_a_by_norm;

    typedef NoiseModelFactor1<Pose3> Base;

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// Constructor
    PointToEdgeFactor() {
    }
    virtual ~PointToEdgeFactor() {}

    PointToEdgeFactor(
        const Key& pose_key, const Point3& p_s, const Point3& p_a, const Point3& p_b,
        const SharedNoiseModel& pose_noise_model) : Base(pose_noise_model, pose_key),
        pose_key_(pose_key), p_s_(p_s), p_a_(p_a), p_b_(p_b)
    {
        Point3 re = p_a_ - p_b_;
        p_a_minus_p_b_norm = re.norm();
        skew_p_b_minus_p_a_by_norm = gtsam::skewSymmetric(re) / p_a_minus_p_b_norm;
    }

    Vector evaluateError(const Pose3 &pose, boost::optional<Matrix &> H) const override
    {
        Eigen::Matrix<double, 3, 6> d_p_d_pose;
        Point3 p_w = pose.transformFrom(p_s_, (H? &d_p_d_pose : nullptr));

        if (H) *H = -skew_p_b_minus_p_a_by_norm * d_p_d_pose;
        Vector3 err = (p_w - p_a_).cross(p_w - p_b_) / p_a_minus_p_b_norm;
        return err;
    }
};

/**
 * Factor to implement error between a point landmark and a 3D plane.
 */
class PointToPlaneFactor: public NoiseModelFactor1<Pose3>
{
protected:

    Key pose_key_;

    Point3 p_s_;

    Vector3 plane_unit_norm_;
    double  negative_OA_dot_norm_;

    typedef NoiseModelFactor1<Pose3> Base;

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// Constructor
    PointToPlaneFactor() {
    }
    virtual ~PointToPlaneFactor() {}


    PointToPlaneFactor(
            const Key& pose_key, const Point3& p_s,
            const Vector3& plane_unit_norm, const double& negative_OA_dot_norm,
            const SharedNoiseModel& pose_noise_model) :
            Base(pose_noise_model, pose_key),  pose_key_(pose_key), p_s_(p_s), plane_unit_norm_(plane_unit_norm),
            negative_OA_dot_norm_(negative_OA_dot_norm)
    {}

    Vector evaluateError(const Pose3 &pose, boost::optional<Matrix &> H) const override
    {
        Eigen::Matrix<double, 3, 6> d_p_d_pose;
        Point3 p_w = pose.transformFrom(p_s_, (H?&d_p_d_pose:nullptr));

        if (H) *H = plane_unit_norm_.transpose() * d_p_d_pose;

        Vector err(1);
        err << plane_unit_norm_.dot(p_w) + negative_OA_dot_norm_;
        return err;
    }
};


}