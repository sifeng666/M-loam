#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/OrientedPlane3.h>

namespace gtsam
{

/**
 * Factor to implement error between a point landmark and a 3D line constructed by two points.
 */
class PointToEdgeFactor: public NoiseModelFactor1<Pose3>
{
protected:

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
        const SharedNoiseModel& pose_noise_model) : Base(pose_noise_model, pose_key), p_s_(p_s), p_a_(p_a), p_b_(p_b)
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
            Base(pose_noise_model, pose_key), p_s_(p_s), plane_unit_norm_(plane_unit_norm),
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

class PlaneToPlaneFactor: public NoiseModelFactor2<Pose3, Pose3>
{
protected:

    OrientedPlane3 this_plane;
    OrientedPlane3 closest_plane;
    typedef NoiseModelFactor2<Pose3, Pose3> Base;
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// Constructor
    PlaneToPlaneFactor() {
    }
    virtual ~PlaneToPlaneFactor() {}


    PlaneToPlaneFactor(
        const Key& this_key, const OrientedPlane3& this_plane_,
        const Key& closest_key, const OrientedPlane3& closest_plane_,
        const SharedNoiseModel& pose_noise_model) :
        Base(pose_noise_model, this_key, closest_key), this_plane(this_plane_), closest_plane(closest_plane_)
    {}

    Vector evaluateError(const Pose3 &pose1, const Pose3 &pose2,
                         boost::optional<Matrix&> H1 = boost::none,
                         boost::optional<Matrix&> H2 = boost::none) const override
    {
        Matrix36 H_r_pose1, H_r_pose2;

        OrientedPlane3 p1 = this_plane.transform(pose1, boost::none, H1 ? &H_r_pose1 : nullptr);

        OrientedPlane3 p2 = closest_plane.transform(pose2, boost::none, H2 ? &H_r_pose2 : nullptr);

        Vector3 err = p1.errorVector(p2);

        if (H1) *H1 = H_r_pose1;

        if (H2) *H2 = H_r_pose2;

        return err;
    }
};

//class PointToPlaneFactor2: public NoiseModelFactor2<Pose3, Pose3>
//{
//protected:
//    Point3 curr_point;
//    OrientedPlane3 plane;
//    typedef NoiseModelFactor2<Pose3, Pose3> Base;
//public:
//
//    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//
//    /// Constructor
//    PointToPlaneFactor2() {
//    }
//    virtual ~PointToPlaneFactor2() {}
//
//
//    PointToPlaneFactor2(
//            const Key& point_key, const Point3& curr_point_,
//            const Key& plane_key, const OrientedPlane3& plane_,
//            const SharedNoiseModel& pose_noise_model) :
//            Base(pose_noise_model, point_key, plane_key), curr_point(curr_point_), plane(plane_)
//    {}
//
//    Vector evaluateError(const Pose3 &pose1, const Pose3 &pose2,
//                         boost::optional<Matrix&> H1 = boost::none,
//                         boost::optional<Matrix&> H2 = boost::none) const override
//    {
//        Matrix36 H_r_pose1, H_r_pose2;
//
//        OrientedPlane3 p1 = this_plane.transform(pose1, boost::none, H1 ? &H_r_pose1 : nullptr);
//
//        OrientedPlane3 p2 = closest_plane.transform(pose2, boost::none, H2 ? &H_r_pose2 : nullptr);
//
//        plane.
//
//        Vector3 err = p1.errorVector(p2);
//
//        if (H1) *H1 = H_r_pose1;
//
//        if (H2) *H2 = H_r_pose2;
//
//        return err;
//    }
//};


}