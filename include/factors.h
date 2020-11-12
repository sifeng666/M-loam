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

//class LidarPose3EdgeFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
//{
//private:
//    gtsam::Vector3 curr_point_, last_point_a_, last_point_b_;
//    double s_;
//
//public:
//
//    LidarPose3EdgeFactor( gtsam::Key pose_key, gtsam::Vector3 curr_point, gtsam::Vector3 last_point_a,
//                          gtsam::Vector3 last_point_b, gtsam::SharedNoiseModel noise_model)
//            : gtsam::NoiseModelFactor1<gtsam::Pose3>(noise_model, pose_key)
//            , curr_point_(curr_point), last_point_a_(last_point_a)
//            , last_point_b_(last_point_b) {}
//
//    gtsam::Vector evaluateError( const Pose3& pose, boost::optional<Matrix&> H1 = boost::none) const {
//        gtsam::Point3 cp(curr_point_(0), curr_point_(1), curr_point_(2));
//        gtsam::Point3 lpa(last_point_a_(0), last_point_a_(1), last_point_a_(2));
//        gtsam::Point3 lpb(last_point_b_(0), last_point_b_(1), last_point_b_(2));
//
//// 	gtsam::Pose3 T_(pose);
//// 	Eigen::Quaterniond q = T_.rotation().toQuaternion().normalized();
//// 	Eigen::Vector3d t = T_.translation().vector();
////
//// 	Eigen::Quaterniond q_identity(1.0, 0.0, 0.0, 0.0);
//// 	Eigen::Quaterniond q_last_curr = q_identity.slerp(s_, q);
//// 	Eigen::Matrix<double, 3, 1> t_last_curr(s_ * t(0), s_ * t(1), s_ * t(2));
////
//// 	gtsam::Pose3 T_last_curr(gtsam::Rot3(q_last_curr), gtsam::Point3(t_last_curr));
//
//        gtsam::Matrix36 Dpose;
//        gtsam::Point3 lp = pose.transform_from(cp, H1 ? &Dpose : 0);
//
//        gtsam::Point3 nu = (lp - lpa).cross(lp - lpb);
//        gtsam::Point3 de = lpa - lpb;
//
//        gtsam::Vector3 residual;
//        residual(0) = nu.x() / de.norm();
//        residual(1) = nu.y() / de.norm();
//        residual(2) = nu.z() / de.norm();
//
//        // if we need jaccobians
//        if(H1) {
//            H1->resize(3, 6);
//
//            *H1 << -(gtsam::skewSymmetric(de.x(), de.y(), de.z()) * Dpose) / de.norm() ;
//        }
//
//        return residual;
//    }
//};
//
//class LidarPose3PlaneNormFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
//{
//private:
//    gtsam::Vector3 curr_point_;
//    gtsam::Vector3 plane_unit_norm_;
//    double negative_OA_dot_norm_;
//
//public:
//    LidarPose3PlaneNormFactor(gtsam::Key pose_key, gtsam::Vector3 curr_point,
//                              gtsam::Vector3 plane_unit_norm, double negative_OA_dot_norm,
//                              gtsam::SharedNoiseModel noise_model)
//            : gtsam::NoiseModelFactor1<gtsam::Pose3>(noise_model, pose_key),
//              curr_point_(curr_point), plane_unit_norm_(plane_unit_norm),
//              negative_OA_dot_norm_(negative_OA_dot_norm) {}
//
//    gtsam::Vector evaluateError( const Pose3& pose, boost::optional<Matrix&> H1 = boost::none) const
//    {
//        gtsam::Point3 cp(curr_point_(0), curr_point_(1), curr_point_(2));
//        gtsam::Point3 norm(plane_unit_norm_(0), plane_unit_norm_(1), plane_unit_norm_(2));
//
//        gtsam::Matrix36 Dpose;
//        gtsam::Point3 point_w = pose.transform_from(cp, H1 ? &Dpose : 0);
//
//        gtsam::Vector1 residual;
//        gtsam::Matrix13 Dnorm, Dpoint;
//        residual(0) = norm.dot(point_w, Dnorm, Dpoint) + negative_OA_dot_norm_;
//
//        // if we need jaccobians
//        if(H1) {
//            H1->resize(1, 6);
//
//            *H1 << Dpoint * Dpose;
//        }
//
//        return residual;
//    }
//
//};
//
//class LidarPose3PlaneFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
//{
//private:
//    gtsam::Vector3 curr_point_;
//    gtsam::Vector3 last_point_j_, last_point_l_, last_point_m_;
//    gtsam::Vector3 ljm_norm_;
//
//    double s_;
//
//public:
//    LidarPose3PlaneFactor( gtsam::Key pose_key, gtsam::Vector3 curr_point,
//                           gtsam::Vector3 last_point_j, gtsam::Vector3 last_point_l,
//                           gtsam::Vector3 last_point_m, const double& s,
//                           gtsam::SharedNoiseModel noise_model)
//            : gtsam::NoiseModelFactor1<gtsam::Pose3>(noise_model, pose_key),
//              curr_point_(curr_point), last_point_j_(last_point_j),
//              last_point_l_(last_point_l), last_point_m_(last_point_m),
//              s_(s)
//    {
//        ljm_norm_ = (last_point_j - last_point_l).cross(last_point_j - last_point_m);
//        ljm_norm_.normalize();
//    }
//
//    gtsam::Vector evaluateError( const Pose3& pose, boost::optional<Matrix&> H1 = boost::none) const
//    {
//        gtsam::Point3 cp(curr_point_(0), curr_point_(1), curr_point_(2));
//        gtsam::Point3 lpj(last_point_j_(0), last_point_j_(1), last_point_j_(2));
//        gtsam::Point3 ljm(ljm_norm_(0), ljm_norm_(1), ljm_norm_(2));
//
//// 	gtsam::Pose3 T_(pose);
//// 	Eigen::Quaterniond q = T_.rotation().toQuaternion().normalized();
//// 	Eigen::Vector3d t = T_.translation().vector();
////
//// 	Eigen::Quaterniond q_identity(1.0, 0.0, 0.0, 0.0);
//// 	Eigen::Quaterniond q_last_curr = q_identity.slerp(s_, q);
//// 	Eigen::Matrix<double, 3, 1> t_last_curr(s_ * t(0), s_ * t(1), s_ * t(2));
////
//// 	gtsam::Pose3 T_last_curr(gtsam::Rot3(q_last_curr), gtsam::Point3(t_last_curr));
//
//        gtsam::Matrix36 Dpose;
//        gtsam::Point3 lp = pose.transform_from(cp, H1 ? &Dpose : 0);
//
//        gtsam::Vector1 residual;
//        gtsam::Point3 lpij = lp - lpj;
//        gtsam::Matrix13 Dnorm, Dpoint;
//        residual(0) = lpij.dot(ljm, Dpoint, Dnorm);
//
//        // if we need jaccobians
//        if(H1) {
//            H1->resize(1, 6);
//
//            *H1 << Dpoint * Dpose;
//        }
//
//        return residual;
//    }
//
//};


}