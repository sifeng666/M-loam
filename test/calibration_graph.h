#pragma once

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>


namespace Eigen
{
    typedef Matrix<double, 6, 6> Matrix6d;
    typedef Matrix<double, 6, 1> Vector6d;
}

namespace ct
{
    class CalibrationGraph
    {
    public:

        typedef std::shared_ptr<CalibrationGraph> Ptr;
        typedef std::shared_ptr<const CalibrationGraph> ConstPtr;
        typedef Eigen::Matrix<double, 6, 6> Matrix6d;

        struct Edge
        {
            Edge(gtsam::Key from_, gtsam::Key to_, const Eigen::Matrix4d& measurement_, const Eigen::Matrix6d& information_)
                    : from(from_), to(to_), measurement(measurement_), information(information_) {}
            gtsam::Key from, to;
            Eigen::Matrix4d measurement;
            Eigen::Matrix6d information;
        };

        CalibrationGraph();

        CalibrationGraph::Ptr clone() const;

        double error() const;

        void clear();

        void set_initial(gtsam::Key id, const Eigen::Matrix4d& initial_estimate);

        void add_edge(gtsam::Key from, gtsam::Key to, const Eigen::Matrix4d& measurement,
                      const Matrix6d& information);

        const std::vector<Edge>& get_edges() const { return edges_; }

        /** \brief get pose for given id. **/
        Eigen::Matrix4d pose(gtsam::Key id) const;

        Eigen::Matrix4d initial_pose(gtsam::Key id) const;

        /** \brief get (optimized) poses sorted by id of the vertexes. **/
        std::unordered_map<gtsam::Key, Eigen::Matrix4d> poses() const;

        /** \brief start optimization for given number of iterations. **/
        bool optimize(int num_iters);

        /** \brief set m estimator. **/
        void setMEstimator(const gtsam::noiseModel::mEstimator::Base::shared_ptr& m_estimator);

        void save(const std::string& filename) const;
        void load(const std::string& filename);


        gtsam::NonlinearFactorGraph& graph();
        gtsam::Values& initial();
        gtsam::Values& result();
        const gtsam::NonlinearFactorGraph& graph() const;
        const gtsam::Values& initial() const;
        const gtsam::Values& result() const;

    protected:
        std::vector<Edge> edges_;
        gtsam::Values initial_;
        gtsam::Values result_;
        gtsam::NonlinearFactorGraph graph_;

        bool robustify_{false};
        gtsam::noiseModel::mEstimator::Base::shared_ptr robustifier_;
    };
}