#include "calibration_graph.h"
#include "helper.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

namespace ct
{
    using namespace gtsam;

    CalibrationGraph::CalibrationGraph() {}

    void CalibrationGraph::clear()
    {
        graph_ = NonlinearFactorGraph();
        initial_.clear();
        edges_.clear();
        result_.clear();
    }

    CalibrationGraph::Ptr CalibrationGraph::clone() const
    {
        return CalibrationGraph::Ptr(new CalibrationGraph(*this));
    }

    void CalibrationGraph::set_initial(gtsam::Key id, const Eigen::Matrix4d& initial_estimate)
    {
        bool addPrior = initial_.empty();

        if (!initial_.exists(id))
            initial_.insert(id, Pose3(initial_estimate));
        else
            initial_.update(id, Pose3(initial_estimate));

        // also update intermediate result.
        if (result_.exists(id))
            result_.update(id, Pose3(initial_estimate));
        else
            result_.insert(id, Pose3(initial_estimate));

        if (addPrior)
        {
            Vector6 diagonal;
            diagonal << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6;
            noiseModel::Diagonal::shared_ptr priorModel = noiseModel::Diagonal::Variances(diagonal);
            graph_.emplace_shared<PriorFactor<Pose3>>(id, Pose3(), priorModel);
        }
    }


    void CalibrationGraph::add_edge(gtsam::Key from, gtsam::Key to, const Eigen::Matrix4d &measurement,
                                    const ct::CalibrationGraph::Matrix6d &information, bool use_info)
    {
        SharedNoiseModel model;
        if (use_info)
            model = noiseModel::Gaussian::Information(information);
        else {
            Vector6 diagonal;
            diagonal << 1e-3, 1e-3, 1e-3, 1e-2, 1e-2, 1e-1;
            model = noiseModel::Diagonal::Variances(diagonal);
        }

        if (robustify_)
        {
            model = noiseModel::Robust::Create(robustifier_, model);
        }

        graph_.emplace_shared<BetweenFactor<Pose3>>(from, to, Pose3(measurement), model);
        edges_.emplace_back(from, to, measurement, information);
    }

    Eigen::Matrix4d CalibrationGraph::pose(gtsam::Key id) const
    {
        return result_.at<Pose3>(id).matrix();
    }

    Eigen::Matrix4d CalibrationGraph::initial_pose(gtsam::Key id) const
    {
        return initial_.at<Pose3>(id).matrix();
    }


    std::unordered_map<gtsam::Key, Eigen::Matrix4d> CalibrationGraph::poses() const
    {
        std::unordered_map<gtsam::Key, Eigen::Matrix4d> poses_map;

        for(auto&&[key, value]: result_)
        {
            poses_map[key] = value.cast<Pose3>().matrix();
        }

        return poses_map;
    }

    void CalibrationGraph::save(const std::string& filename) const { /** not implement yet */ }
    void CalibrationGraph::load(const std::string& filename) { /** not implement yet */ }

    void CalibrationGraph::setMEstimator(
            const gtsam::noiseModel::mEstimator::Base::shared_ptr& m_estimator)
    {
        robustify_ = (m_estimator != nullptr);
        robustifier_ = m_estimator;
    }

    bool CalibrationGraph::optimize(int num_iters)
    {
        if (initial_.empty()) return false;

        std::cout << "Calibration Graph Optimization start!" << std::endl;

        TicToc tic_toc; tic_toc.tic();

        LevenbergMarquardtParams params;
        params.maxIterations = num_iters;
        LevenbergMarquardtOptimizer optimizer(graph_, initial_, params);

        result_ = optimizer.optimize();

        std::cout << "Calibration Graph Optimization done! cost " << tic_toc.toc() <<"ms.\n";

        return true;
    }

    double CalibrationGraph::error() const
    {
        return graph_.error(result_);
    }

    gtsam::NonlinearFactorGraph& CalibrationGraph::graph()
    {
        return graph_;
    }

    const gtsam::NonlinearFactorGraph& CalibrationGraph::graph() const
    {
        return graph_;
    }

    gtsam::Values& CalibrationGraph::initial()
    {
        return initial_;
    }

    const gtsam::Values& CalibrationGraph::initial() const
    {
        return initial_;
    }

    gtsam::Values& CalibrationGraph::result()
    {
        return result_;
    }

    const gtsam::Values& CalibrationGraph::result() const
    {
        return result_;
    }
}


