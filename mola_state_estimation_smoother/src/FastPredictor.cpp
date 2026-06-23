/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
 Closed-source licenses available upon request, for this odometry package
 alone or in combination with the complete SLAM system.
*/

/**
 * @file   FastPredictor.cpp
 * @brief  High-rate pose predictor anchored on the latest backend solution.
 * @author Jose Luis Blanco Claraco
 * @date   Jun 22, 2026
 */

#include "FastPredictor.h"

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <mrpt/math/gtsam_wrappers.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/Lie/SE.h>
#include <mrpt/poses/gtsam_wrappers.h>

#include <algorithm>
#include <vector>

#include "factor_builders.h"

namespace mola::state_estimation_smoother
{

namespace
{
// Mirrors the planar-motion enforcement used by the backend smoother.
void enforce_planar_pose(mrpt::poses::CPose3D& p)
{
    p.z(0);
    p.setYawPitchRoll(p.yaw(), .0, .0);
}
void enforce_planar_twist(mrpt::math::TTwist3D& tw)
{
    tw.vz = 0;
    tw.wx = 0;
    tw.wy = 0;
}
}  // namespace

void FastPredictor::set_anchor(
    const NavState& anchorState, const mrpt::Clock::time_point& anchorStamp,
    const FrameTransforms& frameTransforms)
{
    std::lock_guard<std::mutex> lck(mtx_);
    anchor_          = anchorState;
    anchorStamp_     = anchorStamp;
    frameTransforms_ = frameTransforms;

    // Drop buffered observations strictly older than the new anchor: they are
    // already integrated into the backend solution this anchor came from.
    while (!buffer_.empty() &&
           mrpt::system::timeDifference(buffer_.front()->timestamp, anchorStamp) > 0)
    {
        buffer_.pop_front();
    }
}

std::optional<mrpt::poses::CPose3DPDFGaussian> FastPredictor::frame_transform(
    const std::string& frame_id) const
{
    std::lock_guard<std::mutex> lck(mtx_);
    if (const auto it = frameTransforms_.find(frame_id); it != frameTransforms_.end())
    {
        return it->second;
    }
    return std::nullopt;
}

void FastPredictor::push_observation(
    const mrpt::obs::CObservation::ConstPtr& o, double bufferLength)
{
    if (!o)
    {
        return;
    }
    // The fast layer only uses wheel odometry (the available high-rate relative
    // motion source). IMU attitude/gravity barely affect pose over a <1 s
    // horizon and are left to the backend; LiDAR poses ARE the backend anchor.
    if (!std::dynamic_pointer_cast<const mrpt::obs::CObservationOdometry>(o))
    {
        return;
    }

    std::lock_guard<std::mutex> lck(mtx_);

    // Insert keeping time order (observations usually arrive monotonically, so
    // this is almost always a push_back).
    if (buffer_.empty() ||
        mrpt::system::timeDifference(buffer_.back()->timestamp, o->timestamp) >= 0)
    {
        buffer_.push_back(o);
    }
    else
    {
        auto it = std::upper_bound(
            buffer_.begin(), buffer_.end(), o,
            [](const auto& a, const auto& b)
            { return mrpt::system::timeDifference(a->timestamp, b->timestamp) > 0; });
        buffer_.insert(it, o);
    }

    // Trim history older than bufferLength seconds behind the newest entry.
    const auto newest = buffer_.back()->timestamp;
    while (!buffer_.empty() &&
           mrpt::system::timeDifference(buffer_.front()->timestamp, newest) > bufferLength)
    {
        buffer_.pop_front();
    }
}

void FastPredictor::note_observation_stamp(const mrpt::Clock::time_point& obsStamp)
{
    std::lock_guard<std::mutex> lck(mtx_);
    // Keep the freshest observation stamp (inputs are usually monotonic):
    if (!lastObsStamp_ || mrpt::system::timeDifference(*lastObsStamp_, obsStamp) > 0)
    {
        lastObsStamp_     = obsStamp;
        lastObsWallclock_ = mrpt::Clock::now();
    }
}

std::optional<mrpt::Clock::time_point> FastPredictor::get_current_extrapolated_stamp() const
{
    std::lock_guard<std::mutex> lck(mtx_);
    if (!lastObsStamp_)
    {
        return std::nullopt;
    }
    return mrpt::Clock::fromDouble(
        (mrpt::Clock::nowDouble() - mrpt::Clock::toDouble(lastObsWallclock_)) +
        mrpt::Clock::toDouble(*lastObsStamp_));
}

void FastPredictor::clear()
{
    std::lock_guard<std::mutex> lck(mtx_);
    anchor_.reset();
    frameTransforms_.clear();
    buffer_.clear();
    lastObsStamp_.reset();
}

bool FastPredictor::has_anchor() const
{
    std::lock_guard<std::mutex> lck(mtx_);
    return anchor_.has_value();
}

std::optional<NavState> FastPredictor::predict(
    const mrpt::Clock::time_point& t_query, const Parameters& params) const
{
    // --- 1) Snapshot the anchor + relevant buffered odometry under the lock ---
    NavState                                               anchor;
    mrpt::Clock::time_point                                anchorStamp;
    std::vector<mrpt::obs::CObservationOdometry::ConstPtr> odoms;
    {
        std::lock_guard<std::mutex> lck(mtx_);
        if (!anchor_.has_value())
        {
            return std::nullopt;
        }
        anchor      = *anchor_;
        anchorStamp = anchorStamp_;

        const double anchor_s = mrpt::Clock::toDouble(anchorStamp);
        const double query_s  = mrpt::Clock::toDouble(t_query);
        const double lo       = std::min(anchor_s, query_s);
        const double hi       = std::max(anchor_s, query_s);
        for (const auto& o : buffer_)
        {
            const double s = mrpt::Clock::toDouble(o->timestamp);
            if (s < lo || s > hi)
            {
                continue;
            }
            if (auto odo = std::dynamic_pointer_cast<const mrpt::obs::CObservationOdometry>(o); odo)
            {
                odoms.push_back(odo);
            }
        }
    }

    const double anchor_s = mrpt::Clock::toDouble(anchorStamp);
    const double query_s  = mrpt::Clock::toDouble(t_query);

    // Validity: do not extrapolate too far beyond the freshest available data.
    double newest_s = anchor_s;
    for (const auto& o : odoms)
    {
        newest_s = std::max(newest_s, mrpt::Clock::toDouble(o->timestamp));
    }
    if (std::abs(query_s - newest_s) > params.max_time_to_use_velocity_model)
    {
        return std::nullopt;
    }

    // --- 2) Build the sorted set of node timestamps (merging near-equal ones) -
    const double tol = std::max(1e-4, params.min_time_difference_to_create_new_frame);

    std::vector<double> cand;
    cand.push_back(anchor_s);
    cand.push_back(query_s);
    for (const auto& o : odoms)
    {
        cand.push_back(mrpt::Clock::toDouble(o->timestamp));
    }
    std::sort(cand.begin(), cand.end());

    std::vector<double> nodeStamps;  // representative stamp per node, ascending
    for (const double s : cand)
    {
        if (nodeStamps.empty() || (s - nodeStamps.back()) > tol)
        {
            nodeStamps.push_back(s);
        }
    }

    auto nodeOf = [&](double s) -> mola::id_t
    {
        // nearest representative
        auto   it      = std::lower_bound(nodeStamps.begin(), nodeStamps.end(), s);
        size_t bestIdx = 0;
        double bestDt  = std::numeric_limits<double>::max();
        for (size_t k :
             {it == nodeStamps.begin() ? size_t(0) : size_t(it - nodeStamps.begin() - 1),
              std::min<size_t>(it - nodeStamps.begin(), nodeStamps.size() - 1)})
        {
            const double dt = std::abs(nodeStamps[k] - s);
            if (dt < bestDt)
            {
                bestDt  = dt;
                bestIdx = k;
            }
        }
        return static_cast<mola::id_t>(bestIdx);
    };

    const mola::id_t nAnchor = nodeOf(anchor_s);
    const mola::id_t nQuery  = nodeOf(query_s);

    // --- 3) Assemble the tiny graph + initial values -------------------------
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values               values;

    const gtsam::Vector3 anchorV = {anchor.twist.vx, anchor.twist.vy, anchor.twist.vz};
    const gtsam::Vector3 anchorW = {anchor.twist.wx, anchor.twist.wy, anchor.twist.wz};

    for (size_t k = 0; k < nodeStamps.size(); k++)
    {
        const double dt = nodeStamps[k] - anchor_s;

        // Constant-velocity SE(3) extrapolation of the anchor as the initial
        // guess (same scheme as the backend's estimated_navstate()).
        mrpt::math::CVectorFixed<double, 6> twistDt;
        twistDt[0] = anchor.twist.vx;
        twistDt[1] = anchor.twist.vy;
        twistDt[2] = anchor.twist.vz;
        twistDt[3] = anchor.twist.wx;
        twistDt[4] = anchor.twist.wy;
        twistDt[5] = anchor.twist.wz;
        twistDt *= dt;

        const mrpt::poses::CPose3D mean_k =
            anchor.pose.mean + mrpt::poses::Lie::SE<3>::exp(twistDt);

        values.insert(T(static_cast<mola::id_t>(k)), mrpt::gtsam_wrappers::toPose3(mean_k));
        values.insert(V(static_cast<mola::id_t>(k)), anchorV);
        values.insert(W(static_cast<mola::id_t>(k)), anchorW);
    }

    // 3a) Prior at the anchor node, from the optimized state + covariance.
    {
        mrpt::poses::CPose3DPDFGaussian anchorG;
        anchorG.copyFrom(anchor.pose);  // information -> covariance form

        gtsam::Pose3   priorPose;
        gtsam::Matrix6 priorCov;
        mrpt::gtsam_wrappers::to_gtsam_se3_cov6(anchorG, priorPose, priorCov);
        graph.addPrior(T(nAnchor), priorPose, gtsam::noiseModel::Gaussian::Covariance(priorCov));

        const mrpt::math::CMatrixDouble66 twCov = anchor.twist_inv_cov.inverse_LLt();
        const gtsam::Matrix3              vCov  = twCov.asEigen().block<3, 3>(0, 0);
        const gtsam::Matrix3              wCov  = twCov.asEigen().block<3, 3>(3, 3);
        add_twist_priors(graph, nAnchor, anchorV, vCov, anchorW, wCov);
    }

    // 3b) Constant-velocity kinematic chain between consecutive nodes.
    for (size_t k = 0; k + 1 < nodeStamps.size(); k++)
    {
        const double dt = nodeStamps[k + 1] - nodeStamps[k];
        add_kinematic_factors(
            graph, params, static_cast<mola::id_t>(k), static_cast<mola::id_t>(k + 1), dt);
    }

    // 3c) Wheel-odometry between-factors (relative pose constraints).
    if (odoms.size() >= 2)
    {
        for (size_t i = 0; i + 1 < odoms.size(); i++)
        {
            const auto nFrom = nodeOf(mrpt::Clock::toDouble(odoms[i]->timestamp));
            const auto nTo   = nodeOf(mrpt::Clock::toDouble(odoms[i + 1]->timestamp));
            if (nFrom == nTo)
            {
                continue;  // both snapped to the same node
            }

            // Relative increment, expressed in the body frame of the "from" pose:
            const auto increment = odoms[i + 1]->odometry - odoms[i]->odometry;

            // Same Gaussian motion model as StateEstimationSmoother::fuse_odometry():
            mrpt::obs::CActionRobotMovement2D odoAct;
            odoAct.motionModelConfiguration.modelSelection =
                mrpt::obs::CActionRobotMovement2D::mmGaussian;
            odoAct.motionModelConfiguration.gaussianModel.minStdXY  = 1e-3;
            odoAct.motionModelConfiguration.gaussianModel.minStdPHI = mrpt::DEG2RAD(0.1);
            odoAct.computeFromOdometry(increment, odoAct.motionModelConfiguration);

            mrpt::poses::CPose3DPDFGaussian relPdf;
            relPdf.copyFrom(*odoAct.poseChange);
            relPdf.cov.asEigen().diagonal().array() += 1e-4;  // numerical floor

            gtsam::Pose3   relPose;
            gtsam::Matrix6 relCov;
            mrpt::gtsam_wrappers::to_gtsam_se3_cov6(relPdf, relPose, relCov);

            graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
                T(nFrom), T(nTo), relPose, gtsam::noiseModel::Gaussian::Covariance(relCov));
        }
    }

    // --- 4) Solve + extract the queried node ---------------------------------
    gtsam::Values result;
    try
    {
        gtsam::LevenbergMarquardtParams lmParams;
        result = gtsam::LevenbergMarquardtOptimizer(graph, values, lmParams).optimize();
    }
    catch (const std::exception&)
    {
        return std::nullopt;
    }

    std::optional<gtsam::Marginals> marginals;
    try
    {
        marginals.emplace(graph, result);
    }
    catch (const std::exception&)
    {
        return std::nullopt;
    }

    NavState out;
    try
    {
        const auto qPose = result.at<gtsam::Pose3>(T(nQuery));
        out.pose.mean    = mrpt::poses::CPose3D(mrpt::gtsam_wrappers::toTPose3D(qPose));

        const auto poseCov = gtsam::Matrix6(marginals->marginalCovariance(T(nQuery)));
        out.pose.cov_inv   = mrpt::gtsam_wrappers::to_mrpt_se3_cov6(poseCov).inverse_LLt();

        const auto qV = result.at<gtsam::Vector3>(V(nQuery));
        const auto qW = result.at<gtsam::Vector3>(W(nQuery));
        out.twist     = {qV.x(), qV.y(), qV.z(), qW.x(), qW.y(), qW.z()};

        const auto     vCov     = gtsam::Matrix3(marginals->marginalCovariance(V(nQuery)));
        const auto     wCov     = gtsam::Matrix3(marginals->marginalCovariance(W(nQuery)));
        gtsam::Matrix6 twCov    = gtsam::Matrix6::Zero();
        twCov.block<3, 3>(0, 0) = vCov;
        twCov.block<3, 3>(3, 3) = wCov;
        out.twist_inv_cov       = mrpt::math::CMatrixDouble66(twCov.inverse());
    }
    catch (const std::exception&)
    {
        return std::nullopt;
    }

    if (params.enforce_planar_motion)
    {
        enforce_planar_pose(out.pose.mean);
        enforce_planar_twist(out.twist);
    }

    return out;
}

}  // namespace mola::state_estimation_smoother
