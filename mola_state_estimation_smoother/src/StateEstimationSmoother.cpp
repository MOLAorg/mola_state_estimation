/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 *
 * Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria
 * Licensed under the GNU GPL v3 for non-commercial applications.
 *
 * This file is part of MOLA.
 * MOLA is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * MOLA is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * MOLA. If not, see <https://www.gnu.org/licenses/>.
 * ------------------------------------------------------------------------- */
/**
 * @file   StateEstimationSmoother.cpp
 * @brief  Fuse of odometry, IMU, and SE(3) pose/twist estimations.
 * @author Jose Luis Blanco Claraco
 * @date   Jan 22, 2024
 */

// MOLA & MRPT:
#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/core/get_env.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/math/gtsam_wrappers.h>
#include <mrpt/poses/Lie/SO.h>
#include <mrpt/poses/gtsam_wrappers.h>

// GTSAM:
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/expressions.h>

// Custom factors:
#include "FactorAngularVelocityIntegration.h"
#include "FactorConstLocalVelocity.h"
#include "FactorTrapezoidalIntegrator.h"

// arguments: class_name, parent_class, class namespace
IMPLEMENTS_MRPT_OBJECT(
    StateEstimationSmoother, mola::ExecutableBase,
    mola::state_estimation_smoother)

namespace mola::state_estimation_smoother
{

const bool NAVSTATE_PRINT_FG = mrpt::get_env<bool>("NAVSTATE_PRINT_FG", false);
const bool NAVSTATE_PRINT_FG_ERRORS =
    mrpt::get_env<bool>("NAVSTATE_PRINT_FG_ERRORS", false);

using gtsam::symbol_shorthand::F;  // Frame of reference origin pose (Pose3)
using gtsam::symbol_shorthand::P;  // Position                       (Point3)
using gtsam::symbol_shorthand::R;  // Rotation                       (Rot3)
using gtsam::symbol_shorthand::V;  // Lin velocity (body frame)      (Point3)
using gtsam::symbol_shorthand::W;  // Ang velocity (body frame)      (Point3)

// -------- GtsamImpl -------

struct StateEstimationSmoother::GtsamImpl
{
    GtsamImpl()  = default;
    ~GtsamImpl() = default;

    gtsam::NonlinearFactorGraph fg;
    gtsam::Values               values;
};

// -------- StateEstimationSmoother::State -------
StateEstimationSmoother::State::State()
    : impl(mrpt::make_impl<StateEstimationSmoother::GtsamImpl>())
{
}
StateEstimationSmoother::State::~State() = default;

StateEstimationSmoother::frameid_t StateEstimationSmoother::State::frame_id(
    const std::string& frame_name)
{
    if (auto it = known_frames.find_key(frame_name);
        it != known_frames.getDirectMap().end())
    {
        return it->second;
    }
    else
    {
        const auto newId = static_cast<frameid_t>(known_frames.size());
        known_frames.insert(frame_name, newId);
        return newId;
    }
}

std::optional<
    std::pair<mrpt::Clock::time_point, StateEstimationSmoother::PointData>>
    StateEstimationSmoother::State::last_pose_of_frame_id(
        const std::string& frameId)
{
    const auto frId = frame_id(frameId);

    for (auto it = data.rbegin(); it != data.rend(); ++it)
    {
        if (!it->second.pose) continue;
        const auto& p = *it->second.pose;
        if (p.frameId != frId) continue;
        return std::make_pair(it->first, it->second);
    }
    return {};
}

// -------- StateEstimationSmoother -------
StateEstimationSmoother::StateEstimationSmoother() = default;

StateEstimationSmoother::~StateEstimationSmoother() = default;

void StateEstimationSmoother::initialize(const mrpt::containers::yaml& cfg)
{
    this->mrpt::system::COutputLogger::setLoggerName("StateEstimationSmoother");

    MRPT_LOG_DEBUG_STREAM("initialize() called with:\n" << cfg << "\n");
    ENSURE_YAML_ENTRY_EXISTS(cfg, "params");

    reset();

    // Load params:
    params.loadFrom(cfg["params"]);
}

void StateEstimationSmoother::spinOnce()
{
    // At the predefined module rate, publish the current estimation,
    // if we have any subscriber:
    if (!anyUpdateLocalizationSubscriber()) return;

    auto lck = mrpt::lockHelper(stateMutex_);

    const auto tNowOpt = state_.get_current_extrapolated_stamp();
    if (!tNowOpt)
    {
        MRPT_LOG_THROTTLE_WARN(
            5.0, "Cannot publish vehicle pose (no input data yet?)");
        return;
    }

    const auto nv = estimated_navstate(*tNowOpt, params.reference_frame_name);
    if (!nv)
    {
        MRPT_LOG_THROTTLE_WARN(
            5.0, "Cannot publish vehicle pose (stalled input data?)");
        return;
    }

    LocalizationUpdate lu;
    lu.child_frame     = params.vehicle_frame_name;
    lu.reference_frame = params.reference_frame_name;

    lu.method    = "state_estimator";
    lu.quality   = 1;
    lu.timestamp = *tNowOpt;
    lu.pose      = nv->pose.getPoseMean().asTPose();
    lu.cov       = nv->pose.cov_inv.inverse();

    MRPT_LOG_DEBUG_FMT(
        "Publishing timely pose estimate: t=%f pose=%s",
        mrpt::Clock::toDouble(*tNowOpt), lu.pose.asString().c_str());

    advertiseUpdatedLocalization(lu);
}

void StateEstimationSmoother::reset()
{
    auto lck = mrpt::lockHelper(stateMutex_);

    // reset:
    state_ = State();
}

void StateEstimationSmoother::fuse_odometry(
    const mrpt::obs::CObservationOdometry& odom, const std::string& odomName)
{
    using namespace std::string_literals;

    auto lck = mrpt::lockHelper(stateMutex_);

    state_.update_last_input_stamp(odom.timestamp);

    THROW_EXCEPTION("finish implementation!");

    ASSERT_(!odomName.empty());

    OdomData d;
    d.frameId = state_.frame_id(odomName);
    d.pose    = mrpt::poses::CPose3D(odom.odometry);

    state_.data.insert({odom.timestamp, d});

    delete_too_old_entries();

    MRPT_LOG_DEBUG_FMT(
        "fuse_odometry: t=%f name=%s pose=%s",
        mrpt::Clock::toDouble(odom.timestamp), odomName.c_str(),
        odom.odometry.asString().c_str());
}

void StateEstimationSmoother::fuse_imu(const mrpt::obs::CObservationIMU& imu)
{
    auto lck = mrpt::lockHelper(stateMutex_);

    state_.update_last_input_stamp(imu.timestamp);

    THROW_EXCEPTION("TODO");
    (void)imu;

    delete_too_old_entries();

    MRPT_LOG_DEBUG_FMT("fuse_imu: t=%f", mrpt::Clock::toDouble(imu.timestamp));
}

void StateEstimationSmoother::fuse_gnss(const mrpt::obs::CObservationGPS& gps)
{
    auto lck = mrpt::lockHelper(stateMutex_);

    state_.update_last_input_stamp(gps.timestamp);

    THROW_EXCEPTION("TODO");
    (void)gps;

    delete_too_old_entries();

    MRPT_LOG_DEBUG_FMT("fuse_gnss: t=%f", mrpt::Clock::toDouble(gps.timestamp));
}

void StateEstimationSmoother::fuse_pose(
    const mrpt::Clock::time_point&         timestamp,
    const mrpt::poses::CPose3DPDFGaussian& pose, const std::string& frame_id)
{
    auto lck = mrpt::lockHelper(stateMutex_);

    state_.update_last_input_stamp(timestamp);

    // find last KF of this frame_id before adding the new one:
    const auto lastKF = state_.last_pose_of_frame_id(frame_id);

    // numerical sanity:
    for (int i = 0; i < 6; i++) ASSERT_GT_(pose.cov(i, i), .0);

    PoseData d;
    d.frameId = state_.frame_id(frame_id);
    d.pose    = pose;

    // Help with initial pose hint:
    KinematicState newGuess;
    if (lastKF.has_value())
    {
        const auto poseIncr = pose.mean - lastKF->second.pose->pose.mean;
        newGuess.pose       = lastKF->second.last_known_state.pose + poseIncr;
        newGuess.twist      = lastKF->second.last_known_state.twist;
    }

    state_.data.insert({timestamp, {d, newGuess}});
    delete_too_old_entries();

    MRPT_LOG_DEBUG_FMT(
        "fuse_pose: t=%f frame='%s' p=%s sigmas=%.02e %.02e %.02e (m) %.02e "
        "%.02e %.02e (deg)",
        mrpt::Clock::toDouble(timestamp), frame_id.c_str(),
        pose.mean.asString().c_str(), std::sqrt(pose.cov(0, 0)),
        std::sqrt(pose.cov(1, 1)), std::sqrt(pose.cov(2, 2)),
        mrpt::RAD2DEG(std::sqrt(pose.cov(3, 3))),
        mrpt::RAD2DEG(std::sqrt(pose.cov(4, 4))),
        mrpt::RAD2DEG(std::sqrt(pose.cov(5, 5))));

    // Estimate twist:
    // If we add an additional direct observation of twist, the result is
    // more accurate for coarser time steps:
    if (!lastKF) return;

    const double dt = mrpt::system::timeDifference(lastKF->first, timestamp);

    if (dt > params.max_time_to_use_velocity_model) return;

    ASSERT_GT_(dt, .0);

    mrpt::math::TTwist3D tw;

    const auto incrPosePdf = pose - lastKF->second.pose->pose;
    const auto incrPose    = incrPosePdf.mean;

    tw.vx = incrPose.x() / dt;
    tw.vy = incrPose.y() / dt;
    tw.vz = incrPose.z() / dt;

    const auto logRot =
        mrpt::poses::Lie::SO<3>::log(incrPose.getRotationMatrix());

    tw.wx = logRot[0] / dt;
    tw.wy = logRot[1] / dt;
    tw.wz = logRot[2] / dt;

#if 0
    // Estimate twist cov:
    auto twCov = mrpt::math::CMatrixDouble66(
        incrPosePdf.cov.asEigen() * (1.0 / (dt * dt)));
#endif

    // Ensure minimum covariance for avoiding overconfidence and numerical
    // illness:
    auto twCov = mrpt::math::CMatrixDouble66::Zero();
    for (int i = 0; i < 3; i++)
    {
        twCov(i, i) +=
            mrpt::square(params.sigma_twist_from_consecutive_poses_linear);
        twCov(3 + i, 3 + i) +=
            mrpt::square(params.sigma_twist_from_consecutive_poses_angular);
    }

    this->fuse_twist(timestamp, tw, twCov);
}

void StateEstimationSmoother::fuse_twist(
    const mrpt::Clock::time_point& timestamp, const mrpt::math::TTwist3D& twist,
    const mrpt::math::CMatrixDouble66& twistCov)
{
    auto lck = mrpt::lockHelper(stateMutex_);

    state_.update_last_input_stamp(timestamp);

    TwistData d;
    d.twist    = twist;
    d.twistCov = twistCov;

    state_.data.insert({timestamp, d});

    delete_too_old_entries();

    MRPT_LOG_DEBUG_FMT(
        "fuse_twist: t=%f twist=%s sigmas=%.02e %.02e %.02e (m) %.02e %.02e "
        "%.02e (deg)",
        mrpt::Clock::toDouble(timestamp), twist.asString().c_str(),
        std::sqrt(twistCov(0, 0)), std::sqrt(twistCov(1, 1)),
        std::sqrt(twistCov(2, 2)), mrpt::RAD2DEG(std::sqrt(twistCov(3, 3))),
        mrpt::RAD2DEG(std::sqrt(twistCov(4, 4))),
        mrpt::RAD2DEG(std::sqrt(twistCov(5, 5))));
}

std::optional<NavState> StateEstimationSmoother::estimated_navstate(
    const mrpt::Clock::time_point& timestamp, const std::string& frame_id)
{
    return build_and_optimize_fg(timestamp, frame_id);
}

std::set<std::string> StateEstimationSmoother::known_frame_ids()
{
    auto lck = mrpt::lockHelper(stateMutex_);

    std::set<std::string> ret;
    for (const auto& [name, id] : state_.known_frames.getDirectMap())
        ret.insert(name);

    return ret;
}

namespace
{
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

std::optional<NavState> StateEstimationSmoother::build_and_optimize_fg(
    const mrpt::Clock::time_point queryTimestamp, const std::string& frame_id)
{
    using namespace std::string_literals;

    mrpt::system::CTimeLoggerEntry tle(profiler_, "build_and_optimize_fg");

    auto lck = mrpt::lockHelper(stateMutex_);

    delete_too_old_entries();

    // Return an empty answer if we don't have data, or we would need to
    // extrapolate too much:
    if (state_.data.empty() || state_.known_frames.empty()) return {};
    {
        const double tq_2_tfirst = mrpt::system::timeDifference(
            queryTimestamp, state_.data.begin()->first);
        const double tlast_2_tq = mrpt::system::timeDifference(
            state_.data.rbegin()->first, queryTimestamp);
        if (tq_2_tfirst > params.max_time_to_use_velocity_model ||
            tlast_2_tq > params.max_time_to_use_velocity_model)
        {
            MRPT_LOG_DEBUG_STREAM(
                "[build_and_optimize_fg] Skipping due to need to extrapolate "
                "too much: tq_2_tfirst="
                << tq_2_tfirst << " tlast_2_tq=" << tlast_2_tq
                << " max_time_to_use_velocity_model="
                << params.max_time_to_use_velocity_model);

            return {};
        }
    }

    // shortcuts:
    auto& fg     = state_.impl->fg;
    auto& values = state_.impl->values;

    values.clear();
    fg.resize(0);

    // Build the sequence of time points:
    // FG variable indices will use the indices in this vector:
    auto& dQuery = state_.data[queryTimestamp];
    dQuery.query = QueryPointData();

    using map_it_t = std::map<mrpt::Clock::time_point, PointData>::value_type;

    std::vector<map_it_t*> entries;
    std::optional<size_t>  query_KF_id;
    for (auto& it : state_.data)
    {
        if (it.first == queryTimestamp) query_KF_id = entries.size();

        entries.push_back(&it);
    }
    ASSERT_(query_KF_id.has_value());

    // add const vel kinematic factors between consecutive KFs:
    ASSERT_(entries.size() >= 1);

    for (size_t i = 1; i < entries.size(); i++)
    {
        mola::FactorConstVelKinematics f;
        f.from_kf_   = i - 1;
        f.to_kf_     = i;
        f.deltaTime_ = mrpt::system::timeDifference(
            entries[i - 1]->first, entries[i]->first);

        addFactor(f);
    }

    // Init values:
    for (size_t i = 0; i < entries.size(); i++)
    {
        const auto& e = entries[i]->second;

        const auto lastPose =
            mrpt::gtsam_wrappers::toPose3(e.last_known_state.pose);
        state_.impl->values.insert<gtsam::Point3>(P(i), lastPose.translation());
        state_.impl->values.insert<gtsam::Rot3>(R(i), lastPose.rotation());

        const auto& tw = e.last_known_state.twist;
        state_.impl->values.insert<gtsam::Point3>(
            V(i), gtsam::Vector3(tw.vx, tw.vy, tw.vz));
        state_.impl->values.insert<gtsam::Point3>(
            W(i), gtsam::Vector3(tw.wx, tw.wy, tw.wz));
    }
    for (const auto& [frameName, frameId] : state_.known_frames.getDirectMap())
    {
        // F(0): this variable is not used.
        // We only need to estimate F(i), the SE(3) pose of the frame_id "i" wrt
        // "0" (see paper diagrams!)
        if (frameId == 0) continue;

        // TODO: Save and reuse last optimized value!
        state_.impl->values.insert<gtsam::Pose3>(
            F(frameId), gtsam::Pose3::Identity());
    }

    // Add planar constraints:
    if (params.enforce_planar_motion)
    {
        const double XY_SIGMA       = 1e10;
        const double Z_SIGMA        = 1e-4;
        const auto   planar_z_noise = gtsam::noiseModel::Diagonal::Sigmas(
              gtsam::Vector3(XY_SIGMA, XY_SIGMA, Z_SIGMA));

        for (size_t i = 0; i < entries.size(); i++)
        {
            state_.impl->fg.addPrior(
                P(i), gtsam::Vector3(0, 0, 0), planar_z_noise);
        }
    }

    // Unary prior for initial twist:
    const auto& tw = params.initial_twist;
    fg.addPrior(
        V(0), gtsam::Vector3(tw.vx, tw.vy, tw.vz),
        gtsam::noiseModel::Isotropic::Sigma(3, params.initial_twist_sigma_lin));

    fg.addPrior(
        W(0), gtsam::Vector3(tw.wx, tw.wy, tw.wz),
        gtsam::noiseModel::Isotropic::Sigma(3, params.initial_twist_sigma_ang));

    // Process pose observations:
    // ------------------------------------------
    for (size_t kfId = 0; kfId < entries.size(); kfId++)
    {
        // const auto  tim = entries.at(kfId)->first;
        const auto& d = entries.at(kfId)->second;

        // ---------------------------------
        // Data point of type: Pose
        // ---------------------------------
        if (d.pose.has_value())
        {
            if (d.pose->frameId == 0)
            {
                // Pose observations in the first frame are just priors:
                // (see paper!)

                gtsam::Pose3   p;
                gtsam::Matrix6 pCov;
                mrpt::gtsam_wrappers::to_gtsam_se3_cov6(d.pose->pose, p, pCov);

                {
                    auto noisePos = gtsam::noiseModel::Gaussian::Covariance(
                        pCov.block<3, 3>(3, 3));

                    gtsam::noiseModel::Base::shared_ptr robNoisePos;
                    if (params.robust_param > 0)
                        robNoisePos = gtsam::noiseModel::Robust::Create(
                            gtsam::noiseModel::mEstimator::GemanMcClure::Create(
                                params.robust_param),
                            noisePos);
                    else
                        robNoisePos = noisePos;

                    fg.addPrior(P(kfId), p.translation(), robNoisePos);
                }

                {
                    auto noiseRot = gtsam::noiseModel::Gaussian::Covariance(
                        pCov.block<3, 3>(0, 0));

                    gtsam::noiseModel::Base::shared_ptr robNoiseRot;
                    if (params.robust_param > 0)
                        robNoiseRot = gtsam::noiseModel::Robust::Create(
                            gtsam::noiseModel::mEstimator::GemanMcClure::Create(
                                params.robust_param),
                            noiseRot);
                    else
                        robNoiseRot = noiseRot;

                    fg.addPrior(R(kfId), p.rotation(), robNoiseRot);
                }
            }
            else
            {
                // Pose observations in subsequent frames are more complex:
                // (see paper!)
                THROW_EXCEPTION("todo");
            }
        }

        // ---------------------------------
        // Data point of type: Twist
        // ---------------------------------
        if (d.twist.has_value())
        {
            const auto&          pd   = d.twist.value();
            const gtsam::Vector3 v    = {pd.twist.vx, pd.twist.vy, pd.twist.vz};
            const gtsam::Vector3 w    = {pd.twist.wx, pd.twist.wy, pd.twist.wz};
            gtsam::Matrix3       vCov = pd.twistCov.asEigen().block<3, 3>(0, 0);
            gtsam::Matrix3       wCov = pd.twistCov.asEigen().block<3, 3>(3, 3);

            {
                auto noiseV = gtsam::noiseModel::Gaussian::Covariance(vCov);
                gtsam::noiseModel::Base::shared_ptr robNoiseV;
                if (params.robust_param > 0)
                    robNoiseV = gtsam::noiseModel::Robust::Create(
                        gtsam::noiseModel::mEstimator::GemanMcClure::Create(
                            params.robust_param),
                        noiseV);
                else
                    robNoiseV = noiseV;

                fg.addPrior(V(kfId), v, robNoiseV);
            }
            {
                auto noiseW = gtsam::noiseModel::Gaussian::Covariance(wCov);
                gtsam::noiseModel::Base::shared_ptr robNoiseW;
                if (params.robust_param > 0)
                    robNoiseW = gtsam::noiseModel::Robust::Create(
                        gtsam::noiseModel::mEstimator::GemanMcClure::Create(
                            params.robust_param),
                        noiseW);
                else
                    robNoiseW = noiseW;

                fg.addPrior(W(kfId), w, robNoiseW);
            }
        }

    }  // end for each kfId

    // FG is built: optimize it
    // -------------------------------------
    if (NAVSTATE_PRINT_FG)
    {
        fg.print();
        state_.impl->values.print();

#if 0
        fg.saveGraph("fg.dot");
#endif
    }

    gtsam::LevenbergMarquardtOptimizer lm(fg, state_.impl->values);

    const auto& optimal = lm.optimize();

    const double final_rmse = std::sqrt(fg.error(optimal) / fg.size());

    MRPT_LOG_DEBUG_STREAM(
        "[build_and_optimize_fg] LM ran for "
        << lm.iterations() << " iterations, " << fg.size() << " factors, "
        << state_.data.size() << " KFs, RMSE: "
        << std::sqrt(fg.error(state_.impl->values) / fg.size()) << " => "
        << final_rmse);

    if (NAVSTATE_PRINT_FG)
    {
        optimal.print("Optimized:");
        std::cout << "\n query_KF_id: " << *query_KF_id << std::endl;

        MRPT_LOG_INFO_STREAM("state_.data: ");
        for (const auto& [k, v] : state_.data)
        {
            MRPT_LOG_INFO_FMT(
                "t=%f: %s", mrpt::Clock::toDouble(k), v.asString().c_str());
        }
    }

    if (NAVSTATE_PRINT_FG_ERRORS)
    {
        fg.printErrors(optimal, "Errors for optimized values:");
    }

    // final sanity check:
    if (final_rmse > params.max_rmse)
    {
        MRPT_LOG_WARN_STREAM(
            "[build_and_optimize_fg] Discarding solution due to high "
            "RMSE="
            << final_rmse);

        return {};
    }

    // Extract results from the factor graph:
    // ----------------------------------------------
    NavState out;

    // SE(3) pose:
    auto poseResult = gtsam::Pose3(
        optimal.at<gtsam::Rot3>(R(*query_KF_id)),
        optimal.at<gtsam::Point3>(P(*query_KF_id)));
    const auto outPose = mrpt::gtsam_wrappers::toTPose3D(poseResult);
    out.pose.mean      = mrpt::poses::CPose3D(outPose);

    gtsam::Marginals marginals(fg, optimal);

    // Pose SE(3) cov: (in mrpt order is xyz, then yaw/pitch/roll):
    gtsam::Matrix6 cov_inv    = gtsam::Matrix6::Zero();
    cov_inv.block<3, 3>(0, 0) = marginals.marginalInformation(P(*query_KF_id));
    cov_inv.block<3, 3>(3, 3) = marginals.marginalInformation(R(*query_KF_id));
    out.pose.cov_inv          = cov_inv;

    // Twist (already in body frame in the factor graph):
    const auto linVel = optimal.at<gtsam::Vector3>(V(*query_KF_id));
    const auto angVel = optimal.at<gtsam::Vector3>(W(*query_KF_id));

    out.twist.vx = linVel.x();
    out.twist.vy = linVel.y();
    out.twist.vz = linVel.z();
    out.twist.wx = angVel.x();
    out.twist.wy = angVel.y();
    out.twist.wz = angVel.z();

    // Twist cov:
    gtsam::Matrix6 tw_cov_inv = gtsam::Matrix6::Zero();
    tw_cov_inv.block<3, 3>(0, 0) =
        marginals.marginalInformation(V(*query_KF_id));
    tw_cov_inv.block<3, 3>(3, 3) =
        marginals.marginalInformation(W(*query_KF_id));
    out.twist_inv_cov = tw_cov_inv;

    // delete temporary entry:
    dQuery.query.reset();
    if (dQuery.empty()) state_.data.erase(queryTimestamp);

    // Save optimized values into data entries to bootstrap optimization
    // in next iterations:
    // -------------------------------------------------------------------
    for (size_t i = 0; i < entries.size(); i++)
    {
        auto& e = entries[i]->second;

        const auto pose = gtsam::Pose3(
            optimal.at<gtsam::Rot3>(R(i)), optimal.at<gtsam::Point3>(P(i)));

        auto& ks = e.last_known_state;

        ks.pose = mrpt::poses::CPose3D::FromRotationAndTranslation(
            pose.rotation().matrix(), pose.translation());

        const auto linV = optimal.at<gtsam::Vector3>(V(i));
        const auto angV = optimal.at<gtsam::Vector3>(W(i));
        ks.twist.vx     = linV.x();
        ks.twist.vy     = linV.y();
        ks.twist.vz     = linV.z();
        ks.twist.wx     = angV.x();
        ks.twist.wy     = angV.y();
        ks.twist.wz     = angV.z();

        if (params.enforce_planar_motion)
        {
            enforce_planar_pose(ks.pose);
            enforce_planar_twist(ks.twist);
        }
    }

    // Honor requested frame_id:
    // ----------------------------------
    ASSERTMSG_(
        state_.known_frames.hasKey(frame_id),
        "Requested results in unknown frame_id: '"s + frame_id + "'"s);

    // if this is the first frame_id, we are already done, otherwise,
    // recover and apply the transformation:
    if (const frameid_t frameId = state_.known_frames.direct(frame_id);
        frameId != 0)
    {
        // Apply F(frameId) transformation on the left:
        const auto T = optimal.at<gtsam::Pose3>(F(frameId));

        THROW_EXCEPTION("TODO");
    }

    return out;
}

/// Implementation of Eqs (1),(4) in the MOLA RSS2019 paper.
void StateEstimationSmoother::addFactor(const mola::FactorConstVelKinematics& f)
{
#if 0
    MRPT_LOG_DEBUG_STREAM(
        "[addFactor] FactorConstVelKinematics: "
        << f.from_kf_ << " ==> " << f.to_kf_ << " dt=" << f.deltaTime_);
#endif

    // Add const-vel factor to gtsam itself:
    double dt = f.deltaTime_;

    // trick to easily handle queries on exactly an existing keyframe:
    if (dt == 0) dt = 1e-5;

    ASSERT_GT_(dt, 0.);

    // errors in constant vel:
    const double std_linvel = params.sigma_random_walk_acceleration_linear;
    const double std_angvel = params.sigma_random_walk_acceleration_angular;

    if (dt > params.time_between_frames_to_warning)
    {
        MRPT_LOG_WARN_FMT(
            "A constant-velocity kinematics factor has been added for a "
            "dT=%.03f s.",
            dt);
    }

    // 1) Add GTSAM factors for constant velocity model
    // -------------------------------------------------

    auto Pi  = gtsam::Point3_(P(f.from_kf_));
    auto Pj  = gtsam::Point3_(P(f.to_kf_));
    auto Ri  = gtsam::Rot3_(R(f.from_kf_));
    auto Rj  = gtsam::Rot3_(R(f.to_kf_));
    auto bVi = gtsam::Point3_(V(f.from_kf_));
    auto bVj = gtsam::Point3_(V(f.to_kf_));
    auto bWi = gtsam::Point3_(W(f.from_kf_));
    auto bWj = gtsam::Point3_(W(f.to_kf_));

    const auto kPi  = P(f.from_kf_);
    const auto kPj  = P(f.to_kf_);
    const auto kbVi = V(f.from_kf_);
    const auto kbVj = V(f.to_kf_);
    const auto kRi  = R(f.from_kf_);
    const auto kRj  = R(f.to_kf_);
    const auto kbWi = W(f.from_kf_);
    const auto kbWj = W(f.to_kf_);

    // See line 3 of eq (4) in the MOLA RSS2019 paper
    // Modify to use velocity in local frame: reuse FactorConstLocalVelocity
    // here too:
    state_.impl->fg.emplace_shared<FactorConstLocalVelocity>(
        kRi, kbVi, kRj, kbVj,
        gtsam::noiseModel::Isotropic::Sigma(3, std_linvel * dt));

    // \omega is in the body frame, we need a special factor to rotate it:
    // See line 4 of eq (4) in the MOLA RSS2019 paper.
    state_.impl->fg.emplace_shared<FactorConstLocalVelocity>(
        kRi, kbWi, kRj, kbWj,
        gtsam::noiseModel::Isotropic::Sigma(3, std_angvel * dt));

    // 2) Add kinematics / numerical integration factor
    // ---------------------------------------------------
    auto noise_kinematicsPosition = gtsam::noiseModel::Isotropic::Sigma(
        3, params.sigma_integrator_position);

    auto noise_kinematicsOrientation = gtsam::noiseModel::Isotropic::Sigma(
        3, params.sigma_integrator_orientation);

    // Impl. line 2 of eq (1) in the MOLA RSS2019 paper
    state_.impl->fg.emplace_shared<FactorTrapezoidalIntegrator>(
        kPi, kbVi, kRi, kPj, kbVj, kRj, dt, noise_kinematicsPosition);

    // Impl. line 1 of eq (4) in the MOLA RSS2019 paper.
    state_.impl->fg.emplace_shared<FactorAngularVelocityIntegration>(
        kRi, kbWi, kRj, dt, noise_kinematicsOrientation);
}

void StateEstimationSmoother::addFactor(const mola::FactorTricycleKinematics& f)
{
    THROW_EXCEPTION("Write me!");
    (void)f;
}

void StateEstimationSmoother::delete_too_old_entries()
{
    auto lck = mrpt::lockHelper(stateMutex_);

    if (state_.data.empty()) return;

    const double newestTime =
        mrpt::Clock::toDouble(state_.data.rbegin()->first);
    const double minTime = newestTime - params.sliding_window_length;

    for (auto it = state_.data.begin(); it != state_.data.end();)
    {
        const double t = mrpt::Clock::toDouble(it->first);
        if (t < minTime)
        {
            // remove it:
            it = state_.data.erase(it);
        }
        else { ++it; }
    }
}

std::string StateEstimationSmoother::PointData::asString() const
{
    std::ostringstream ss;

    if (pose) ss << "pose: " << pose->pose.mean << " ";
    if (odom) ss << "odom: " << odom->pose << " ";
    if (twist) ss << "twist: " << twist->twist.asString() << " ";
    if (query) ss << "query";

    return ss.str();
}

}  // namespace mola::state_estimation_smoother
