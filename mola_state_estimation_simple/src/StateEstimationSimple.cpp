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
 * @file   StateEstimationSimple.cpp
 * @brief  Fuse of odometry, IMU, and SE(3) pose/twist estimations.
 * @author Jose Luis Blanco Claraco
 * @date   Jan 22, 2024
 */

#include <mola_imu_preintegration/RotationIntegrator.h>
#include <mola_state_estimation_simple/StateEstimationSimple.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/poses/Lie/SO.h>

// arguments: class_name, parent_class, class namespace
IMPLEMENTS_MRPT_OBJECT(StateEstimationSimple, mola::ExecutableBase, mola::state_estimation_simple)

namespace mola::state_estimation_simple
{

StateEstimationSimple::StateEstimationSimple() = default;

void StateEstimationSimple::initialize(const mrpt::containers::yaml& cfg)
{
    this->mrpt::system::COutputLogger::setLoggerName("StateEstimationSimple");

    MRPT_LOG_DEBUG_STREAM("initialize() called with:\n" << cfg << "\n");
    ENSURE_YAML_ENTRY_EXISTS(cfg, "params");

    reset();

    // Load params:
    params.loadFrom(cfg["params"]);
}

void StateEstimationSimple::spinOnce()
{
    // do nothing for this module
}

void StateEstimationSimple::reset()
{
    // reset:
    state_ = State();

    MRPT_LOG_INFO_STREAM("reset() called");
}

void StateEstimationSimple::fuse_odometry(
    const mrpt::obs::CObservationOdometry& odom, [[maybe_unused]] const std::string& odomName)
{
    // this will work well only for simple datasets with one odometry:
    if (state_.last_odom_obs && state_.last_pose)
    {
        const auto poseIncr = odom.odometry - state_.last_odom_obs->odometry;

        state_.last_pose->mean = state_.last_pose->mean + mrpt::poses::CPose3D(poseIncr);

        // We can skip velocity-based model, but retain the twist:
        // state_.last_twist: do not modify
        state_.pose_already_updated_with_odom = true;
    }
    // copy:
    state_.last_odom_obs = odom;

    MRPT_LOG_DEBUG_STREAM("fuse_odometry: odom=" << odom.asString());
}

void StateEstimationSimple::fuse_imu(const mrpt::obs::CObservationIMU& imu)
{
    // Simple approach to integrate IMU readings with angular velocities:
    // 1) Move forward the prediction in time until this observation's time,
    // 2) Assume angular velocity is exactly as measured by this new IMU reading.
    if (!imu.has(mrpt::obs::TIMUDataIndex::IMU_WX) ||  //
        !imu.has(mrpt::obs::TIMUDataIndex::IMU_WY) ||  //
        !imu.has(mrpt::obs::TIMUDataIndex::IMU_WZ))
    {
        MRPT_LOG_THROTTLE_INFO(5.0, "Ignoring IMU reading since it has no angular velocity data");
        return;
    }

    // Do not predict a new pose for this timestamp, so we can use the last *real*
    // call to fuse_pose() from an outter source.

    // and now overwrite twist (wx,wy,wz) part from IMU data:
    mrpt::math::TTwist3D imuReading;
    imuReading.wx = imu.get(mrpt::obs::TIMUDataIndex::IMU_WX);
    imuReading.wy = imu.get(mrpt::obs::TIMUDataIndex::IMU_WY);
    imuReading.wz = imu.get(mrpt::obs::TIMUDataIndex::IMU_WZ);

    // Transform frames: IMU -> vehicle:
    imuReading.rotate(imu.sensorPose.asTPose());

    state_.last_twist->wx = imuReading.wx;
    state_.last_twist->wy = imuReading.wy;
    state_.last_twist->wz = imuReading.wz;

    MRPT_LOG_DEBUG_STREAM("fuse_imu(): new twist: " << state_.last_twist->asString());
}

void StateEstimationSimple::fuse_gnss(const mrpt::obs::CObservationGPS& gps)
{
    // This estimator will just ignore GPS.
    // Refer to the smoother for a more versatile estimator.
    (void)gps;

    MRPT_LOG_DEBUG_STREAM("fuse_gnss(): ignored in this class");
}

void StateEstimationSimple::fuse_pose(
    const mrpt::Clock::time_point& timestamp, const mrpt::poses::CPose3DPDFGaussian& pose,
    [[maybe_unused]] const std::string& frame_id)
{
    mrpt::poses::CPose3D incrPose;

    // numerical sanity: variances>=0 (==0 allowed for some components only)
    for (int i = 0; i < 6; i++) ASSERT_GE_(pose.cov(i, i), .0);
    // and the sum of all strictly >0
    ASSERT_GT_(pose.cov.trace(), .0);

    double dt = 0;
    if (state_.last_pose_obs_tim)
        dt = mrpt::system::timeDifference(*state_.last_pose_obs_tim, timestamp);

    if (dt < 0)
    {
        MRPT_LOG_WARN_STREAM("Ignoring fuse_pose() call with dt=" << dt);
        return;
    }

    MRPT_LOG_DEBUG_STREAM("fuse_pose(): dt=" << dt << " pose=" << pose.mean);
    if (state_.last_twist)
    {
        MRPT_LOG_DEBUG_STREAM("fuse_pose(): twist before=" << state_.last_twist->asString());
    }

    if (dt < params.max_time_to_use_velocity_model && state_.last_pose)
    {
        auto& tw = state_.last_twist.emplace();

        incrPose = pose.mean - (state_.last_pose)->mean;

        tw.vx = incrPose.x() / dt;
        tw.vy = incrPose.y() / dt;
        tw.vz = incrPose.z() / dt;

        const auto logRot = mrpt::poses::Lie::SO<3>::log(incrPose.getRotationMatrix());

        tw.wx = logRot[0] / dt;
        tw.wy = logRot[1] / dt;
        tw.wz = logRot[2] / dt;
    }
    else { state_.last_twist.reset(); }

    if (state_.last_twist)
    {
        MRPT_LOG_DEBUG_STREAM("fuse_pose(): twist after= " << state_.last_twist->asString());
    }

    // save for next iter:
    state_.last_pose                      = pose;
    state_.last_pose_obs_tim              = timestamp;
    state_.pose_already_updated_with_odom = false;
}

void StateEstimationSimple::fuse_twist(
    [[maybe_unused]] const mrpt::Clock::time_point& timestamp, const mrpt::math::TTwist3D& twist,
    [[maybe_unused]] const mrpt::math::CMatrixDouble66& twistCov)
{
    state_.last_twist = twist;
}

std::optional<NavState> StateEstimationSimple::estimated_navstate(
    const mrpt::Clock::time_point& timestamp, [[maybe_unused]] const std::string& frame_id)
{
    if (!state_.last_pose_obs_tim) return {};  // None

    const double dt = mrpt::system::timeDifference(*state_.last_pose_obs_tim, timestamp);

    if (!state_.last_twist || !state_.last_pose ||
        std::abs(dt) > params.max_time_to_use_velocity_model)
        return {};  // None

    NavState ret;

    mrpt::poses::CPose3D poseExtrapolation;

    if (state_.pose_already_updated_with_odom)
    {
        // We have already updated the pose via wheels odometry, don't
        // extrapolate:
        poseExtrapolation = mrpt::poses::CPose3D::Identity();
    }
    else
    {  // normal case: use twist to extrapolate:

        const auto& tw = state_.last_twist.value();

        // For the velocity model, we don't have any known "bias":
        const mola::RotationIntegrationParams rotParams = {};

        const auto rot33 = mola::incremental_rotation({tw.wx, tw.wy, tw.wz}, rotParams, dt);

        poseExtrapolation = mrpt::poses::CPose3D::FromRotationAndTranslation(
            rot33, mrpt::math::TVector3D(tw.vx, tw.vy, tw.vz) * dt);
    }

    // pose mean:
    ret.pose.mean = state_.last_pose->mean + poseExtrapolation;

    // pose cov:
    auto cov = state_.last_pose->cov;

    double varXYZ = mrpt::square(dt * params.sigma_random_walk_acceleration_linear);
    double varRot = mrpt::square(dt * params.sigma_random_walk_acceleration_angular);

    for (int i = 0; i < 3; i++) cov(i, i) += varXYZ;
    for (int i = 3; i < 6; i++) cov(i, i) += varRot;

    ret.pose.cov_inv = cov.inverse_LLt();

    // twist:
    ret.twist = state_.last_twist.value();

    // TODO(jlbc): twist covariance

    return ret;
}

}  // namespace mola::state_estimation_simple
