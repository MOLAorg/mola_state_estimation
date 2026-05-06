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
 * @file   StateEstimationSimple.cpp
 * @brief  Fuse of odometry, IMU, and SE(3) pose/twist estimations.
 * @author Jose Luis Blanco Claraco
 * @date   Jan 22, 2024
 */

#include <mola_imu_preintegration/ImuIntegrator.h>
#include <mola_state_estimation_simple/StateEstimationSimple.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/obs/CObservationRobotPose.h>
#include <mrpt/poses/Lie/SO.h>

// arguments: class_name, parent_class, class namespace
IMPLEMENTS_MRPT_OBJECT(StateEstimationSimple, mola::ExecutableBase, mola::state_estimation_simple)

namespace mola::state_estimation_simple
{

StateEstimationSimple::StateEstimationSimple() = default;

void StateEstimationSimple::initialize(const mrpt::containers::yaml& cfg)
{
    auto lck = std::scoped_lock(state_mtx_);

    this->mrpt::system::COutputLogger::setLoggerName("StateEstimationSimple");

    MRPT_LOG_DEBUG_STREAM("initialize() called with:\n" << cfg << "\n");
    ENSURE_YAML_ENTRY_EXISTS(cfg, "params");

    // reset:
    state_ = State();

    // Load params:
    params.loadFrom(cfg["params"]);

    // Initialize parent:
    mola::NavStateFilter::initialize(cfg);
}

void StateEstimationSimple::spinOnce()
{
    // do nothing for this module
}

void StateEstimationSimple::reset()
{
    auto lck = std::scoped_lock(state_mtx_);

    // reset:
    state_ = State();

    MRPT_LOG_INFO_STREAM("reset() called");
}

void StateEstimationSimple::fuse_odometry(
    const mrpt::obs::CObservationOdometry& odom, [[maybe_unused]] const std::string& odomName)
{
    auto lck = std::scoped_lock(state_mtx_);

    // Advance last_pose by the incremental 2D odometry delta:
    if (state_.last_odom_obs && state_.last_pose)
    {
        const auto poseIncr    = odom.odometry - state_.last_odom_obs->odometry;
        state_.last_pose->mean = state_.last_pose->mean + mrpt::poses::CPose3D(poseIncr);
        state_.pose_already_updated_with_odom = true;
    }
    state_.last_odom_obs = odom;

    // Use wheel velocities when available: they give a correct, uncontaminated
    // twist for de-skewing and sigma computation, independently of whether
    // LiDAR ICP has produced a new pose yet.
    if (odom.hasVelocities)
    {
        if (!state_.last_twist)
        {
            state_.last_twist.emplace();
        }
        auto& tw = *state_.last_twist;
        tw.vx    = odom.velocityLocal.vx;
        tw.vy    = odom.velocityLocal.vy;
        tw.vz    = 0;
        tw.wz    = odom.velocityLocal.omega;
        // wx, wy: left as-is (set by fuse_imu() when IMU is active, or zero
        // from default construction above when it is not).
        // Note: fuse_imu() still overrides wx/wy/wz whenever it runs.

        const double varXYZ = mrpt::square(0.1);  // [m²/s²]
        const double varRot = mrpt::square(0.05);  // [rad²/s²]
        auto&        cov    = state_.last_twist_cov.emplace();
        cov.setDiagonal({varXYZ, varXYZ, varXYZ, varRot, varRot, varRot});

        MRPT_LOG_DEBUG_STREAM("fuse_odometry: twist from velocityLocal: " << tw.asString());
    }

    MRPT_LOG_DEBUG_STREAM("fuse_odometry: odom=" << odom.asString());
}

void StateEstimationSimple::fuse_odometry_3d_pose(
    const mrpt::obs::CObservationRobotPose& obs, const std::string& odomName)
{
    auto lck = std::scoped_lock(state_mtx_);

    // Apply sensor-to-base correction if the sensor is not at the origin:
    auto sensedPose = obs.pose;
    if (obs.sensorPose != mrpt::poses::CPose3D())
    {
        sensedPose = sensedPose + mrpt::poses::CPose3DPDFGaussian(-obs.sensorPose);
    }

    auto& src = state_.per_source[odomName];

    // Compute and apply the incremental delta to last_pose, keeping it in the
    // LiDAR SLAM frame rather than replacing it with the absolute odom pose
    // (which lives in a potentially offset odometry reference frame).
    if (src.last_pose.has_value() && state_.last_pose.has_value())
    {
        const double dt =
            src.last_obs_tim ? mrpt::system::timeDifference(*src.last_obs_tim, obs.timestamp) : 0.0;

        if (dt < 0)
        {
            MRPT_LOG_THROTTLE_WARN_STREAM(
                5.0, "fuse_odometry_3d_pose(): backwards timestamp for source '"
                         << odomName << "', dt=" << dt << ". Resetting source.");
            src.last_pose    = sensedPose;
            src.last_obs_tim = obs.timestamp;
            return;
        }

        const auto delta       = sensedPose.mean - src.last_pose->mean;
        state_.last_pose->mean = state_.last_pose->mean + delta;
        // pose_already_updated_with_odom is NOT set here because
        // last_pose_obs_tim is updated to obs.timestamp below, so
        // estimated_navstate() will compute the correct dt and extrapolate
        // normally. (Contrast with fuse_odometry() which does NOT update
        // last_pose_obs_tim and must suppress extrapolation via the flag.)

        // Derive twist from the per-source consecutive 3D odom poses.
        // This gives the correct wheel-odometry velocity independently of how
        // last_pose has been set by LiDAR ICP.
        if (dt > 0 && dt < params.max_time_to_use_velocity_model)
        {
            auto&        tw     = state_.last_twist.emplace();
            const auto   logRot = mrpt::poses::Lie::SO<3>::log(delta.getRotationMatrix());
            const double dt2    = dt * dt;

            tw.vx = delta.x() / dt;
            tw.vy = delta.y() / dt;
            tw.vz = delta.z() / dt;
            tw.wx = logRot[0] / dt;
            tw.wy = logRot[1] / dt;
            tw.wz = logRot[2] / dt;

            auto& twistCov = state_.last_twist_cov.emplace();
            twistCov.setDiagonal(
                {mrpt::square(params.sigma_relative_pose_linear) / dt2,
                 mrpt::square(params.sigma_relative_pose_linear) / dt2,
                 mrpt::square(params.sigma_relative_pose_linear) / dt2,
                 mrpt::square(params.sigma_relative_pose_angular) / dt2,
                 mrpt::square(params.sigma_relative_pose_angular) / dt2,
                 mrpt::square(params.sigma_relative_pose_angular) / dt2});

            MRPT_LOG_DEBUG_STREAM(
                "fuse_odometry_3d_pose('" << odomName << "'): twist=" << tw.asString());
        }
    }

    src.last_pose    = sensedPose;
    src.last_obs_tim = obs.timestamp;

    // Bootstrap last_pose when no SLAM source has set it yet, so that
    // estimated_navstate() can return valid results when CObservationRobotPose
    // is the sole pose source (e.g., in tests or lidar-odom-only setups).
    // When fuse_pose() is also active it owns last_pose_obs_tim and overwrites
    // it using the per-source src.last_obs_tim guard, so this update is safe.
    if (!state_.last_pose.has_value())
    {
        state_.last_pose = sensedPose;
    }
    state_.last_pose_obs_tim = obs.timestamp;

    MRPT_LOG_DEBUG_STREAM("fuse_odometry_3d_pose('" << odomName << "'): pose=" << sensedPose.mean);
}

void StateEstimationSimple::fuse_imu(const mrpt::obs::CObservationIMU& imu)
{
    auto lck = std::scoped_lock(state_mtx_);

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

    // IMU only observes angular velocity: preserve linear (vx,vy,vz) from the
    // last fuse_pose().
    if (!state_.last_twist)
    {
        state_.last_twist.emplace();
    }
    state_.last_twist->wx = imuReading.wx;
    state_.last_twist->wy = imuReading.wy;
    state_.last_twist->wz = imuReading.wz;

    const double varRot = mrpt::square(params.sigma_imu_angular_velocity);
    if (!state_.last_twist_cov)
    {
        const double varXYZ_no_info = mrpt::square(5.0);  // wide linear prior, [m²/s²]
        auto&        twistCov       = state_.last_twist_cov.emplace();
        twistCov.setDiagonal(
            {varXYZ_no_info, varXYZ_no_info, varXYZ_no_info, varRot, varRot, varRot});
    }
    else
    {
        auto& twistCov = *state_.last_twist_cov;
        twistCov(3, 3) = varRot;
        twistCov(4, 4) = varRot;
        twistCov(5, 5) = varRot;
    }

    MRPT_LOG_DEBUG_STREAM("fuse_imu(): new twist: " << state_.last_twist->asString());
}

void StateEstimationSimple::fuse_gnss(const mrpt::obs::CObservationGPS& gps)
{
    auto lck = std::scoped_lock(state_mtx_);

    // This estimator will just ignore GPS.
    // Refer to the smoother for a more versatile estimator.
    (void)gps;

    MRPT_LOG_DEBUG_STREAM("fuse_gnss(): ignored in this class");
}

void StateEstimationSimple::fuse_pose(
    const mrpt::Clock::time_point& timestamp, const mrpt::poses::CPose3DPDFGaussian& pose,
    const std::string& frame_id)
{
    auto lck = std::scoped_lock(state_mtx_);

    // Numerical sanity: variances >= 0 (== 0 allowed for some components only)
    for (int i = 0; i < 6; i++) ASSERT_GE_(pose.cov(i, i), .0);
    ASSERT_GT_(pose.cov.trace(), .0);

    // fuse_pose() is the exclusive path for the primary localization source
    // (LiDAR ICP). Wheel-odometry CObservationRobotPose observations are
    // routed to fuse_odometry_3d_pose() in onNewObservation() instead, so
    // they never arrive here and cannot corrupt last_pose_obs_tim.

    // Per-source bookkeeping for this localization source.
    // We use src.last_pose rather than last_pose for the incrPose calculation
    // so that the derived twist reflects true ICP-to-ICP motion even when
    // fuse_odometry() / fuse_odometry_3d_pose() have modified last_pose in
    // between ICP scans.
    auto& src = state_.per_source[frame_id];

    double dt = 0;
    if (src.last_obs_tim)
    {
        dt = mrpt::system::timeDifference(*src.last_obs_tim, timestamp);
    }

    if (dt < 0)
    {
        MRPT_LOG_THROTTLE_WARN_STREAM(
            5.0, "Ignoring fuse_pose() call with backwards timestamp: dt=" << dt << " frame_id="
                                                                           << frame_id);
        src.last_obs_tim = timestamp;
        src.last_pose    = pose;
        return;
    }

    MRPT_LOG_DEBUG_STREAM("fuse_pose(): dt=" << dt << " pose=" << pose.mean);
    if (state_.last_twist)
    {
        MRPT_LOG_DEBUG_STREAM("fuse_pose(): twist before=" << state_.last_twist->asString());
    }

    if (src.last_pose.has_value() && dt > 0 && dt < params.max_time_to_use_velocity_model)
    {
        // Velocity from consecutive ICP poses, uncontaminated by odometry
        // updates to the shared last_pose between scans:
        auto&        tw       = state_.last_twist.emplace();
        const auto   incrPose = pose.mean - src.last_pose->mean;
        const auto   logRot   = mrpt::poses::Lie::SO<3>::log(incrPose.getRotationMatrix());
        const double dt2      = dt * dt;

        tw.vx = incrPose.x() / dt;
        tw.vy = incrPose.y() / dt;
        tw.vz = incrPose.z() / dt;
        tw.wx = logRot[0] / dt;
        tw.wy = logRot[1] / dt;
        tw.wz = logRot[2] / dt;

        auto& twistCov = state_.last_twist_cov.emplace();
        twistCov.setDiagonal(
            {mrpt::square(params.sigma_relative_pose_linear) / dt2,
             mrpt::square(params.sigma_relative_pose_linear) / dt2,
             mrpt::square(params.sigma_relative_pose_linear) / dt2,
             mrpt::square(params.sigma_relative_pose_angular) / dt2,
             mrpt::square(params.sigma_relative_pose_angular) / dt2,
             mrpt::square(params.sigma_relative_pose_angular) / dt2});
    }
    else
    {
        MRPT_LOG_DEBUG_STREAM("fuse_pose(): resetting twist");
        state_.last_twist.reset();
        state_.last_twist_cov.reset();
    }

    if (state_.last_twist)
    {
        MRPT_LOG_DEBUG_STREAM("fuse_pose(): twist after= " << state_.last_twist->asString());
    }
    if (state_.last_twist_cov)
    {
        MRPT_LOG_DEBUG_STREAM(
            "fuse_pose(): twist_cov after=\n"
            << state_.last_twist_cov->asString());
    }

    src.last_pose    = pose;
    src.last_obs_tim = timestamp;

    state_.last_pose                      = pose;
    state_.last_pose_obs_tim              = timestamp;
    state_.pose_already_updated_with_odom = false;
}

namespace
{
void enforce_planar_pose(mrpt::poses::CPose3D& p)
{
    p.z(0);
    p.setYawPitchRoll(p.yaw(), .0, .0);
}

}  // namespace

void StateEstimationSimple::fuse_twist(
    [[maybe_unused]] const mrpt::Clock::time_point& timestamp, const mrpt::math::TTwist3D& twist,
    const mrpt::math::CMatrixDouble66& twistCov)
{
    auto lck = std::scoped_lock(state_mtx_);

    state_.last_twist     = twist;
    state_.last_twist_cov = twistCov;

    MRPT_LOG_DEBUG_STREAM("fuse_twist(): twist    = " << state_.last_twist->asString());
    MRPT_LOG_DEBUG_STREAM("fuse_twist(): twist_cov= " << state_.last_twist_cov->asString());
}

std::optional<NavState> StateEstimationSimple::estimated_navstate(
    const mrpt::Clock::time_point& timestamp, [[maybe_unused]] const std::string& frame_id)
{
    auto lck = std::scoped_lock(state_mtx_);

    if (!state_.last_pose_obs_tim)
    {
        return {};  // None
    }

    const double dt = mrpt::system::timeDifference(*state_.last_pose_obs_tim, timestamp);

    if (!state_.last_twist || !state_.last_pose ||
        std::abs(dt) > params.max_time_to_use_velocity_model)
    {
        return {};  // None
    }

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
        const mola::imu::ImuIntegrationParams rotParams = {};

        const auto rot33 = mola::imu::incremental_rotation({tw.wx, tw.wy, tw.wz}, rotParams, dt);

        poseExtrapolation = mrpt::poses::CPose3D::FromRotationAndTranslation(
            rot33, mrpt::math::TVector3D(tw.vx, tw.vy, tw.vz) * dt);
    }

    // Enforce planar motion?
    if (params.enforce_planar_motion)
    {
        enforce_planar_pose(state_.last_pose->mean);
        enforce_planar_pose(poseExtrapolation);
    }

    // pose mean:
    ret.pose.mean = state_.last_pose->mean + poseExtrapolation;

    // pose cov:
    auto cov = state_.last_pose->cov;

    const double varXYZ = mrpt::square(dt * params.sigma_random_walk_acceleration_linear);
    const double varRot = mrpt::square(dt * params.sigma_random_walk_acceleration_angular);

    for (int i = 0; i < 3; i++)
    {
        cov(i, i) += varXYZ;
    }
    for (int i = 3; i < 6; i++)
    {
        cov(i, i) += varRot;
    }

    // sigma_rel is a position-domain quantity (meters): add it directly as
    // position variance, independent of the fuse_pose/query dt ratio.
    cov(0, 0) += mrpt::square(params.sigma_relative_pose_linear);
    cov(1, 1) += mrpt::square(params.sigma_relative_pose_linear);
    cov(2, 2) += mrpt::square(params.sigma_relative_pose_linear);
    cov(3, 3) += mrpt::square(params.sigma_relative_pose_angular);
    cov(4, 4) += mrpt::square(params.sigma_relative_pose_angular);
    cov(5, 5) += mrpt::square(params.sigma_relative_pose_angular);

    ret.pose.cov_inv = cov.inverse_LLt();

    // twist:
    ret.twist = state_.last_twist.value();

    if (state_.last_twist_cov.has_value())
    {
        ret.twist_inv_cov = state_.last_twist_cov->inverse_LLt();
    }

    return ret;
}

void StateEstimationSimple::onNewObservation(const CObservation::ConstPtr& o)
{
    auto lck = std::scoped_lock(state_mtx_);

    const ProfilerEntry tleg(profiler_, "onNewObservation");

    ASSERT_(o);

    MRPT_LOG_DEBUG_STREAM(
        "onNewObservation(): sensorLabel='" << o->sensorLabel << "' class='"
                                            << o->GetRuntimeClass()->className);

    // IMU:
    if (auto obsIMU = std::dynamic_pointer_cast<const mrpt::obs::CObservationIMU>(o); obsIMU)
    {
        if (std::regex_match(
                o->sensorLabel,
                state_.do_process_imu_labels_re.get_regex(params.do_process_imu_labels_re)))
        {
            this->fuse_imu(*obsIMU);
        }
        else
        {
            MRPT_LOG_DEBUG_FMT(
                "Skipping IMU reading labeled '%s' for not passing regex", o->sensorLabel.c_str());
        }
    }
    // Odometry source:
    else if (auto obsOdom = std::dynamic_pointer_cast<const mrpt::obs::CObservationOdometry>(o);
             obsOdom)
    {
        if (std::regex_match(
                o->sensorLabel, state_.do_process_odometry_labels_re.get_regex(
                                    params.do_process_odometry_labels_re)))
        {
            this->fuse_odometry(*obsOdom, o->sensorLabel);
        }
        else
        {
            MRPT_LOG_DEBUG_FMT(
                "Skipping odometry reading labeled '%s' for not passing regex",
                o->sensorLabel.c_str());
        }
    }
    // Robot pose wrt a reference frame (odometry or map):
    else if (auto obsPose = std::dynamic_pointer_cast<const mrpt::obs::CObservationRobotPose>(o);
             obsPose)
    {
        if (std::regex_match(
                o->sensorLabel, state_.do_process_odometry_labels_re.get_regex(
                                    params.do_process_odometry_labels_re)))
        {
            // Route to the dedicated 3D-odometry path so it never touches
            // last_pose_obs_tim (which belongs to the LiDAR ICP source) and
            // applies the pose as an incremental delta in the SLAM frame rather
            // than replacing last_pose with the absolute odom-frame pose.
            this->fuse_odometry_3d_pose(*obsPose, o->sensorLabel);
        }
        else
        {
            MRPT_LOG_DEBUG_FMT(
                "Skipping robot pose reading labeled '%s' for not passing regex",
                o->sensorLabel.c_str());
        }
    }
    // GNSS source:
    else if (auto obsGPS = std::dynamic_pointer_cast<const mrpt::obs::CObservationGPS>(o); obsGPS)
    {
        if (std::regex_match(
                o->sensorLabel,
                state_.do_process_gnss_labels_re.get_regex(params.do_process_gnss_labels_re)))
        {
            this->fuse_gnss(*obsGPS);
        }
        else
        {
            MRPT_LOG_DEBUG_FMT(
                "Skipping GNSS reading labeled '%s' for not passing regex", o->sensorLabel.c_str());
        }
    }
    else
    {
        MRPT_LOG_THROTTLE_DEBUG_FMT(
            10.0,
            "Do not know how to handle incoming observation label='%s' "
            "class='%s'",
            o->sensorLabel.c_str(), o->GetRuntimeClass()->className);
    }
}

std::optional<mrpt::math::TTwist3D> StateEstimationSimple::get_last_twist() const
{
    auto lck = std::scoped_lock(state_mtx_);

    return state_.last_twist;
}

}  // namespace mola::state_estimation_simple
