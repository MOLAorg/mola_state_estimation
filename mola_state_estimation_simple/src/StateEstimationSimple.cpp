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
#include <mrpt/core/get_env.h>
#include <mrpt/obs/CObservationRobotPose.h>
#include <mrpt/obs/gnss_messages.h>
#include <mrpt/poses/Lie/SO.h>
#include <mrpt/topography/conversions.h>

#include <Eigen/Dense>
#include <chrono>
#include <fstream>
#include <memory>

// arguments: class_name, parent_class, class namespace
IMPLEMENTS_MRPT_OBJECT(StateEstimationSimple, mola::ExecutableBase, mola::state_estimation_simple)

namespace mola::state_estimation_simple
{

namespace
{
/// Age [s], relative to the newest buffered reading, beyond which an unfused
/// IMU reading is discarded. Only reached if nothing ever asks the estimator
/// for a state, since any query or measurement drains everything older than
/// its own timestamp.
constexpr double kPendingImuMaxAge = 1.0;
}  // namespace

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

    seedInitialTwistFromParams();

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

    // Buffered-but-unfused IMU readings survive the reset. They are sensor data
    // for instants the filter has not reached yet, not part of the past this
    // call exists to forget, and the next query consumes exactly the ones its
    // own timestamp covers. Discarding them here would instead make the
    // estimate depend on how many readings happened to be delivered before the
    // reset ran, i.e. on thread scheduling rather than on the input.
    auto pendingImu = std::move(state_.pending_imu);

    // reset:
    state_ = State();

    state_.pending_imu = std::move(pendingImu);

    seedInitialTwistFromParams();

    MRPT_LOG_INFO_STREAM("reset() called");
}

void StateEstimationSimple::seedInitialTwistFromParams()
{
    if (params.initial_twist == mrpt::math::TTwist3D())
    {
        return;  // nothing configured, keep the legacy "unknown velocity" state
    }

    state_.last_twist = params.initial_twist;

    auto&        cov     = state_.last_twist_cov.emplace();
    const double var_lin = mrpt::square(params.initial_twist_sigma_lin);
    const double var_ang = mrpt::square(params.initial_twist_sigma_ang);
    cov.setZero();
    for (int i = 0; i < 3; i++)
    {
        cov(i, i) = var_lin;
    }
    for (int i = 3; i < 6; i++)
    {
        cov(i, i) = var_ang;
    }

    state_.vel_filter_P = {var_lin, var_lin, var_lin, var_ang, var_ang, var_ang};
}

namespace
{
// Debug instrumentation: dumps every velocity-filter call (input measurement,
// filtered output, covariances, dt) to a CSV for offline analysis of the
// pose-increment -> velocity -> deskew/prior feedback loop.
// Controlled by env var MOLA_VEL_FILTER_DUMP (path to CSV). Disabled if unset.
std::ofstream* vel_filter_dump_stream()
{
    static std::unique_ptr<std::ofstream> s_stream = []() -> std::unique_ptr<std::ofstream>
    {
        const std::string path = mrpt::get_env<std::string>("MOLA_VEL_FILTER_DUMP");
        if (path.empty())
        {
            return nullptr;
        }
        auto st = std::make_unique<std::ofstream>(path, std::ios::out | std::ios::trunc);
        if (!st->is_open())
        {
            return nullptr;
        }
        // Header:
        (*st) << "tim,caller,dt,filter_enabled,"
                 "z_vx,z_vy,z_vz,z_wx,z_wy,z_wz,"
                 "R_vx,R_vy,R_vz,R_wx,R_wy,R_wz,"
                 "out_vx,out_vy,out_vz,out_wx,out_wy,out_wz,"
                 "P_vx,P_vy,P_vz,P_wx,P_wy,P_wz\n";
        return st;
    }();
    return s_stream.get();
}

// Debug instrumentation: dumps every estimated_navstate() result (the pose +
// covariance that LIO uses as ICP initial guess AND prior, plus the returned
// twist) so the prior-vs-data weighting and the velocity feedback can be
// analyzed offline. Controlled by env var MOLA_NAVSTATE_DUMP. Disabled if unset.
std::ofstream* navstate_dump_stream()
{
    static std::unique_ptr<std::ofstream> s_stream = []() -> std::unique_ptr<std::ofstream>
    {
        const std::string path = mrpt::get_env<std::string>("MOLA_NAVSTATE_DUMP");
        if (path.empty())
        {
            return nullptr;
        }
        auto st = std::make_unique<std::ofstream>(path, std::ios::out | std::ios::trunc);
        if (!st->is_open())
        {
            return nullptr;
        }
        (*st) << "tim,dt,"
                 "x,y,z,yaw,pitch,roll,"
                 "tw_vx,tw_vy,tw_vz,tw_wx,tw_wy,tw_wz,"
                 "cov_x,cov_y,cov_z,cov_yaw,cov_pitch,cov_roll,"
                 "covinv_x,covinv_y,covinv_z,covinv_yaw,covinv_pitch,covinv_roll\n";
        return st;
    }();
    return s_stream.get();
}
}  // namespace

void StateEstimationSimple::update_vel_filter(
    const std::array<double, 6>& z, const std::array<double, 6>& R_diag,
    const mrpt::Clock::time_point& tim, const std::string& caller)
{
    // For instrumentation only: dt since the previous filtered sample, taken
    // from the first component this call actually observes (finite R).
    double dbg_dt = 0.0;
    for (int i = 0; i < 6; i++)
    {
        if (R_diag[i] < 1e8 && state_.vel_filter_last_tim[i].has_value())
        {
            dbg_dt = mrpt::system::timeDifference(*state_.vel_filter_last_tim[i], tim);
            break;
        }
    }

    auto write_twist_and_cov = [&](const std::array<double, 6>& v, const std::array<double, 6>& P)
    {
        auto& tw = state_.last_twist.emplace();
        tw.vx    = v[0];
        tw.vy    = v[1];
        tw.vz    = v[2];
        tw.wx    = v[3];
        tw.wy    = v[4];
        tw.wz    = v[5];

        auto& cov = state_.last_twist_cov.emplace();
        cov.setZero();
        for (int i = 0; i < 6; i++)
        {
            cov(i, i) = P[i];
        }

        // Debug instrumentation (no-op unless MOLA_VEL_FILTER_DUMP is set):
        if (std::ofstream* st = vel_filter_dump_stream(); st)
        {
            (*st) << mrpt::format("%.6f", mrpt::Clock::toDouble(tim)) << "," << caller << ","
                  << dbg_dt << "," << (params.velocity_filter_enabled ? 1 : 0);
            for (int i = 0; i < 6; i++)
            {
                (*st) << "," << z[i];
            }
            for (int i = 0; i < 6; i++)
            {
                (*st) << "," << R_diag[i];
            }
            for (int i = 0; i < 6; i++)
            {
                (*st) << "," << v[i];
            }
            for (int i = 0; i < 6; i++)
            {
                (*st) << "," << P[i];
            }
            (*st) << "\n";
            st->flush();
        }
    };

    if (!params.velocity_filter_enabled)
    {
        // Write-through: no filtering, behaves identically to the pre-filter code.
        write_twist_and_cov(z, R_diag);
        return;
    }

    // Process noise: velocity random walk, one sigma per component [units/s].
    const std::array<double, 6> sigma_q = {
        params.sigma_random_walk_acceleration_linear,
        params.sigma_random_walk_acceleration_linear,
        params.sigma_random_walk_acceleration_linear,
        params.sigma_random_walk_acceleration_angular,
        params.sigma_random_walk_acceleration_angular,
        params.sigma_random_walk_acceleration_angular,
    };

    constexpr double kNoInfo = 1e8;  // R threshold for "unobserved" components
    constexpr double kEps    = 1e-12;

    // Start from the current filtered estimate; components this call does not
    // observe are carried over unchanged (value, covariance, and their clock).
    std::array<double, 6> v = {0, 0, 0, 0, 0, 0};
    if (state_.last_twist.has_value())
    {
        const auto& cur_tw = *state_.last_twist;
        v                  = {cur_tw.vx, cur_tw.vy, cur_tw.vz, cur_tw.wx, cur_tw.wy, cur_tw.wz};
    }

    // Each component runs its own scalar Kalman filter on its own clock, so a
    // high-rate source (e.g. IMU angular) cannot starve a lower-rate, mid-scan
    // ("in the past") source (e.g. LiDAR pose linear): a component is only
    // touched by sources that actually observe it (finite R), and a sample that
    // is older than that component's last update is ignored for that component
    // alone (the fresher one wins) instead of dropping the whole call.
    for (int i = 0; i < 6; i++)
    {
        // Unobserved by this source: leave value, covariance and clock as-is.
        if (R_diag[i] >= kNoInfo)
        {
            continue;
        }

        // Bootstrap this component on its first observation (or after a twist
        // reset cleared the per-component clocks).
        if (!state_.vel_filter_last_tim[i].has_value() || !state_.last_twist.has_value())
        {
            v[i]                          = z[i];
            state_.vel_filter_P[i]        = R_diag[i];
            state_.vel_filter_last_tim[i] = tim;
            continue;
        }

        const double dt = mrpt::system::timeDifference(*state_.vel_filter_last_tim[i], tim);
        if (dt < 0)
        {
            continue;  // out-of-order for THIS component; keep the fresher value
        }

        // Predict: grow uncertainty over this component's own elapsed time.
        state_.vel_filter_P[i] += mrpt::square(sigma_q[i] * dt);

        // Update:
        const double denom = state_.vel_filter_P[i] + R_diag[i];
        if (denom <= kEps)
        {
            continue;  // avoid 0/0 or NaN
        }
        const double K = state_.vel_filter_P[i] / denom;
        v[i] += K * (z[i] - v[i]);
        state_.vel_filter_P[i] *= (1.0 - K);
        state_.vel_filter_last_tim[i] = tim;
    }

    write_twist_and_cov(v, state_.vel_filter_P);
}

void StateEstimationSimple::fuse_odometry(
    const mrpt::obs::CObservationOdometry& odom, [[maybe_unused]] const std::string& odomName)
{
    auto lck = std::scoped_lock(state_mtx_);

    fuse_pending_imu_up_to(odom.timestamp);

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
        // 2D odometry measures vx, vy, wz only. Pass large R for the
        // unmeasured components (vz, wx, wy) so the filter gain is ~0 for them.
        const double no_info = 1e9;
        const double var_xyz = mrpt::square(params.sigma_wheel_odom_linear_vel);  // [m²/s²]
        const double var_rot = mrpt::square(params.sigma_wheel_odom_angular_vel);  // [rad²/s²]

        const double cur_vz = state_.last_twist.has_value() ? state_.last_twist->vz : 0.0;
        const double cur_wx = state_.last_twist.has_value() ? state_.last_twist->wx : 0.0;
        const double cur_wy = state_.last_twist.has_value() ? state_.last_twist->wy : 0.0;

        const std::array<double, 6> z = {
            odom.velocityLocal.vx,    odom.velocityLocal.vy, cur_vz, cur_wx, cur_wy,
            odom.velocityLocal.omega,
        };
        const std::array<double, 6> R_diag = {
            var_xyz, var_xyz, no_info, no_info, no_info, var_rot,
        };

        update_vel_filter(z, R_diag, odom.timestamp, "fuse_odometry");

        MRPT_LOG_DEBUG_STREAM(
            "fuse_odometry: twist from velocityLocal: " << state_.last_twist->asString());
    }

    MRPT_LOG_DEBUG_STREAM("fuse_odometry: odom=" << odom.asString());
}

void StateEstimationSimple::fuse_odometry_3d_pose(
    const mrpt::obs::CObservationRobotPose& obs, const std::string& odomName)
{
    auto lck = std::scoped_lock(state_mtx_);

    fuse_pending_imu_up_to(obs.timestamp);

    // Apply sensor-to-base correction if the sensor is not at the origin:
    auto sensedPose = obs.pose;
    if (obs.sensorPose != mrpt::poses::CPose3D())
    {
        sensedPose = sensedPose + mrpt::poses::CPose3DPDFGaussian(-obs.sensorPose);
    }

    auto& src        = state_.per_source[odomName];
    src.in_map_frame = false;

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
            const auto   logRot  = mrpt::poses::Lie::SO<3>::log(delta.getRotationMatrix());
            const double dt2     = dt * dt;
            const double var_lin = mrpt::square(params.sigma_relative_pose_linear) / dt2;
            const double var_ang = mrpt::square(params.sigma_relative_pose_angular) / dt2;

            const std::array<double, 6> z = {
                delta.x() / dt, delta.y() / dt, delta.z() / dt,
                logRot[0] / dt, logRot[1] / dt, logRot[2] / dt,
            };
            const std::array<double, 6> R_diag = {
                var_lin, var_lin, var_lin, var_ang, var_ang, var_ang,
            };

            update_vel_filter(z, R_diag, obs.timestamp, "fuse_odometry_3d_pose");

            MRPT_LOG_DEBUG_STREAM(
                "fuse_odometry_3d_pose('" << odomName
                                          << "'): twist=" << state_.last_twist->asString());
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

    // Angular velocity, transformed from the IMU frame to the vehicle frame:
    mrpt::math::TTwist3D imuReading;
    imuReading.wx = imu.get(mrpt::obs::TIMUDataIndex::IMU_WX);
    imuReading.wy = imu.get(mrpt::obs::TIMUDataIndex::IMU_WY);
    imuReading.wz = imu.get(mrpt::obs::TIMUDataIndex::IMU_WZ);
    imuReading.rotate(imu.sensorPose.asTPose());

    // The reading is buffered, not fused right away: it is fused once some
    // other call establishes the time of interest (a query or another
    // measurement), and only if it is not newer than that time. The estimate
    // returned for a given time is then a function of the measurement
    // timestamps alone, and no longer of how many IMU readings happened to be
    // delivered first, which is what makes concurrent sensor inputs
    // reproducible.
    state_.pending_imu.emplace(
        imu.timestamp, State::PendingImu{imuReading.wx, imuReading.wy, imuReading.wz});

    // Keep the buffer bounded in case nothing ever asks for an estimate:
    const auto oldestToKeep =
        state_.pending_imu.rbegin()->first - std::chrono::duration_cast<mrpt::Clock::duration>(
                                                 std::chrono::duration<double>(kPendingImuMaxAge));
    state_.pending_imu.erase(
        state_.pending_imu.begin(), state_.pending_imu.lower_bound(oldestToKeep));
}

void StateEstimationSimple::fuse_pending_imu_up_to(const mrpt::Clock::time_point& upTo)
{
    // IMU only observes angular velocity: preserve linear (vx,vy,vz) from the
    // last fuse_pose(). Pass a very large R for the linear components so the
    // filter gain for them is ~0 (no new information from this IMU reading).
    const double no_info = 1e9;
    const double var_ang = mrpt::square(params.sigma_imu_angular_velocity);

    const std::array<double, 6> R_diag = {
        no_info, no_info, no_info, var_ang, var_ang, var_ang,
    };

    const auto itEnd = state_.pending_imu.upper_bound(upTo);
    for (auto it = state_.pending_imu.begin(); it != itEnd; ++it)
    {
        const double cur_vx = state_.last_twist.has_value() ? state_.last_twist->vx : 0.0;
        const double cur_vy = state_.last_twist.has_value() ? state_.last_twist->vy : 0.0;
        const double cur_vz = state_.last_twist.has_value() ? state_.last_twist->vz : 0.0;

        const std::array<double, 6> z = {
            cur_vx, cur_vy, cur_vz, it->second.wx, it->second.wy, it->second.wz,
        };

        update_vel_filter(z, R_diag, it->first, "fuse_imu");

        MRPT_LOG_DEBUG_STREAM("fuse_imu(): new twist: " << state_.last_twist->asString());
    }
    state_.pending_imu.erase(state_.pending_imu.begin(), itEnd);
}

void StateEstimationSimple::fuse_all_pending_imu()
{
    if (state_.pending_imu.empty())
    {
        return;
    }
    fuse_pending_imu_up_to(state_.pending_imu.rbegin()->first);
}

#if defined(MOLA_KERNEL_NAVSTATE_FILTER_HAS_GEO_REFERENCE)
void StateEstimationSimple::set_geo_reference(const mola::Georeferencing& georef)
{
    auto lck       = std::scoped_lock(state_mtx_);
    geo_reference_ = georef;
}

std::optional<mola::Georeferencing> StateEstimationSimple::get_geo_reference() const
{
    auto lck = std::scoped_lock(state_mtx_);
    return geo_reference_;
}
#endif

void StateEstimationSimple::fuse_gnss(const mrpt::obs::CObservationGPS& gps)
{
    auto lck = std::scoped_lock(state_mtx_);

    // GNSS fusion is opt-in and needs a geo-reference to place fixes in the map
    // frame. Without either, ignore (legacy behavior; see the smoother for a
    // full graph-based estimator).
    if (!params.gnss_enabled)
    {
        MRPT_LOG_DEBUG_STREAM("fuse_gnss(): ignored (gnss_enabled=false)");
        return;
    }
    if (!geo_reference_.has_value())
    {
        MRPT_LOG_THROTTLE_WARN(5.0, "fuse_gnss(): ignored, no geo-reference set");
        return;
    }
    // We only correct an existing anchor; the LiDAR/odometry backbone must exist.
    if (!state_.last_pose.has_value())
    {
        MRPT_LOG_DEBUG_STREAM("fuse_gnss(): ignored, no last_pose yet");
        return;
    }

    // Reject fixes that predate the anchor's own timestamp: fusing a delayed
    // GNSS position into an anchor already extrapolated past that time (while
    // leaving last_pose_obs_tim untouched) would apply the correction a second
    // time on the next estimated_navstate() extrapolation.
    if (state_.last_pose_obs_tim.has_value() && gps.timestamp < *state_.last_pose_obs_tim)
    {
        MRPT_LOG_DEBUG_STREAM("fuse_gnss(): ignored, stale GNSS fix older than last_pose_obs_tim");
        return;
    }

    if (!gps.has_GGA_datum())
    {
        MRPT_LOG_DEBUG_STREAM("fuse_gnss(): ignored, no GGA datum");
        return;
    }
    if (!gps.covariance_enu.has_value())
    {
        MRPT_LOG_THROTTLE_WARN(5.0, "fuse_gnss(): ignored, GNSS reading has no ENU covariance");
        return;
    }

    // RTK gate: reject anything but low-uncertainty fixes. The larger of the
    // east/north variances defines the horizontal sigma; this also rejects the
    // UINT32_MAX no-fix covariance sentinel.
    const auto&  cov_enu = *gps.covariance_enu;
    const double var_e   = cov_enu(0, 0);
    const double var_n   = cov_enu(1, 1);
    // A valid position fix must report strictly positive horizontal variances.
    // Zero or negative values are invalid (uninitialized / no-fix / corrupted)
    // and would otherwise fabricate a spuriously confident anchor correction.
    if (!std::isfinite(var_e) || !std::isfinite(var_n) || var_e <= 0 || var_n <= 0)
    {
        MRPT_LOG_THROTTLE_WARN_FMT(
            5.0, "fuse_gnss(): ignored, non-positive ENU variance (var_e=%g, var_n=%g)", var_e,
            var_n);
        return;
    }
    if (params.gnss_fuse_z)
    {
        const double var_u = cov_enu(2, 2);
        if (!std::isfinite(var_u) || var_u <= 0)
        {
            MRPT_LOG_THROTTLE_WARN_FMT(
                5.0, "fuse_gnss(): ignored, non-positive ENU up-variance (var_u=%g)", var_u);
            return;
        }
    }
    const double horiz_var   = std::max(var_e, var_n);
    const double horiz_sigma = std::sqrt(horiz_var);
    if (!std::isfinite(horiz_sigma) || horiz_sigma > params.gnss_max_horizontal_sigma)
    {
        MRPT_LOG_DEBUG_FMT(
            "fuse_gnss(): ignored, horiz_sigma=%.3f m > gate %.3f m", horiz_sigma,
            params.gnss_max_horizontal_sigma);
        return;
    }

    // Geodetic -> ENU (wrt the map datum) -> map frame.
    const auto& gga       = gps.getMsgByClass<mrpt::obs::gnss::Message_NMEA_GGA>();
    const auto  geoCoords = gga.getAsStruct<mrpt::topography::TGeodeticCoords>();

    mrpt::math::TPoint3D enu_point;
    mrpt::topography::geodeticToENU_WGS84(geoCoords, enu_point, geo_reference_->geo_coord);

    // Antenna position in the map frame:
    const mrpt::poses::CPose3D antenna_in_map =
        geo_reference_->T_enu_to_map.mean +
        mrpt::poses::CPose3D(enu_point.x, enu_point.y, enu_point.z, 0, 0, 0);

    // The fix locates the ANTENNA; shift by the antenna lever arm (expressed in
    // the vehicle frame via the current attitude) to obtain the vehicle-frame
    // position the anchor represents.
    mrpt::math::TPoint3D vehicle_in_map = antenna_in_map.translation();
    if (gps.sensorPose != mrpt::poses::CPose3D())
    {
        const auto lever_map = state_.last_pose->mean.rotateVector(gps.sensorPose.translation());
        vehicle_in_map -= lever_map;
    }

    // Full linear Kalman correction of the anchor pose from the GNSS position
    // observation. The state is [x y z yaw pitch roll]; the position rows of
    // the 6x6 covariance may be correlated with orientation (e.g. after an
    // IMU/ICP update), so a per-axis diagonal-only correction would leave
    // those cross terms stale and inconsistent. Twist is untouched (GNSS
    // observes neither). A covariance FLOOR keeps the downstream ICP prior
    // from collapsing below the motion-model needs.
    const double sigma_xy    = std::max(horiz_sigma, params.gnss_min_sigma_floor_xy);
    const double meas_var_xy = mrpt::square(sigma_xy);
    const double meas_var_z  = mrpt::square(
         std::max(std::sqrt(std::max(cov_enu(2, 2), .0)), params.gnss_min_sigma_floor_z));

    auto&     mean   = state_.last_pose->mean;
    auto&     cov    = state_.last_pose->cov;
    const int n_axes = params.gnss_fuse_z ? 3 : 2;

    Eigen::VectorXd innovation(n_axes);
    innovation(0) = vehicle_in_map.x - mean.x();
    innovation(1) = vehicle_in_map.y - mean.y();
    if (n_axes == 3)
    {
        innovation(2) = vehicle_in_map.z - mean.z();
    }

    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(n_axes, n_axes);
    R(0, 0)           = meas_var_xy;
    R(1, 1)           = meas_var_xy;
    if (n_axes == 3)
    {
        R(2, 2) = meas_var_z;
    }

    auto P = cov.asEigen();  // 6x6 Eigen map onto the pose covariance.

    // H selects the first n_axes rows (the observed position components), so
    // H*P is simply the top n_axes rows of P, and P*H^T its left n_axes cols.
    const Eigen::MatrixXd HP = P.topRows(n_axes);
    const Eigen::MatrixXd S  = P.topLeftCorner(n_axes, n_axes) + R;
    if (S.determinant() <= 0)
    {
        MRPT_LOG_DEBUG_STREAM("fuse_gnss(): ignored, non-invertible innovation covariance");
        return;
    }
    const Eigen::MatrixXd K  = P.leftCols(n_axes) * S.inverse();
    const Eigen::VectorXd dx = K * innovation;

    mean.x(mean.x() + dx(0));
    mean.y(mean.y() + dx(1));
    if (n_axes == 3)
    {
        mean.z(mean.z() + dx(2));
    }
    mean.setYawPitchRoll(mean.yaw() + dx(3), mean.pitch() + dx(4), mean.roll() + dx(5));

    P -= K * HP;

    MRPT_LOG_DEBUG_FMT(
        "fuse_gnss(): corrected anchor to map=(%.3f,%.3f,%.3f) horiz_sigma=%.3f m", mean.x(),
        mean.y(), mean.z(), horiz_sigma);
}

void StateEstimationSimple::fuse_pose(
    const mrpt::Clock::time_point& timestamp, const mrpt::poses::CPose3DPDFGaussian& pose,
    const std::string& frame_id)
{
    auto lck = std::scoped_lock(state_mtx_);

    // The IMU readings up to this measurement's own time belong before it in
    // the filter; newer ones stay buffered (see fuse_imu()):
    fuse_pending_imu_up_to(timestamp);

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
    auto& src        = state_.per_source[frame_id];
    src.in_map_frame = true;

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
        const auto   incrPose = pose.mean - src.last_pose->mean;
        const auto   logRot   = mrpt::poses::Lie::SO<3>::log(incrPose.getRotationMatrix());
        const double dt2      = dt * dt;
        const double var_lin  = mrpt::square(params.sigma_relative_pose_linear) / dt2;
        const double var_ang  = mrpt::square(params.sigma_relative_pose_angular) / dt2;

        const std::array<double, 6> z = {
            incrPose.x() / dt, incrPose.y() / dt, incrPose.z() / dt,
            logRot[0] / dt,    logRot[1] / dt,    logRot[2] / dt,
        };
        const std::array<double, 6> R_diag = {
            var_lin, var_lin, var_lin, var_ang, var_ang, var_ang,
        };

        update_vel_filter(z, R_diag, timestamp, "fuse_pose");
    }
    else if (src.last_pose.has_value())
    {
        // dt <= 0 or dt >= max_time_to_use_velocity_model for a source that DID
        // have a prior pose: a genuine reason to distrust whatever twist is
        // currently held. When src.last_pose has no value at all (the very
        // first fuse_pose() call ever for this source), there is simply
        // nothing new to compute here, so any twist already held -- from
        // params.initial_twist, or from a real fuse_twist()/fuse_odometry()/
        // fuse_imu() measurement fused before this source's first pose -- is
        // left untouched instead of being wiped out.
        MRPT_LOG_DEBUG_STREAM("fuse_pose(): resetting twist");
        state_.last_twist.reset();
        state_.last_twist_cov.reset();
        state_.vel_filter_P = State().vel_filter_P;
        for (auto& t : state_.vel_filter_last_tim)
        {
            t.reset();
        }
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

    // Drop a PRE-ANCHOR odometry baseline: fuse_odometry() can be called
    // before the first fuse_pose() ever runs (odometry usually starts
    // streaming immediately, LiDAR ICP needs a scan first), in which case
    // it already skips applying an increment (no last_pose to add it to)
    // but still records state_.last_odom_obs as a baseline. Once this,
    // the FIRST pose, is accepted below, that baseline predates the anchor
    // -- the next fuse_odometry() call would otherwise apply the delta
    // spanning that whole pre-anchor gap onto the fresh pose. Only the
    // first-ever pose needs this: once an odom baseline is established
    // AFTER a pose already exists, subsequent fuse_pose() calls (periodic
    // ICP corrections) must NOT reset it, or the odometry chain never gets
    // to dead-reckon between them -- see test_odometry_fusion(), which
    // fuse_pose()s twice before relying on the odometry increment still
    // being anchored to the first call's baseline.
    if (!state_.last_pose.has_value())
    {
        state_.last_odom_obs.reset();
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
    const mrpt::Clock::time_point& timestamp, const mrpt::math::TTwist3D& twist,
    const mrpt::math::CMatrixDouble66& twistCov)
{
    auto lck = std::scoped_lock(state_mtx_);

    fuse_pending_imu_up_to(timestamp);

    std::array<double, 6> z = {
        twist.vx, twist.vy, twist.vz, twist.wx, twist.wy, twist.wz,
    };
    std::array<double, 6> R_diag;
    for (int i = 0; i < 6; i++)
    {
        R_diag[i] = twistCov(i, i);
    }

    update_vel_filter(z, R_diag, timestamp, "fuse_twist");

    MRPT_LOG_DEBUG_STREAM("fuse_twist(): twist    = " << state_.last_twist->asString());
    MRPT_LOG_DEBUG_STREAM("fuse_twist(): twist_cov= " << state_.last_twist_cov->asString());
}

#if defined(MOLA_KERNEL_NAVSTATE_FILTER_HAS_TRANSFORM_FRAME)
bool StateEstimationSimple::transform_frame(const mrpt::poses::CPose3D& b)
{
    auto lck = std::scoped_lock(state_mtx_);

    // Everything expressed in the map frame: the fused pose and the per-source
    // last poses of map-frame sources, used to derive velocity from
    // consecutive observations.
    if (state_.last_pose)
    {
        state_.last_pose->changeCoordinatesReference(b);
    }
    for (auto& [name, src] : state_.per_source)
    {
        if (!src.in_map_frame || !src.last_pose)
        {
            continue;
        }
        src.last_pose->changeCoordinatesReference(b);
    }

#if defined(MOLA_KERNEL_NAVSTATE_FILTER_HAS_GEO_REFERENCE)
    // The ENU-to-map transform ends in the map frame, so it must follow it.
    // The geodetic datum (geo_coord) is a property of the Earth, not of the
    // map frame, hence left untouched.
    if (geo_reference_)
    {
        geo_reference_->T_enu_to_map.changeCoordinatesReference(b);
    }
#endif

    // Deliberately NOT touched: last_twist / last_twist_cov / vel_filter_P and
    // the buffered IMU readings. All of them live in the vehicle's own frame
    // (fuse_pose() derives velocity from `pose - previous_pose`, and
    // estimated_navstate() extrapolates by right-composition), which a change
    // of the map frame leaves invariant. Wheel-odometry readings are likewise
    // in their own frame, and so are the odometry-frame entries skipped above.

    MRPT_LOG_INFO_STREAM("transform_frame(): applied reference-frame change " << b);

    return true;
}
#endif

std::optional<NavState> StateEstimationSimple::estimated_navstate(
    const mrpt::Clock::time_point& timestamp, [[maybe_unused]] const std::string& frame_id)
{
    auto lck = std::scoped_lock(state_mtx_);

    // Bring the filter up to the queried time, using only the IMU readings that
    // precede it (see fuse_imu()):
    fuse_pending_imu_up_to(timestamp);

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

    // Debug instrumentation (no-op unless MOLA_NAVSTATE_DUMP is set):
    if (std::ofstream* st = navstate_dump_stream(); st)
    {
        const auto& m = ret.pose.mean;
        (*st) << mrpt::format("%.6f", mrpt::Clock::toDouble(timestamp)) << "," << dt << ","  //
              << m.x() << "," << m.y() << "," << m.z() << "," << m.yaw() << "," << m.pitch() << ","
              << m.roll();
        const auto& tw = ret.twist;
        (*st) << "," << tw.vx << "," << tw.vy << "," << tw.vz << "," << tw.wx << "," << tw.wy << ","
              << tw.wz;
        for (int i = 0; i < 6; i++)
        {
            (*st) << "," << cov(i, i);
        }
        for (int i = 0; i < 6; i++)
        {
            (*st) << "," << ret.pose.cov_inv(i, i);
        }
        (*st) << "\n";
        st->flush();
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

std::optional<mrpt::math::TTwist3D> StateEstimationSimple::get_last_twist()
{
    auto lck = std::scoped_lock(state_mtx_);

    // No time of interest is given here, so everything received is fused:
    fuse_all_pending_imu();

    return state_.last_twist;
}

}  // namespace mola::state_estimation_simple
