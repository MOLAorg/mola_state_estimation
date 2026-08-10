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
 * @file   StateEstimationSimple.h
 * @brief  Fuse of odometry, IMU, and SE(3) pose/twist estimations.
 * @author Jose Luis Blanco Claraco
 * @date   Jan 22, 2024
 */
#pragma once

// This package
#include <mola_kernel/utils/RegexCache.h>
#include <mola_state_estimation_simple/Parameters.h>

// MOLA
#include <mola_kernel/Georeferencing.h>
#include <mola_kernel/interfaces/NavStateFilter.h>
#include <mola_kernel/version.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/optional_ref.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>

#include <array>
#include <mutex>
#include <optional>

namespace mola::state_estimation_simple
{
/** Simple motion-model state estimator fusing odometry, IMU, and SE(3) pose/twist.
 *
 * Usage:
 * - (1) Call initialize() or set the required parameters directly in params_.
 * - (2) Integrate measurements with `fuse_*()` methods. Each CObservation
 *       class includes a `timestamp` field which is used to estimate the
 *       trajectory.
 * - (3) Repeat (2) as needed.
 * - (4) Read the estimation up to any nearby moment in time with
 *       estimated_navstate()
 *
 * ## Prior covariance model (estimated_navstate)
 *
 * Given `dt` seconds elapsed since the last `fuse_pose()` call, the returned
 * prior pose covariance diagonal is:
 *
 *   cov_xyz = sigma_relative_pose_linear^2
 *           + (sigma_random_walk_acceleration_linear * dt)^2
 *
 *   cov_rot = sigma_relative_pose_angular^2
 *           + (sigma_random_walk_acceleration_angular * dt)^2
 *
 * `sigma_relative_pose_linear` [m] is a dt-independent floor on position
 * uncertainty and is the primary knob for tightening the ICP prior.
 * `sigma_random_walk_acceleration_linear` [m/s^2] adds time-growing
 * uncertainty due to unmodeled accelerations.
 *
 * \note This implementation of mola::NavStateFilter ignores the passed
 *       "frame_id" and GNSS observations.
 *
 * \ingroup mola_state_estimation_grp
 */
class StateEstimationSimple : public mola::NavStateFilter

{
    DEFINE_MRPT_OBJECT(StateEstimationSimple, mola::state_estimation_simple)

   public:
    StateEstimationSimple();

    /** \name Main API
     *  @{ */

    Parameters params;

    /**
     * @brief Initializes the object and reads all parameters from a YAML node.
     * @param cfg a YAML node with a dictionary of parameters to load from.
     */
    void initialize(const mrpt::containers::yaml& cfg) override;

    void spinOnce() override;

    /** Resets the estimator state to an initial state.
     *  \sa currentIntegrationState
     */
    void reset() override;

    /** Integrates new SE(3) pose estimation of the vehicle wrt frame_id
     */
    void fuse_pose(
        const mrpt::Clock::time_point& timestamp, const mrpt::poses::CPose3DPDFGaussian& pose,
        const std::string& frame_id) override;

    /** Integrates new wheels-based odometry observations into the estimator.
     *  This is a convenience method that internally ends up calling
     *  fuse_pose(), but computing the uncertainty of odometry increments
     *  according to a given motion model.
     */
    void fuse_odometry(
        const mrpt::obs::CObservationOdometry& odom,
        const std::string&                     odomName = "odom_wheels") override;

    /** Integrates new IMU observations into the estimator */
    void fuse_imu(const mrpt::obs::CObservationIMU& imu) override;

    /** Integrates new GNSS observations into the estimator */
    void fuse_gnss(const mrpt::obs::CObservationGPS& gps) override;

#if defined(MOLA_KERNEL_NAVSTATE_FILTER_HAS_GEO_REFERENCE)
    /** Provides the fixed geo-reference (geodetic datum + ENU->map transform)
     *  used by fuse_gnss() to convert GNSS fixes into map-frame corrections.
     *  In map-based localization this comes from the loaded `.mm` map.
     *  Without it (and without params.gnss_enabled) GNSS is ignored. */
    void set_geo_reference(const mola::Georeferencing& georef) override;

    /** Returns the currently set geo-reference, if any. */
    std::optional<mola::Georeferencing> get_geo_reference() const override;
#endif

#if defined(MOLA_KERNEL_NAVSTATE_FILTER_HAS_TRANSFORM_FRAME)
    /** Re-expresses the stored poses in a new reference frame (`p` -> `b + p`).
     *  Every velocity held here is in the vehicle's own frame, so a change of
     *  the map frame leaves the twist and its covariance untouched. */
    bool transform_frame(const mrpt::poses::CPose3D& b) override;
#endif

    /** Integrates new twist estimation (in the odom frame) */
    void fuse_twist(
        const mrpt::Clock::time_point& timestamp, const mrpt::math::TTwist3D& twist,
        const mrpt::math::CMatrixDouble66& twistCov) override;

    /** Computes the estimated vehicle state at a given timestep using the
     * observations in the time window. A std::nullopt is returned if there is
     * no valid observations yet, or if requested a timestamp out of the model
     * validity time window (e.g. too far in the future to be trustful).
     */
    std::optional<NavState> estimated_navstate(
        const mrpt::Clock::time_point& timestamp, const std::string& frame_id) override;

    /** Returns the twist after fusing every IMU reading received so far,
     *  irrespective of its timestamp (see fuse_imu() on why IMU readings are
     *  buffered). Prefer estimated_navstate() whenever the time of interest is
     *  known. */
    std::optional<mrpt::math::TTwist3D> get_last_twist();

    /** @} */

    // Implementation of RawDataConsumer
    void onNewObservation(const CObservation::ConstPtr& o) override;

   private:
    struct State
    {
        State() = default;

        std::optional<mrpt::obs::CObservationOdometry> last_odom_obs;
        std::optional<mrpt::Clock::time_point>         last_pose_obs_tim;
        std::optional<mrpt::poses::CPose3DPDFGaussian> last_pose;
        std::optional<mrpt::math::TTwist3D>            last_twist;
        std::optional<mrpt::math::CMatrixDouble66>     last_twist_cov;
        bool                                           pose_already_updated_with_odom = false;

        // Per-component variance for the velocity Kalman filter.
        // Indices 0-2: linear (vx,vy,vz), 3-5: angular (wx,wy,wz).
        std::array<double, 6> vel_filter_P = {1e4, 1e4, 1e4, 1e4, 1e4, 1e4};

        // Per-component last-update time. Each velocity component keeps its own
        // clock so sources with very different rates and timestamp conventions
        // do not starve each other: a source only advances the clock of the
        // components it actually observes (finite measurement noise). This stops
        // the high-rate IMU (angular) from rejecting the lower-rate, mid-scan
        // (hence "in the past") LiDAR pose updates (linear) as backwards-in-time.
        std::array<std::optional<mrpt::Clock::time_point>, 6> vel_filter_last_tim;

        // Per-source bookkeeping used by fuse_pose() to compute velocity from
        // consecutive poses of the SAME source (LiDAR ICP), independently of
        // whether odometry has since modified last_pose. Without this, fuse_pose()
        // would compute incrPose = ICP_result - (ICP_prev + odom_accumulated),
        // i.e. the odometry residual, rather than the true robot velocity.
        //
        // Also used by fuse_odometry_3d_pose() for 3D odometry deltas.
        struct SourceState
        {
            std::optional<mrpt::poses::CPose3DPDFGaussian> last_pose;
            std::optional<mrpt::Clock::time_point>         last_obs_tim;

            // Which frame `last_pose` is expressed in. Localization sources
            // (fuse_pose) are in the map frame; 3D odometry sources keep their
            // own, potentially offset, odometry frame. A change of the map
            // frame must only be applied to the former: the latter keeps
            // receiving observations in its original frame, and rebasing the
            // stored anchor would corrupt the next delta.
            bool in_map_frame = true;
        };
        std::map<std::string, SourceState> per_source;

        // Angular velocity (already rotated into the vehicle frame) of an IMU
        // reading that has been received but not fused yet. See fuse_imu().
        struct PendingImu
        {
            double wx = 0;
            double wy = 0;
            double wz = 0;
        };

        // IMU readings waiting to be fused, ordered by timestamp. A multimap,
        // so that two readings sharing a timestamp are both kept: which of them
        // is "the" reading for that instant is undecidable, and dropping one
        // would silently discard data that used to be fused.
        std::multimap<mrpt::Clock::time_point, PendingImu> pending_imu;

        // To be built from parameters strings when changed.
        RegexCache do_process_imu_labels_re;
        RegexCache do_process_odometry_labels_re;
        RegexCache do_process_gnss_labels_re;
    };

    // Integrates a CObservationRobotPose that comes from an odometry source
    // (e.g. wheel encoders forwarded as 3D pose). Unlike fuse_pose(), this
    // applies an incremental delta to last_pose (keeping it in the LiDAR SLAM
    // frame) and does NOT update last_pose_obs_tim, so it never interferes with
    // the LiDAR ICP timestamp used for dt validation and pose extrapolation.
    void fuse_odometry_3d_pose(
        const mrpt::obs::CObservationRobotPose& obs, const std::string& odomName);

    /** Applies a scalar Kalman predict+update step to each velocity component.
     *  When velocity_filter_enabled is false this is a plain write-through.
     *  Must be called with state_mtx_ already held. */
    void update_vel_filter(
        const std::array<double, 6>& z, const std::array<double, 6>& R_diag,
        const mrpt::Clock::time_point& tim, const std::string& caller = "");

    /** Fuses the buffered IMU readings with a timestamp not newer than `upTo`,
     *  in timestamp order, and drops them from the buffer.
     *  Must be called with state_mtx_ already held. */
    void fuse_pending_imu_up_to(const mrpt::Clock::time_point& upTo);

    /** Fuses every buffered IMU reading, whatever its timestamp.
     *  Must be called with state_mtx_ already held. */
    void fuse_all_pending_imu();

    State state_;

    // Fixed geo-reference (datum + ENU->map) for GNSS fusion. Config-like:
    // set once via set_geo_reference() and deliberately NOT cleared by reset(),
    // so relocalization keeps the map's geo-reference.
    std::optional<mola::Georeferencing> geo_reference_;

    mutable std::recursive_mutex state_mtx_;
};

}  // namespace mola::state_estimation_simple
