/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
 Closed-source licenses available upon request, for this odometry package
 alone or in combination with the complete SLAM system.
*/

/**
 * @file   StateEstimationSmoother.h
 * @brief  Fuse of odometry, IMU, and SE(3) pose/twist estimations.
 * @author Jose Luis Blanco Claraco
 * @date   Jan 22, 2024
 */
#pragma once

// this package:
#include <mola_gtsam_factors/FactorConstVelKinematics.h>
#include <mola_gtsam_factors/FactorTricycleKinematics.h>
#include <mola_kernel/interfaces/LocalizationSourceBase.h>
#include <mola_kernel/interfaces/NavStateFilter.h>
#include <mola_kernel/interfaces/RawDataSourceBase.h>
#include <mola_kernel/version.h>
#include <mola_state_estimation_smoother/Parameters.h>

// MOLA:
#include <mola_imu_preintegration/ImuIntegrator.h>

// MRPT:
#include <mrpt/containers/bimap.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/pimpl.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/CTimeLogger.h>

// std:
#include <mutex>
#include <optional>
#include <set>

namespace mola::state_estimation_smoother
{
/** Sliding window Factor-graph data fusion for odometry, IMU, GNSS, and SE(3)
 * pose/twist estimations.
 *
 * Frame conventions:
 * - There is a frame of reference for each source of odometry, e.g.
 *   there may be one for LiDAR-odometry, another for visual-odometry, or
 *   wheels-based odometry, etc. Each such frame is referenced with a "frame
 *   name" (an arbitrary string).
 *
 * - Internally, this class uses the {utm}, {enu}, and {map} frames. Refer to
 *   the frame diagrams [here](https://docs.mola-slam.org/latest/mola_state_estimators.html).
 *
 * - The name for the reference frame (Default: `"map"`) and the robot/vehicle (`"base_link"`)
 *   can be changed from the parameters (e.g. the config yaml file).
 *
 * - This package DOES NOT follow the [ROS REP 105](https://www.ros.org/reps/rep-0105.html)
 *   specifications in the sense that `/tf` from `{map} → {odom}` are not published.
 *   Instead, it directly emits `{map} → {base_link}` from the fusion of all available data.
 *
 * - Publishing the vehicle pose in a timely manner uses "params.reference_frame_name" as
 *   reference frame.
 *
 * - IMU readings are, by definition, given in the local robot body frame, although
 *   they can have a relative transformation between the vehicle and sensor.
 *
 * Main API methods and frame conventions:
 * - `estimated_navstate()`: Output estimations can be requested in any of the
 *    existing frames of reference.
 * - `fuse_pose()`: Can be used to integrate information from any "odometry" or
 *   "localization" input, as mentioned above.
 * - `fuse_gnss()`: TO-DO.
 * - `fuse_imu()`: TO-DO.
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
 * Old observations are automatically removed.
 *
 * A constant SE(3) velocity model is internally used, without any
 * particular assumptions on the vehicle kinematics.
 *
 * For more theoretical descriptions, see:
 * https://docs.mola-slam.org/latest/mola_state_estimators.html
 *
 * \ingroup mola_state_estimation_grp
 */
class StateEstimationSmoother : public mola::NavStateFilter, public mola::LocalizationSourceBase
{
    DEFINE_MRPT_OBJECT(StateEstimationSmoother, mola::state_estimation_smoother)

   public:
    StateEstimationSmoother();

    /** \name Main API
     *  @{ */

    Parameters params;

    /**
     * @brief Initializes the object and reads all parameters from a YAML node.
     * @param cfg a YAML node with a dictionary of parameters to load from.
     */
    void initialize(const mrpt::containers::yaml& cfg) override;

    void spinOnce() override;

    /** Resets the estimator state to an initial state */
    void reset() override;

    /** Integrates new SE(3) pose odometry estimation of the vehicle wrt frame_id
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

    /** Integrates new twist estimation (in the odom frame) */
    void fuse_twist(
        const mrpt::Clock::time_point& timestamp, const mrpt::math::TTwist3D& twist,
        const mrpt::math::CMatrixDouble66& twistCov) override;

    /** Computes the estimated vehicle state at a given timestep using the
     * observations in the time window. A std::nullopt is returned if there is
     * no valid observations yet, or if requested a timestamp out of the model
     * validity time window (e.g. too far in the future to be trustful).
     */
    [[nodiscard]] std::optional<NavState> estimated_navstate(
        const mrpt::Clock::time_point& timestamp, const std::string& frame_id) override;

    /// Returns a list of known odometry frame_ids:
    auto known_odometry_frame_ids() -> std::set<std::string>;

    /** @} */

   protected:
    // Implementation of RawDataConsumer
#if MOLA_VERSION_CHECK(2, 1, 0)
    void onNewObservation(const CObservation::ConstPtr& o) override;
#else
    void onNewObservation(const CObservation::Ptr& o) override;
#endif

   private:
    // everything related to gtsam is hidden in the public API via pimpl
    // to reduce compilation dependencies, and build time and memory usage.
    struct GtsamImpl;

    using odometry_frameid_t = uint8_t;
    using frame_index_t      = uint32_t;

    struct KinematicState
    {
        mrpt::poses::CPose3D pose;  //!< in the reference frame
        mrpt::math::TTwist3D twist;  //!< in the local frame of reference
    };

    // Accesses to this struct values in state_ must be protected by stateMutex_
    struct State
    {
        State();

        mrpt::pimpl<GtsamImpl> impl;

        /// The next numeric ID to assign to a new frame, for usage in GTSAM symbols P(i), v(i)...
        frame_index_t next_frame_index = 0;

        /// A bimap of timestamps <=> frame indices. Updated by
        mrpt::containers::bimap<mrpt::Clock::time_point, frame_index_t> stamp2frame_index;

        /// A bimap of known odometry "frame_id" <=> "numeric IDs":
        mrpt::containers::bimap<std::string, odometry_frameid_t> known_odom_frames;

        /** For real-time mode operation (not offline): returns the current extrapolated stamp,
         *  by adding the difference between the last observation wallclock time and now to the
         *  last observation timestamp.
         */
        std::optional<mrpt::Clock::time_point> get_current_extrapolated_stamp() const
        {
            if (!last_observation_stamp)
            {
                return {};
            }
            return mrpt::Clock::fromDouble(
                (mrpt::Clock::nowDouble() -
                 mrpt::Clock::toDouble(last_observation_wallclock_stamp)) +
                mrpt::Clock::toDouble(*last_observation_stamp));
        }

        std::optional<mrpt::Clock::time_point> last_observation_stamp;
        mrpt::Clock::time_point                last_observation_wallclock_stamp;
    };

    State                state_;
    std::recursive_mutex stateMutex_;

    /// Creates a new frame index for timestamp t, or returns the existing one if close enough.
    [[nodiscard]] frame_index_t add_or_get_timestamp_frame_index(const mrpt::Clock::time_point& t);

    /// Creates or returns the existing ID, for an odometry frame_id:
    [[nodiscard]] odometry_frameid_t add_or_get_odom_frame_id(const std::string& frame_id_name);

    std::optional<NavState> build_and_optimize_fg(
        const mrpt::Clock::time_point queryTimestamp, const std::string& frame_id);

    /// Implementation of Eqs (1),(4) in the MOLA RSS2019 paper.
    void addFactor(const mola::FactorConstVelKinematics& f);
    void addFactor(const mola::FactorTricycleKinematics& f);

    void delete_too_old_entries();

    mrpt::system::CTimeLogger profiler_{true, "StateEstimationSmoother"};
};

}  // namespace mola::state_estimation_smoother
