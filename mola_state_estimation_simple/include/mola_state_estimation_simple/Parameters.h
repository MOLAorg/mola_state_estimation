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
 * @file   Parameters.h
 * @brief  Parameters for StateEstimationSimple
 * @author Jose Luis Blanco Claraco
 * @date   Jan 22, 2024
 */

#pragma once

#include <mrpt/containers/yaml.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TTwist3D.h>

namespace mola::state_estimation_simple
{
/** Parameters needed by StateEstimationSimple.
 *
 * \ingroup mola_state_estimation_grp
 */
class Parameters
{
   public:
    Parameters() = default;

    /// Loads all parameters from a YAML map node.
    void loadFrom(const mrpt::containers::yaml& cfg);

    /** Valid estimations will be extrapolated only up to this time since the
     * last incorporated observation. */
    double max_time_to_use_velocity_model = 2.0;  // [s]

    mrpt::math::TTwist3D initial_twist;

    double sigma_random_walk_acceleration_linear  = 1.0;  // [m/s²]
    double sigma_random_walk_acceleration_angular = 10.0;  // [rad/s²]

    double sigma_relative_pose_linear  = 1.0;  // [m]
    double sigma_relative_pose_angular = 0.1;  // [rad]

    double sigma_imu_angular_velocity = 0.05;  // [rad/s]

    /** \name Wheel-odometry velocity fusion (fuse_odometry(), CObservationOdometry
     *  readings with hasVelocities=true)
     *  Kept tighter than sigma_relative_pose_linear/angular by default: wheel
     *  encoders give a direct, uncontaminated velocity reading over one control
     *  cycle, unlike a pose difference derived from ICP or another lower-rate
     *  source. Lower further for platforms with known-accurate encoders and no
     *  wheel slip; raise for loose terrain (mud, gravel, vegetation) where slip
     *  makes the reading less trustworthy than these defaults assume.
     *  @{ */
    double sigma_wheel_odom_linear_vel  = 0.1;  // [m/s]
    double sigma_wheel_odom_angular_vel = 0.05;  // [rad/s]
    /** @} */

    /** If true, velocity estimates are smoothed via a per-component scalar
     *  Kalman filter. The process noise reuses
     *  sigma_random_walk_acceleration_linear/angular [m/s^2, rad/s^2], and
     *  measurement noise comes from the covariance already computed by each
     *  fuse_*() call. Default: true.
     *
     *  Enabled by default because raw velocities are obtained by differentiating
     *  consecutive poses (pose increment / dt), which amplifies pose noise; that
     *  velocity is then fed back into the LiDAR-odometry motion model (ICP
     *  initial guess + prior), so without smoothing the loop can oscillate.
     *  Set to false to recover the legacy direct pass-through behavior. */
    bool velocity_filter_enabled = true;

    bool enforce_planar_motion = false;

    /// regex for IMU sensor labels (ROS topics) to accept as IMU readings.
    std::string do_process_imu_labels_re = ".*";

    /// regex for odometry inputs labels (ROS topics) to be accepted as inputs
    std::string do_process_odometry_labels_re = ".*";

    /// regex for GNSS (GPS) labels (ROS topics) to be accepted as inputs
    std::string do_process_gnss_labels_re = ".*";

    /** \name GNSS (GPS/RTK) absolute-position fusion
     *  When enabled AND a geo-reference has been provided (see
     *  StateEstimationSimple::set_geo_reference), low-uncertainty GNSS fixes
     *  (e.g. RTK) nudge the pose anchor toward the absolute map-frame position
     *  to arrest slow drift, without collapsing the ICP prior below what the
     *  motion model needs to cover the next step.
     *  @{ */

    /// Master switch. Default false preserves the legacy "GNSS ignored" behavior.
    bool gnss_enabled = false;

    /** Reject any GNSS fix whose reported horizontal sigma
     *  (sqrt of the larger of the ENU east/north variances) exceeds this [m].
     *  The default 0.20 m cleanly selects RTK-fixed samples and rejects
     *  RTK-float / no-fix (and the UINT32_MAX no-fix covariance sentinel). */
    double gnss_max_horizontal_sigma = 0.20;  // [m]

    /** Lower bound on the horizontal measurement sigma actually used to correct
     *  the anchor [m]. A 2 cm RTK fix is softened to this floor so the prior
     *  Gaussian sent to the ICP still spans the next-step motion at speed. */
    double gnss_min_sigma_floor_xy = 0.10;  // [m]

    /// Same floor for the vertical component [m] (used only if gnss_fuse_z).
    double gnss_min_sigma_floor_z = 0.30;  // [m]

    /** If true, also correct the Z of the anchor. Default false: vehicle-frame
     *  vertical is usually poorly observed and GNSS altitude is noisier. */
    bool gnss_fuse_z = false;

    /** @} */
};

}  // namespace mola::state_estimation_simple
