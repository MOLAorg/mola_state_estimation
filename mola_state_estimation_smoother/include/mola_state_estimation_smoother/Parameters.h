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
 * @file   Parameters.h
 * @brief  Parameters for StateEstimationSmoother
 * @author Jose Luis Blanco Claraco
 * @date   Jan 22, 2024
 */

#pragma once

#include <mrpt/containers/yaml.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TTwist3D.h>

#include <regex>

namespace mola::state_estimation_smoother
{
/** Parameters needed by StateEstimationSmoother.
 *
 * \ingroup mola_navstate_fuse__grp
 */
class Parameters
{
   public:
    Parameters() = default;

    /// Loads all parameters from a YAML map node.
    void loadFrom(const mrpt::containers::yaml& cfg);

    /// Used to publish timely pose updates
    std::string vehicle_frame_name = "base_link";

    /// Used to publish timely pose updates. Typically, 'map' or 'odom', etc.
    /// See the docs online.
    std::string reference_frame_name = "map";

    /** Valid estimations will be extrapolated only up to this time since the
     * last incorporated observation. If a request is done farther away, an
     * empty estimation will be returned.
     */
    double max_time_to_use_velocity_model = 2.0;  // [s]

    /// Time to keep past observations in the filter
    double sliding_window_length = 5.0;  // [s]

    /// If the time between two keyframes is larger than this, a warning will be
    /// emitted; but the algorithm will keep trying its best.
    double time_between_frames_to_warning = 3.0;  // [s]

    double sigma_random_walk_acceleration_linear  = 1.0;  // [m/s²]
    double sigma_random_walk_acceleration_angular = 1.0;  // [rad/s²]
    double sigma_integrator_position              = 0.10;  // [m]
    double sigma_integrator_orientation           = 0.10;  // [rad]

    double sigma_twist_from_consecutive_poses_linear  = 1.0;  // [m/s]
    double sigma_twist_from_consecutive_poses_angular = 1.0;  // [rad/s]

    double robust_param = 0.0;  // 0: no robust
    double max_rmse     = 2.0;

    mrpt::math::TTwist3D initial_twist;
    double               initial_twist_sigma_lin = 20.0;  // [m/s]
    double               initial_twist_sigma_ang = 3.0;  // [rad/s]

    bool enforce_planar_motion = false;

    //!< regex for IMU sensor labels (ROS topics) to accept as IMU readings.
    std::regex do_process_imu_labels_re{".*"};

    //!< regex for odometry inputs labels (ROS topics) to be accepted as inputs
    std::regex do_process_odometry_labels_re{".*"};

    //!< regex for GNSS (GPS) labels (ROS topics) to be accepted as inputs
    std::regex do_process_gnss_labels_re{".*"};
};

}  // namespace mola::state_estimation_smoother
