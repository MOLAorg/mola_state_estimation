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
 * @brief  Parameters for StateEstimationSmoother
 * @author Jose Luis Blanco Claraco
 * @date   Jan 22, 2024
 */

#pragma once

#include <mola_kernel/Georeferencing.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TTwist3D.h>
#include <mrpt/typemeta/TEnumType.h>

namespace mola::state_estimation_smoother
{

enum class KinematicModel : uint8_t
{
    ConstantVelocity,
    Tricycle,
};

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

    /** @name Reference frame IDs
     * @{  */

    /// Used to publish timely pose updates
    std::string vehicle_frame_name = "base_link";

    /// Used to publish timely pose updates. Typically, 'map' or 'odom', etc.
    /// See the docs online.
    std::string reference_frame_name = "map";

    /// The ENU geo-reference frame. See the docs online.
    std::string enu_frame_name = "enu";

    /** If true, `spinOnce()` additionally advertises the REP-105 correction
     * `reference_frame -> map_to_odom_frame_name` (i.e. `map -> odom`), taken
     * directly from the estimator's own `T_map_to_odom_i` graph variable. This
     * is time-consistent and smooth by construction, so the ROS bridge can
     * forward it to `/tf` verbatim instead of composing it from
     * `(map->base_link) * (odom->base_link)^-1` (that composition samples the
     * two factors at mismatched timestamps and injects motion-correlated
     * jitter). The primary `reference_frame -> vehicle_frame` update is still
     * advertised, so the high-rate vehicle-pose topic is unchanged. The extra
     * update carries a distinct `method` suffix (`/map_odom`) so the bridge's
     * TF/odometry source filters can route it independently. Default off. */
    bool publish_map_to_odom_tf = false;

    /** The odometry frame_id (source) whose `T_map_to_odom` is published when
     * `publish_map_to_odom_tf` is true. Empty = auto-select when exactly one
     * odometry source is known (the common single-wheel-odometry case);
     * required if several odometry sources exist. This selects which fused
     * source to read; it is NOT necessarily the ROS `/tf` child frame name (see
     * `map_to_odom_child_frame`). */
    std::string map_to_odom_frame_name;

    /** The `/tf` child frame the `map -> odom` correction is published under
     * when `publish_map_to_odom_tf` is true. Empty = use `map_to_odom_frame_name`
     * (the source's own frame_id). Set this when the fused source's internal
     * frame_id differs from the REP-105 odom `/tf` frame the external wheel
     * driver publishes `odom -> base_link` for: the two must match for the tree
     * `map -> odom -> base_link` to connect, and an odometry source's sensor
     * label (e.g. `odom_wheels`) is often not that ROS frame (`odom`). */
    std::string map_to_odom_child_frame;

    /** If true, `spinOnce()` additionally advertises the fused vehicle pose in a
     * separate child frame `fused_vehicle_frame_name` (default `base_link_fused`)
     * at the module rate. Because it is a distinct child frame it never collides
     * with a `odom -> base_link` chain on `/tf`, so it gives consumers a
     * high-rate fused pose without touching the canonical robot tree (no sensor
     * frames hang off it). Default off. */
    bool publish_fused_vehicle_tf = false;

    /// Child frame name for `publish_fused_vehicle_tf`.
    std::string fused_vehicle_frame_name = "base_link_fused";

    /** @}  */

    /** @name Kinematic factors and keyframe creation (motion model)
     * @{ */

    /** Kinematic model to be used in the internal motion model factors.
     *  Options: `KinematicModel::ConstantVelocity`, `KinematicModel::Tricycle`
     */
    KinematicModel kinematic_model = KinematicModel::ConstantVelocity;

    /** Valid estimations will be extrapolated only up to this time since the
     * last incorporated observation. If a request is done farther away, an
     * empty estimation will be returned.
     */
    double max_time_to_use_velocity_model = 2.0;  // [s]

    /// Time to keep past observations in the filter
    double sliding_window_length = 5.0;  // [s]

    double min_time_difference_to_create_new_frame = 0.01;  // [s]

    /// If the time between two keyframes is larger than this, a warning will be
    /// emitted; but the algorithm will keep trying its best.
    double time_between_frames_to_warning = 3.0;  // [s]

    /** When adding GNSS observations, specially with consumer grade receivers with errors larger
     * than a few centimeters, we may be more permissive in the temporal distance between the GNSS
     * datum and the associated existing keyframe. This parameter is the extended, alternative value
     * to use instead of "min_time_difference_to_create_new_frame". [seconds]
     */
    double gnss_nearby_keyframe_stamp_tolerance = 1.0;  // [s]

    /** When adding IMU observations, this is the temporal distance between the IMU reading
     * and the associated existing keyframe. This applies to gravity-oriented (IMU attitude) and
     * gravity-estimation (accelerometer) only factors, not to high-frequency IMU preintegration.
     *
     * This parameter is the extended, alternative value
     * to use instead of "min_time_difference_to_create_new_frame". [seconds]
     */
    double imu_nearby_keyframe_stamp_tolerance = 0.10;  // [s]

    /** High-rate same-sensor decimation for wheel odometry. If > 0, wheel
     * odometry readings arriving less than this many seconds after the last
     * *kept* one are MERGED rather than turned into their own keyframe: the
     * reading is dropped without advancing the pose anchor, so the next kept
     * reading fuses the accumulated pose increment together with its
     * accumulated motion-model covariance. This caps the keyframe/factor rate
     * a high-rate odometry stream imposes on the solver without discarding any
     * motion. 0 disables it (every reading creates/updates a keyframe as
     * before). This is independent of, and coarser than,
     * min_time_difference_to_create_new_frame (which only merges
     * near-simultaneous readings from different sensors). [seconds]
     */
    double odometry_min_sample_period = 0.0;  // [s]

    /** \name Wheel-odometry motion-model noise (fuse_odometry(), MRPT's
     *  mmGaussian proportional model over each odometry increment: noise
     *  grows with the distance traveled / angle turned, rather than being a
     *  fixed per-step sigma). Defaults match MRPT's CActionRobotMovement2D
     *  Gaussian model defaults except for the min_std floors, which are kept
     *  tighter here (a wheel encoder's near-zero-motion noise floor is well
     *  below MRPT's general-purpose default). Lower a1..a4 for platforms
     *  with accurate encoders and no wheel slip; raise for loose terrain.
     *  @{ */
    double odom_motion_model_a1 = 0.01;  // [m/m] noise growth per m traveled
    double odom_motion_model_a2 = 0.05729577951;  // [m/deg] MRPT default: RAD2DEG(0.001)
    double odom_motion_model_a3 =
        0.017453292519943295;  // [deg/m] MRPT default: DEG2RAD(1.0), noise growth per m traveled
    double odom_motion_model_a4              = 0.05;  // [deg/deg] noise growth per deg rotated
    double odom_motion_model_min_std_xy      = 1e-3;  // [m]
    double odom_motion_model_min_std_phi_deg = 0.1;  // [deg]
    /** @} */

    /** High-rate same-sensor decimation for IMU. If > 0, IMU readings arriving
     * less than this many seconds after the last *processed* one are skipped.
     * Unlike wheel odometry, IMU attitude/gravity are absolute observations, so
     * dropping intermediate readings simply lowers the redundant-factor rate;
     * there is nothing to accumulate. 0 disables it. [seconds]
     */
    double imu_min_sample_period = 0.0;  // [s]

    double sigma_random_walk_acceleration_linear = 0.5;  // [m/s²]
    /** Angular random-walk sigma for the constant-velocity factor between keyframes.
     *  Kept loose on purpose: a tight value makes that factor override the per-keyframe
     *  gyro prior, averaging genuine fast rotations away (the optimized W then keeps only
     *  a fraction of the measured rate). The downstream predict-twist low-pass damps the
     *  extra boundary-node variability this allows. */
    double sigma_random_walk_acceleration_angular = 10.0;  // [rad/s²]

    /** If true, the short-term extrapolation velocity used by
     * estimated_navstate() is a low-pass-filtered version of the newest
     * keyframe's optimized twist, instead of the raw value. The newest keyframe
     * is the boundary node of the sliding window (constrained on one side only),
     * so its raw velocity is the noisiest state in the graph; extrapolating it
     * un-damped injects jitter into a front end's motion prior. This mirrors the
     * velocity low-pass of the lightweight estimator. On by default so the
     * prior stays smooth; it is a plain dt-aware EMA, hence deterministic and
     * reproducible run-to-run. */
    bool predict_twist_filter_enabled = true;

    /** [s] Time constant of the predict-twist low-pass
     * (predict_twist_filter_enabled). Larger = smoother prior but more lag
     * behind genuine acceleration. */
    double predict_twist_filter_time_const = 0.3;
    double sigma_integrator_position       = 0.10;  // [m]
    double sigma_integrator_orientation    = 0.10;  // [rad]

    double sigma_twist_from_consecutive_poses_linear  = 1.0;  // [m/s]
    double sigma_twist_from_consecutive_poses_angular = 1.0;  // [rad/s]

    mrpt::math::TTwist3D initial_twist;

    // Defaults: somewhat confident that the vehicle is near rest.
    // Change these if needed to start with the vehicle at high speed.
    double initial_twist_sigma_lin = 0.1;  // [m/s]
    double initial_twist_sigma_ang = 0.1;  // [rad/s]

    bool enforce_planar_motion = false;

    /** If set, the first ever frame will also have an SE(3) edge favoring it to be the identity in
     * the "reference_frame", with a sigma given by this value. Use a small number, like 1e-6, for
     * initialing the first odometry pose near the map origin. Do not set when using geo-referenced
     * maps.
     */
    std::optional<double> link_first_pose_to_reference_origin_sigma;

    /** @} */

    /** @name IMU related
     * @{  */

    /** When an IMU provides global attitude measurements (azimuth and gravity aligned), this is the
     * uncertainty or noise sigma [degrees]. */
    double imu_attitude_sigma_deg = 2.0;

    /** When an IMU provides global attitude measurements (azimuth and gravity aligned), this must
     * define the angle (in degrees) to add to IMU yaw orientation to obtain azimuth so 0 deg is
     * North. Note that ENU axes are such vehicle yaw is 0 when pointing East instead.
     * Example cases:
     * - IMU absolute yaw=0 points True North ==> offset=0
     * - IMU absolute yaw=0 points East ==> offset=-90
     */
    double imu_attitude_azimuth_offset_deg = 0.0;

    /** When using an IMU with acceleration, use this sigma to estimate the up-vector, hence
     * gravity-align the map.
     * Set to 0 to disable.
     */
    double imu_normalized_gravity_alignment_sigma = 0.4;

    /** When an IMU provides angular velocity (gyroscope), add a direct prior on the
     * corresponding keyframe's body-frame angular-velocity variable, sensor-to-vehicle
     * rotated. Without this, angular velocity is only constrained by the constant-velocity
     * kinematic factor between keyframes plus whatever pose factors happen to be fused, so a
     * genuine, fast rotation can go unrepresented in the graph for as long as those lag (e.g.
     * while ICP is failing and not fusing new poses at all) -- which then also lags the
     * short-term prediction consumed by front ends as their next ICP prior/initial guess.
     * Sigma is in [rad/s]; set to 0 to disable.
     *
     * The default (0.10 rad/s ~ 5.7 deg/s) is deliberately loose: this is one raw,
     * instantaneous sample taken as the average angular velocity over a whole keyframe
     * interval, on a platform that vibrates, so its honest uncertainty is several deg/s. A
     * tighter value pins each keyframe's W hard to its own noisy sample; two such priors on
     * near-simultaneous keyframes then disagree by more than the constant-velocity factor
     * between them allows, and the only way the optimizer can relieve that is by rotating the
     * poses, which can make the window numerically singular. Tune tighter only for a
     * genuinely well-characterized, well-isolated gyro.
     */
    double imu_angular_velocity_sigma = 0.10;

    /** @} */

    /** @name Geo-referencing
     * @{  */

    /** If `true`, this estimator will try to estimate the best geo-referencing for {enu} ->
     * {map} from incoming GNSS readings and other sensors. If `false`, geo-referencing is
     * assumed to be given from either these initial parameters or, if not set, from an external
     * source (e.g. a geo-referenced `.mm` map loaded in mola_lidar_odometry).
     */
    bool estimate_geo_reference = false;

    /** If estimate_geo_reference is `false` and this is set, the geo-referencing will be taken
     * from this value and never attempted to be optimized or changed.
     * Other geo-reference information coming from external sources may override this fixed initial
     * value, though.
     */
    std::optional<mola::Georeferencing> fixed_geo_reference;

    /** Maximum position sigma (in meters) to consider state estimation converged.
     *  Used when other modules query `has_converged_localization()`.
     *  This applies to both, the `map->base_link` and `enu->map` poses.
     */
    double convergence_max_position_sigma = 1.0;  // [m]

    /** Maximum orientation sigma (in degrees) to consider state estimation converged.
     *  Used when other modules query `has_converged_localization()`.
     *  This applies to both, the `map->base_link` and `enu->map` poses.
     */
    double convergence_max_orientation_sigma_deg = 5.0;  // [deg]

    /** If true and estimate_geo_reference is true, once converged, this module
     *  will publish the estimated geo-referencing via MapSourceBase.
     */
    bool publish_estimated_georef_on_convergence = true;

    /** Huber robust cost threshold for GNSS factors [sigmas].
     *  Applied in whitened (normalized) residual space, so units are
     *  standard deviations, not meters. Suitable values: ~1.5 (switch to
     *  linear loss beyond 1.5σ). Set to 0 to disable robust cost (plain
     *  Gaussian).
     */
    double gnss_huber_threshold = 1.5;  // [sigmas]

    /** @} */

    /** @name Nonlinear optimization
     * @{ */

    /** Each new sensor will become a call to isam2.update(), plus this number of additional
     * refining steps. In theory, more steps lead to more accurate results. */
    uint32_t additional_isam2_update_steps = 3;

    /** @} */

    /** @name Real-time async backend
     * @{ */

    /** If `true`, the iSAM2 window solve runs in a dedicated backend thread and
     * `estimated_navstate()` is served by a lock-free FastPredictor re-anchored
     * on the latest backend solution, so a query never runs a solve nor blocks
     * on `stateMutex_` (per-query latency stays sub-millisecond). If `false`
     * (default) the solve runs inline on the caller's thread, which is
     * deterministic and is what the unit tests and offline batch runs use.
     */
    bool async_backend = false;

    /** [s] Only used when `async_backend` is `true`: how far back the
     * FastPredictor keeps high-rate observations to extrapolate from the anchor.
     */
    double fast_predictor_buffer_length = 1.0;  // [s]

    /** @} */

    /** @name Sensor input names
     * @{  */

    /// regex for IMU sensor labels (ROS topics) to accept as IMU readings.
    std::string do_process_imu_labels_re = ".*";

    /// regex for odometry inputs labels (ROS topics) to be accepted as inputs
    std::string do_process_odometry_labels_re = ".*";

    /// regex for GNSS (GPS) labels (ROS topics) to be accepted as inputs
    std::string do_process_gnss_labels_re = ".*";

    /** @} */

    struct Visualization
    {
        bool show_console_messages = true;

        // this is automatically called by parent's loadFrom()
        void loadFrom(const mrpt::containers::yaml& cfg);
    };
    Visualization visualization;
};

}  // namespace mola::state_estimation_smoother

MRPT_ENUM_TYPE_BEGIN_NAMESPACE(
    mola::state_estimation_smoother, mola::state_estimation_smoother::KinematicModel)
MRPT_FILL_ENUM(KinematicModel::ConstantVelocity);
MRPT_FILL_ENUM(KinematicModel::Tricycle);
MRPT_ENUM_TYPE_END()
