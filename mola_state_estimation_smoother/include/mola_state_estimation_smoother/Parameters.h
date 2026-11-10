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

/// With a fixed geo-reference, an orientation that only position sources
/// (e.g. GNSS without IMU) observe is held fixed and reported as unknown,
/// instead of leaving the solver indeterminate:
#define MOLA_STATE_ESTIMATION_SMOOTHER_HAS_ORIENTATION_GAUGE 1

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

    /** Other odometry source frame_ids (comma or space separated) whose
     * `T_map_to_odom` is also advertised when `publish_map_to_odom_tf` is true, as
     * `reference_frame -> <frame_id>`, so data in those frames (e.g. the local map
     * of a LiDAR odometry source, in `lidar_odom`) can be transformed to the map
     * frame. Sources not known or not solved yet are skipped. */
    std::string map_to_odom_extra_frames;

    /** If true, `spinOnce()` additionally advertises the fused vehicle pose in a
     * separate child frame `fused_vehicle_frame_name` (default `base_link_fused`)
     * at the module rate. Because it is a distinct child frame it never collides
     * with a `odom -> base_link` chain on `/tf`, so it gives consumers a
     * high-rate fused pose without touching the canonical robot tree (no sensor
     * frames hang off it). Default off. */
    bool publish_fused_vehicle_tf = false;

    /// Child frame name for `publish_fused_vehicle_tf`.
    std::string fused_vehicle_frame_name = "base_link_fused";

    /** If true, publish the estimated IMU attitude yaw bias (see
     * `imu_attitude_yaw_bias_sigma_deg`) as module diagnostics, once per diagnostics period
     * (the MOLA ROS 2 bridge publishes them as `std_msgs/Float32` topics
     * `mola_diagnostics/<module>/imu_yaw_bias_deg` and `.../imu_yaw_bias_sigma_deg`).
     * Default off. */
    bool publish_imu_yaw_bias = false;

    /** If true (and `imu_bias_tracking`), publish the gyroscope bias of each tracked IMU as
     * module diagnostics, once per diagnostics period: `imu_bias_<label>_gyro_{x,y,z}_deg_s`
     * and `imu_bias_<label>_gyro_sigma_deg_s` (largest axis sigma), with the label's
     * non-alphanumeric characters replaced by `_`, plus the motion hint counts
     * `imu_bias_hints_static` and `imu_bias_hints_moving`. Default off. */
    bool publish_imu_biases = false;

    /** If true (and `estimate_geo_reference`), publish the geo-reference convergence
     * as module diagnostics, once per diagnostics period: `georef_pos_sigma_m` and
     * `georef_ori_sigma_deg` (the largest position and orientation sigmas compared
     * against the thresholds), the thresholds themselves (`georef_max_pos_sigma_m`,
     * `georef_max_ori_sigma_deg`), `georef_azimuth_pinned` (1 while the azimuth is
     * still held by its initial anchor) and `georef_converged` (1 once the
     * geo-reference has been accepted). Default off. */
    bool publish_georef_convergence = false;

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

    /** Keep the pose of every keyframe with a pose or odometry observation as
     *  it leaves the sliding window, i.e. its final optimized value, and
     *  serve them through estimated_trajectory(). Off by default: in a live
     *  system nothing reads them and the container would grow without bound.
     *  Intended for offline runs that want the smoothed trajectory instead of
     *  the front end's own registered poses. A reset() keeps them, and records
     *  the keyframes still in the window as final.
     */
    bool keep_finalized_trajectory = false;

    double min_time_difference_to_create_new_frame = 0.01;  // [s]

    /// If the time between two keyframes is larger than this, a warning will be
    /// emitted; but the algorithm will keep trying its best.
    double time_between_frames_to_warning = 3.0;  // [s]

    /** When adding GNSS observations, specially with consumer grade receivers with errors larger
     * than a few centimeters, we may be more permissive in the temporal distance between the GNSS
     * datum and the associated existing keyframe. This parameter is the extended, alternative value
     * to use instead of "min_time_difference_to_create_new_frame". [seconds]
     * The fix is attached to the closest keyframe, and the antenna is moved to the
     * fix time with that keyframe's twist, so this offset does not bias the fix.
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
     *  Gaussian model defaults. Lower a1..a4 for platforms with accurate
     *  encoders and no wheel slip; raise for loose terrain.
     *
     *  Units, verified against MRPT's computeFromOdometry_modelGaussian()
     *  implementation directly (its own header/docs comment says a2 [m/deg]
     *  and a3 [deg/m], which is WRONG -- the implementation multiplies a2
     *  and a3 by CPose2D::phi(), which is radians, not by a degrees
     *  conversion of it):
     *    sigma_xy  = minStdXY  + a1*|translation| + a2*|rotation_rad|
     *    sigma_phi = minStdPHI + a3*|translation| + a4*|rotation_rad|
     *  So a1 [m/m], a2 [m/rad], a3 [rad/m], a4 [rad/rad] (dimensionless
     *  either way). Don't trust a docs-only cross-check for this MRPT
     *  struct; the implementation is the source of truth.
     *
     *  min_std_xy/min_std_phi_deg are NOT the effective floor in practice:
     *  fuse_odometry_locked() adds a further +1e-4 to every diagonal
     *  covariance entry after this model runs (an independent numerical
     *  regularizer, unrelated to these two fields), i.e. sigma >= 0.01 m /
     *  0.01 rad (~0.57 deg) regardless of what these are set to. Tuning
     *  either below that is a no-op today.
     *  @{ */
    double odom_motion_model_a1 = 0.01;  // [m/m] noise growth per m traveled
    double odom_motion_model_a2 = 0.05729577951;  // [m/rad] MRPT default: RAD2DEG(0.001)
    double odom_motion_model_a3 =
        0.017453292519943295;  // [rad/m] MRPT default: DEG2RAD(1.0), noise growth per m traveled
    double odom_motion_model_a4              = 0.05;  // [rad/rad] noise growth per rad rotated
    double odom_motion_model_min_std_xy      = 1e-3;  // [m] -- see effective-floor note above
    double odom_motion_model_min_std_phi_deg = 0.1;  // [deg] -- see effective-floor note above
    /** @} */

    /** Extra uncertainty of a planar (2D) wheel-odometry increment in the
     *  degrees of freedom it does not measure: z, and pitch and roll. Added on
     *  top of the 0.01 m / 0.01 rad floor above. The default 0 asserts planar
     *  motion in the vehicle frame (no height, pitch or roll change between
     *  readings), right for a wheeled robot on even ground; raise both for
     *  uneven terrain, or for a platform that pitches and rolls as it moves.
     *  @{ */
    double odom_out_of_plane_std_z          = 0;  // [m]
    double odom_out_of_plane_std_roll_pitch = 0;  // [rad]
    /** @} */

    /** High-rate same-sensor decimation for fuse_pose() sources. If > 0,
     * readings of a given frame_id arriving less than this many seconds after
     * the last *kept* one of that same frame_id are dropped before they reach
     * the graph. A pose source publishing in the hundreds of Hz otherwise
     * packs the sliding window with keyframes that carry no new information,
     * and the solver pays for every one of them.
     *
     * Nothing is lost for odometry frames: a dropped reading does not advance
     * the source's chain of increments, so the next kept one asserts the whole
     * merged span as a single increment. A "map" pose stands alone, so there is
     * nothing to accumulate for it either.
     *
     * It applies to EVERY fuse_pose() source, e.g. LiDAR odometry too. Keep
     * it well below the period of any source that must not be thinned: at
     * about that period, timestamp jitter alone drops every other reading.
     * A dropped reading still refreshes the source's own-frame pose that
     * estimated_navstate() extrapolates from.
     *
     * 0 disables it. Independent of, and coarser than,
     * min_time_difference_to_create_new_frame. [seconds]
     */
    double pose_min_sample_period = 0.0;  // [s]

    /** \name Per-increment accuracy of fuse_pose() odometry frames
     *  Poses given in any frame other than reference_frame_name come from a
     *  drifting source (wheel, visual or LiDAR odometry), so they are fused as
     *  a chain of increments between consecutive keyframes, plus one absolute
     *  factor on the first reading to resolve T_map_to_odom_i. The covariance
     *  such a source publishes is that of its absolute dead-reckoned pose,
     *  which grows without bound and says nothing about one increment, so the
     *  increments' uncertainty is defined here instead.
     *  @{ */

    /** Linear sigma asserted for ONE increment, whatever it spans.
     * Motion-dependent error belongs in
     * relative_pose_increment_sigma_per_sqrt_meter, which scales with the
     * increment and so with pose_min_sample_period. [m]
     *
     * 0 (default) = automatic, set by loadFrom(): 0.1 mm while IMU preintegration
     * is enabled, 2 cm otherwise. Both values, and
     * the default relative_pose_increment_sigma_ang, suit LiDAR odometry; wheel
     * or visual odometry needs its own (e.g. 2 cm and 5 mrad).
     */
    double relative_pose_increment_sigma_lin = 0.0;  // [m]

    /** Angular counterpart of relative_pose_increment_sigma_lin. Must be > 0.
     * [rad] */
    double relative_pose_increment_sigma_ang = 1e-5;  // [rad]

    /** Growth of the per-increment linear uncertainty with the distance the
     * increment spans, as a random walk: the asserted variance is
     * `relative_pose_increment_sigma_lin^2 + k^2 * distance`. A dead-reckoned
     * error is not a fixed number per reading: slips and skids happen while
     * moving, independently along the path. Because variance, not sigma, grows
     * linearly with distance, the total asserted over a path does not depend on
     * how many increments it is split into (keyframe rate,
     * pose_min_sample_period). relative_pose_increment_sigma_lin is the floor
     * asserted while standing still. 0 disables it. [m/sqrt(m)] */
    double relative_pose_increment_sigma_per_sqrt_meter = 0.0;

    /** Angular counterpart of relative_pose_increment_sigma_per_sqrt_meter,
     * growing with the rotated angle. [rad/sqrt(rad)] */
    double relative_pose_increment_sigma_per_sqrt_rad = 0.0;

    /** If > 0, fuse_pose() factors, both absolute and relative, are wrapped in
     * a Huber m-estimator with this threshold, in whitened units, i.e. in
     * sigmas of the covariance supplied for that factor. So it only means
     * something when that covariance is calibrated. A dead-reckoning source
     * produces occasional gross errors (a slipped foot, a skidded wheel), and
     * under a plain Gaussian each one pulls with unbounded weight. Huber is
     * convex: unlike a redescending kernel it cannot switch off a correct
     * reading whose keyframe starts far from it. The one-time anchor factor
     * of a relative source is not wrapped. 0 disables it. */
    double pose_robust_huber_threshold = 0.0;

    /** @} */

    /** High-rate same-sensor decimation for IMU. If > 0, IMU readings are
     * averaged over periods of this many seconds, and one averaged reading per
     * period is fused (see mola::imu::ImuAverager). Averaging, rather than keeping one raw
     * reading, keeps vibration faster than this rate from aliasing into the
     * gravity and angular-velocity factors. 0 fuses every reading. [seconds]
     */
    double imu_min_sample_period = 0.0;  // [s]

    /** Linear random-walk sigma for the constant-velocity factor between keyframes.
     *  It is a prior on the vehicle's acceleration, centripetal included: it must
     *  not be tighter than the platform's real accelerations, or the optimizer
     *  cuts every turn short and bends an accurate pose source with it. */
    double sigma_random_walk_acceleration_linear = 5.0;  // [m/s²]
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
     * behind genuine acceleration: a lagging prior at the end of a turn can
     * pull the registration into a heading error. */
    double predict_twist_filter_time_const = 0.1;

    /** If true, the covariance of a prediction in an odometry source's own
     * frame (the motion prior a front end such as a LiDAR odometry receives)
     * grows with that source's measured one-step prediction error, instead of
     * with the twist marginal of the graph: the latter mostly reflects how
     * loosely the kinematic factors tie velocities to poses, not how well the
     * motion is predicted. */
    bool predict_covariance_from_residuals = true;

    /** [s] Time constant of the running mean of those prediction errors. */
    double predict_residual_time_const = 10.0;
    /** Per-link sigma of the trapezoidal position integration. Loose, so that
     * velocities follow the poses and not the other way around. */
    double sigma_integrator_position    = 1.0;  // [m]
    double sigma_integrator_orientation = 0.10;  // [rad]

    /** @name Floor on the uncertainty reported by estimated_navstate()
     *  @{ */

    /** [m] Flat, dt-independent floor added to the position variance of the
     * pose returned by estimated_navstate() (and by the asynchronous fast
     * predictor). 0 disables it, which is the shipped default and reproduces
     * the behavior of every release before this parameter existed.
     *
     * This is NOT a model of anything the graph knows. It exists because a
     * front end may use the returned covariance as a WEIGHT, not merely as a
     * diagnostic: mola_lidar_odometry turns a non-zero `pose.cov_inv` into a
     * prior factor inside its ICP solve. The marginal this estimator reports is
     * the graph's own opinion of its extrapolation, and when the graph is
     * confident that opinion is tight enough to pin the registration to the
     * prediction instead of letting the scan data move it. A floor bounds how
     * hard the estimator is allowed to lean on the front end, independently of
     * how well conditioned the window happens to be.
     *
     * The lightweight estimator has carried the same knob since it was written
     * (`mola_state_estimation_simple`, same parameter names, 0.5 m / 0.1 rad),
     * which is why the two estimators hand a front end priors of very different
     * strength. Set this to that value to make the two comparable.
     *
     * Applied isotropically, so it is invariant to whether the caller reads the
     * rotation block in Euler (yaw, pitch, roll) or Lie-tangent (w_x, w_y, w_z)
     * order: adding the same variance to all three diagonal entries commutes
     * with any permutation of them.
     */
    double sigma_relative_pose_linear = 0.0;  // [m]

    /** [rad] Angular counterpart of sigma_relative_pose_linear. 0 disables it.
     */
    double sigma_relative_pose_angular = 0.0;  // [rad]

    /** @} */

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
     * uncertainty or noise sigma [degrees]. Set to 0 to ignore IMU attitude readings, e.g. for
     * drivers that publish a constant placeholder orientation instead of flagging it as missing.
     */
    double imu_attitude_sigma_deg = 2.0;

    /** When an IMU provides global attitude measurements (azimuth and gravity aligned), this must
     * define the angle (in degrees) to add to IMU yaw orientation to obtain azimuth so 0 deg is
     * North. Note that ENU axes are such vehicle yaw is 0 when pointing East instead.
     * Example cases:
     * - IMU absolute yaw=0 points True North ==> offset=0
     * - IMU absolute yaw=0 points East ==> offset=-90
     */
    double imu_attitude_azimuth_offset_deg = 0.0;

    /** Stationary sigma [degrees] of an unknown bias of the IMU attitude azimuth (yaw), e.g.
     * magnetometer hard/soft-iron residuals or a mounting offset, estimated as a state. All
     * readings share it, so averaging them never brings the heading sigma below this value while
     * nothing else observes the heading, unlike independent readings, whose fused sigma shrinks
     * as 1/sqrt(N). Set to 0 to disable (attitude readings fused as independent).
     */
    double imu_attitude_yaw_bias_sigma_deg = 0.0;

    /** Correlation time [s] of the IMU attitude yaw bias, modeled as a first-order Gauss-Markov
     * process: b(k) = phi * b(k-1) + w, phi = exp(-dt/tau), var(w) = sigma^2 (1 - phi^2).
     * Larger values make the bias, once observed (e.g. by a localization in a map), last longer.
     */
    double imu_attitude_yaw_bias_time_constant = 120.0;

    /** Minimum period [s] between fused attitude readings of each IMU (0 = all of them, after
     * imu_min_sample_period). Consecutive absolute attitude readings are strongly correlated, so a
     * high rate adds cost but no information. Gravity and angular velocity are not affected.
     */
    double imu_attitude_min_sample_period = 0.0;

    /** When using an IMU with acceleration, use this sigma [deg] to estimate the up-vector, hence
     * gravity-align the map.
     * Set to 0 to disable.
     */
    double imu_normalized_gravity_alignment_sigma = 2.0;

    /** Accelerometer readings are averaged over periods of about this length [s], and one
     * gravity factor is fused per period (0 = one per imu_min_sample_period reading).
     * On a vibrating vehicle short averages tilt by several degrees, while vibration averages
     * out over ~1 s; vehicle accelerations still bias it, hence the sigma above.
     */
    double imu_gravity_min_sample_period = 1.0;

    /** If true, the centripetal acceleration of a turning vehicle (the gyro
     * angular velocity times the newest estimated linear velocity) is removed
     * from each accelerometer reading before it is averaged into a gravity
     * observation: unlike vibration, it does not average out over a turn, and
     * would read as a tilt towards the outside of it. Only applied to readings
     * in m/s^2, not to accelerometers normalized to g. */
    bool imu_gravity_compensate_centripetal = true;

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

    /** If true, track the gyroscope and accelerometer biases of each IMU
     * (mola::imu::ImuBiasTracker: standstill windows detected from its readings, vetoed by
     * motion hints from the odometry inputs) and offer them to other modules as a
     * mola::ImuBiasSource. Requires mola_kernel and mola_imu_preintegration versions that
     * have them; otherwise it is ignored with a warning. Default off. */
    bool imu_bias_tracking = false;

    /// IMUs whose biases are tracked (regex of sensor labels), whether fused or not
    /// (do_process_imu_labels_re does not apply here).
    std::string imu_bias_labels_re = ".*";

    /// If true (and `imu_bias_tracking`), subtract the tracked gyroscope bias from the
    /// angular velocity readings before fusing them (imu_angular_velocity_sigma).
    bool imu_bias_correct_angular_velocity = true;

    /** Motion hints for the bias tracker, from the increments of each pose input (any
     * frame, by frame name) and wheel odometry over at least `imu_bias_hint_min_period`
     * seconds, and from fused twists: "static" if both the speed and the angular speed
     * are below these thresholds, else "moving". The thresholds must be above the
     * jitter of the inputs at rest (LiDAR odometry: a few mm and hundredths of a
     * degree per scan); the speed one can be loose, since a straight constant speed
     * motion does not bias the readings. */
    double imu_bias_hint_max_speed             = 0.1;  // [m/s]
    double imu_bias_hint_max_angular_speed_deg = 0.1;  // [deg/s]
    double imu_bias_hint_min_period            = 1.0;  // [s]

    /// mola::imu::ImuBiasTracker::Parameters (empty: its defaults).
    mrpt::containers::yaml imu_bias_tracker = mrpt::containers::yaml::Map();

    /** IMU preintegration between consecutive keyframes: every kinematic link
     * whose interval is covered by raw readings gets an on-manifold
     * preintegration factor (gtsam::ImuFactor on the raw accelerometer and
     * gyroscope readings), and every keyframe gets a gyroscope and
     * accelerometer bias variable, chained by a random walk. Gravity is along
     * -z of {enu}: the factors include T_enu_to_map, so {map} may be tilted.
     * Default on; inert without IMU readings. */
    struct ImuPreintegration
    {
        bool        enabled         = true;
        std::string sensor_label_re = ".*";  //!< IMUs used (regex of labels)

        double gyro_noise_density     = 2.4e-4;  //!< [rad/s/sqrt(Hz)]
        double accel_noise_density    = 1.6e-3;  //!< [m/s^2/sqrt(Hz)]
        double gyro_bias_random_walk  = 2e-5;  //!< [rad/s^2/sqrt(Hz)]
        double accel_bias_random_walk = 3e-4;  //!< [m/s^3/sqrt(Hz)]
        double integration_sigma      = 1e-4;  //!< position integration noise
        double gravity                = 9.80665;  //!< [m/s^2]

        /** Multiplier taking the accelerometer readings to m/s^2. 0 (default):
         * automatic, decided once from their mean norm over the first readings,
         * since some drivers publish them in units of g. */
        double accel_scale = 0;

        double initial_gyro_bias_sigma  = 0.02;  //!< [rad/s]
        double initial_accel_bias_sigma = 0.2;  //!< [m/s^2]

        /** The constant-velocity linear factors of a link (velocity and position
         * integration) contradict any acceleration the IMU measures: their
         * sigmas are multiplied by this while preintegration is on. They stay,
         * weak, so a link without IMU data is still determined. */
        double linear_kinematics_sigma_scale = 10.0;

        /// Largest gap between consecutive readings integrated across [s].
        double max_reading_gap = 0.1;

        /** Floor of the preintegrated uncertainty, added to it: unmodeled errors
         * (timing, extrinsics, vibration), and numerical conditioning, since
         * keyframes can be milliseconds apart. Rotation [rad], position [m],
         * velocity [m/s]. */
        double min_sigma_rotation = 2e-4;
        double min_sigma_position = 1e-3;
        double min_sigma_velocity = 5e-3;

        /// Links shorter than this get no preintegration factor [s].
        double min_link_duration = 2e-3;
    };
    ImuPreintegration imu_preintegration;

    /** If true, the increments that fuse_pose() asserts for an odometry frame
     * use the covariance the source publishes with each reading, as the
     * covariance of the motion since its previous reading, instead of
     * relative_pose_increment_sigma_*. Right for a registration against a
     * local map built from the recent trajectory, whose covariance is relative
     * to it; wrong for a dead-reckoned source, whose covariance grows without
     * bound. Default off. */
    bool relative_pose_increment_from_source_covariance = false;

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
     *  Gaussian). It is also the scale of the other kernels in
     *  gnss_robust_kernel.
     */
    double gnss_huber_threshold = 1.5;  // [sigmas]

    /** Robust kernel of the GNSS factors: "huber" (default), or a redescending
     *  one, whose influence fades for large residuals: "cauchy",
     *  "geman_mcclure" or "dcs". Scale: gnss_huber_threshold. Kernels with zero
     *  influence beyond a cutoff (e.g. Tukey) are not offered: they leave the
     *  geo-reference unobserved while the estimate is still far off. */
    std::string gnss_robust_kernel = "huber";

    /** \name GNSS admission tests and innovation gate
     *  A fix must pass these before it becomes a factor. Rejections are counted
     *  (gnss_gate_stats()) and logged as throttled warnings.
     * @{ */

    /** GGA fix qualities accepted, separated by spaces or commas (e.g. "4 5" for
     *  RTK fixed and float only; 1: single point, 2: DGPS, 4: RTK fixed, 5: RTK
     *  float). Empty = any valid fix (quality 0 is always discarded). */
    std::string gnss_accepted_fix_qualities;

    /** Fixes whose receiver horizontal sigma exceeds this are discarded [m].
     *  0 = no limit. The simple estimator uses 0.20 to keep RTK-fixed only. */
    double gnss_max_horizontal_sigma = 0.0;  // [m]

    /** Fixes whose receiver horizontal sigma exceeds this many times the lowest
     *  one received in the last `gnss_sigma_ratio_window` seconds are discarded.
     *  Receivers that keep reporting a fixed solution while it degrades (e.g.
     *  multipath) raise their sigma: this catches it without a receiver-specific
     *  absolute threshold. Sigmas below `gnss_min_sigma_floor_xy` never trigger
     *  it, since the floor makes them equivalent. 0 = disabled.
     *
     *  Use it only when the odometry is better than the degraded fixes: rejected
     *  fixes no longer bound its drift. With good LiDAR-inertial odometry, 2
     *  with floors of 0.02 / 0.05 m works well; the floors must be that low for
     *  it to catch fixes just a few cm worse than the best ones. */
    double gnss_max_sigma_ratio = 0.0;

    /** Time window [s] for the reference sigma of `gnss_max_sigma_ratio`. It
     *  must outlast the degraded stretches: a short one makes the degraded
     *  fixes their own reference. */
    double gnss_sigma_ratio_window = 300.0;  // [s]

    /** Fixes whose height differs from the datum height by more than this are
     *  discarded [m]. 0 = no limit. A cheap guard against fabricated fixes,
     *  e.g. a datum position with zero height. */
    double gnss_max_height_deviation = 0.0;  // [m]

    /** Innovation gate: a fix is rejected if the squared Mahalanobis distance
     *  of its ENU position to the predicted antenna position exceeds this
     *  (chi-squared, 3 DOF; 25 is ~99.998%). 0 disables the gate. */
    double gnss_gate_chi2 = 25.0;

    /** Uncertainty of the predicted antenna position right after a good fix,
     *  besides the fix's own [m]: extrapolation, lever arm, time-stamping. */
    double gnss_gate_prediction_sigma = 0.5;  // [m]

    /** Growth of the predicted position sigma with the distance traveled since
     *  the last accepted fix [m/m], so that the gate opens up during a GNSS
     *  outage instead of locking out the fixes after it. */
    double gnss_gate_drift_per_meter = 0.02;

    /** After this many consecutive rejected fixes that agree with each other,
     *  the estimate is the one deemed wrong: they are accepted again, with a
     *  warning. 0 = never. Agreeing means, with a prediction, that the fixes
     *  moved like the predicted antenna (within half the predicted path, which
     *  allows heading errors up to ~30 deg) over at least
     *  gnss_gate_reacquire_min_path; without one, mutually consistent positions
     *  whose displacement matches the odometry's over that same distance (or,
     *  with no odometry at all, just mutually consistent positions).
     *  A source at a fixed wrong position does not move with the vehicle. */
    int gnss_gate_reacquire_after = 10;

    /** Predicted displacement the vehicle must cover during a rejected run before
     *  it can be reacquired [m]: standing still, a drifted estimate and a source at a
     *  fixed wrong position look alike. */
    double gnss_gate_reacquire_min_path = 5.0;  // [m]

    /** Before the geo-reference converges, consecutive fixes are compared with
     *  each other: they may not be farther apart than this speed allows, plus
     *  their uncertainty [m/s]. */
    double gnss_max_plausible_speed = 50.0;  // [m/s]

    /** With estimate_geo_reference, the datum is defined only after this many
     *  consecutive, mutually consistent fixes (the first of them), so a single
     *  bogus fix at start-up cannot define the whole map frame. */
    int gnss_datum_min_consistent_fixes = 3;

    /** @} */

    /** Lower bound [m] on the horizontal (East, North) standard deviation of a
     *  GNSS fix: a receiver variance below it is raised to it. RTK fixes often
     *  report ~1 cm, smaller than what the rest of the graph can explain
     *  (time-stamping, lever arm, odometry noise), and an overconfident fix bends
     *  the trajectory instead of nudging it. 0 disables it. Same name and default
     *  as in the simple estimator.
     */
    double gnss_min_sigma_floor_xy = 0.10;  // [m]

    /// Same floor for the vertical (Up) component [m].
    double gnss_min_sigma_floor_z = 0.30;  // [m]

    /** If not empty, the final geo-reference (geodetic datum + T_enu_to_map,
     *  fixed or estimated) is written to this YAML file when the estimator is
     *  destroyed, in the same format as mp2p_icp georeferencing files. Nothing
     *  is written if no geo-reference is known by then.
     */
    std::string save_final_georef_to_file;

    /** @} */

    /** @name Nonlinear optimization
     * @{ */

    /** Each new sensor will become a call to isam2.update(), plus this number of additional
     * refining steps. In theory, more steps lead to more accurate results. */
    uint32_t additional_isam2_update_steps = 3;

    /** iSAM2 relinearization threshold for poses, in tangent-space units (m
     * and rad): a pose is relinearized only once its estimate moved this much
     * from its linearization point. Coarser values leave the solution away from
     * the optimum by more than an accurate pose source's own error. Velocities
     * and IMU biases use a fixed 0.1. */
    double isam2_relinearize_threshold = 0.01;

    /** Eliminate the newest states last, so they stay at the root of the iSAM2
     * Bayes tree: each update and each query of the current state then only
     * touches a short path of the tree. Only has an effect with GTSAM 4.2. */
    bool isam2_newest_keys_last = true;

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

    /** Allow fusing CObservationRobotPose observations labeled "ground_truth",
     *  which is how MOLA's offline dataset sources publish their reference
     *  trajectory. Off by default: fusing it makes any accuracy number measured
     *  against that same trajectory meaningless. */
    bool fuse_ground_truth_label = false;

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
