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
 * @file   StateEstimationSmoother.h
 * @brief  Fuse of odometry, IMU, and SE(3) pose/twist estimations.
 * @author Jose Luis Blanco Claraco
 * @date   Jan 22, 2024
 */
#pragma once

// this package:
#include <mola_gtsam_factors/id.h>
#include <mola_kernel/interfaces/LocalizationSourceBase.h>
#include <mola_kernel/interfaces/MapSourceBase.h>
#include <mola_kernel/interfaces/NavStateFilter.h>
#include <mola_kernel/interfaces/RawDataSourceBase.h>
#include <mola_kernel/interfaces/VizInterface.h>
#include <mola_kernel/utils/RegexCache.h>
#include <mola_kernel/version.h>
#include <mola_state_estimation_smoother/Parameters.h>

// MOLA:
#include <mola_imu_preintegration/ImuIntegrator.h>

#if __has_include(<mola_kernel/interfaces/ImuBiasSource.h>) && \
    __has_include(<mola_imu_preintegration/ImuBiasTracker.h>)
#include <mola_imu_preintegration/ImuBiasTracker.h>
#include <mola_kernel/interfaces/ImuBiasSource.h>
/** Feature macro: the smoother can track IMU biases and offer them as an ImuBiasSource. */
#define MOLA_SMOOTHER_HAS_IMU_BIAS_SOURCE 1
#endif

// MRPT:
#include <mrpt/containers/bimap.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/pimpl.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/topography/data_types.h>

// std:
#include <array>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <thread>
#include <utility>
#include <vector>

namespace mola::state_estimation_smoother
{
// Defined in src/, kept out of the public API (used only via pointer here):
struct Snapshot;
class FastPredictor;

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
 *      existing frames of reference.
 * - `fuse_pose()`: Can be used to integrate information from any "odometry" or
 *     "localization" input, as mentioned above.
 * - `fuse_gnss()`: Integrate GNSS observations, to help with localization in geo-referenced maps,
 *     or to automatically find-out the geo-referencing of a map.
 * - `fuse_imu()`: Used to help with (1) global azimuth in geo-referenced maps, (2) vertical
 *     direction from accelerometer, (3) angular velocity from gyroscope.
 *
 * Usage:
 * - (1) Call initialize() to set the required parameters.
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
class StateEstimationSmoother : public mola::NavStateFilter,
                                public mola::LocalizationSourceBase,
                                public mola::MapSourceBase
#if defined(MOLA_SMOOTHER_HAS_IMU_BIAS_SOURCE)
    ,
                                public mola::ImuBiasSource
#endif

{
    DEFINE_MRPT_OBJECT(StateEstimationSmoother, mola::state_estimation_smoother)
   private:
    class AbsFactorConstVelKinematics;  // Forward decls.
    class AbsFactorTricycleKinematics;

   public:
    StateEstimationSmoother();

    // Make not copiable due to the pimpl gtsam state.
    StateEstimationSmoother(const StateEstimationSmoother&)            = delete;
    StateEstimationSmoother(StateEstimationSmoother&&)                 = delete;
    StateEstimationSmoother& operator=(const StateEstimationSmoother&) = delete;
    StateEstimationSmoother& operator=(StateEstimationSmoother&&)      = delete;
    // Out-of-line: fastPredictor_ is a unique_ptr to an incomplete type here.
    ~StateEstimationSmoother();

    /** \name Main API
     *  @{ */

    /** Parameters can only be set via initialize(), then read-only accesses through this method. */
    const Parameters& parameters() { return params_; }

    /**
     * @brief Initializes the object and reads all parameters from a YAML node.
     * @param cfg a YAML node with a dictionary of parameters to load from.
     */
    void initialize(const mrpt::containers::yaml& cfg) override;

    void spinOnce() override;

    /** Resets the estimator state to an initial state. Tracked IMU biases are kept. */
    void reset() override;

    /** Like reset(), but {map} and the odometry frames are kept: the poses
     * fused next continue in them, the first one is not linked to the origin,
     * and T_enu_to_map keeps its rotation. */
#if defined(MOLA_KERNEL_NAVSTATE_FILTER_HAS_RESET_MOTION)
    void reset_motion() override;
#else
    void reset_motion();
#endif

#if defined(MOLA_SMOOTHER_HAS_IMU_BIAS_SOURCE)
    /** \name ImuBiasSource (with `imu_bias_tracking`)
     *  @{ */
    [[nodiscard]] std::vector<std::string> imu_bias_sensor_labels() const override;
    /** @} */
#endif

    /** Integrates a new SE(3) pose of the vehicle wrt frame_id.
     *
     *  If frame_id is the reference frame ("map"), the pose is fused as an
     *  absolute prior, with the given covariance. Any other frame_id is an
     *  odometry frame, which drifts, so only the increments between its
     *  consecutive readings are fused, plus one absolute factor on its first
     *  reading (with the given covariance) to resolve T_map_to_odom_i.
     *  The increments' uncertainty comes from the relative_pose_increment_*
     *  parameters, not from the given covariance.
     */
    void fuse_pose(
        const mrpt::Clock::time_point& timestamp, const mrpt::poses::CPose3DPDFGaussian& pose,
        const std::string& frame_id) override;

#if defined(MOLA_KERNEL_NAVSTATE_FILTER_HAS_FUSE_POSE_DEFERRED)
    /** As fuse_pose(), with a covariance that becomes ready later. With
     *  `async_backend`, the backend thread waits for it (before taking the
     *  state lock), so the caller never does; otherwise it is awaited here.
     */
    void fuse_pose_deferred(
        const mrpt::Clock::time_point& timestamp, const mrpt::poses::CPose3D& mean,
        const std::shared_future<mrpt::math::CMatrixDouble66>& cov,
        const std::string&                                     frame_id) override;
#endif

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

    /** Anchors T_enu_to_map to a known, fixed value (equivalent to setting
     *  the `fixed_geo_reference` YAML parameter, but at runtime): used by
     *  localization front-ends (e.g. mola_lidar_odometry) once they load a
     *  geo-referenced map, so GNSS fixes constrain the vehicle's position in
     *  that map's frame without the estimator also trying to solve for
     *  T_enu_to_map itself (that combination leaves T_enu_to_map and every
     *  fused Pose3RotationFactor/MeasuredGravityFactor-derived orientation
     *  jointly underconstrained, since only their *relative* rotation is
     *  ever measured). Re-initializes the smoother's factor graph, so it
     *  should be called before any run-relevant sensor fusion, not mid-run.
     */
#if defined(MOLA_KERNEL_NAVSTATE_FILTER_HAS_GEO_REFERENCE)
    void set_geo_reference(const mola::Georeferencing& georef) override;

    /** Implements NavStateFilter::get_geo_reference(): the fixed geo-reference,
     *  or, when `estimate_geo_reference` is enabled, the datum (first GNSS fix)
     *  with the latest estimate of T_enu_to_map, once its azimuth is observable.
     *  std::nullopt otherwise.
     */
    [[nodiscard]] std::optional<mola::Georeferencing> get_geo_reference() const override;
#endif

    /** Writes the current geo-reference (see get_geo_reference()) to a YAML
     *  file, in the mp2p_icp georeferencing format. Returns false if there is
     *  none yet or the file cannot be written. Done automatically on destruction
     *  if the `save_final_georef_to_file` parameter is set.
     */
    bool save_geo_reference_to_file(const std::string& yamlFile) const;

    /// Counters of the GNSS admission tests and innovation gate.
    struct GnssGateStats
    {
        size_t accepted             = 0;  //!< fixes that passed every test
        size_t rejected             = 0;  //!< by any test below
        size_t rejected_admission   = 0;  //!< quality, covariance, sigma, height
        size_t rejected_innovation  = 0;  //!< innovation gate or fix-to-fix consistency
        size_t consecutive_rejected = 0;  //!< current run of rejections
        size_t reacquired           = 0;  //!< runs accepted after gnss_gate_reacquire_after
        size_t datum_restarts       = 0;  //!< datum candidates dropped as inconsistent
    };
    [[nodiscard]] GnssGateStats gnss_gate_stats() const;

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

    /** Implements NavStateFilter::estimated_trajectory().
     *
     *  Returns the smoothed trajectory: each keyframe's pose as it was when it
     *  left the sliding window, i.e. after every measurement the window could
     *  bring to bear on it, plus the keyframes still inside the window at their
     *  current value. This is a different quantity from estimated_navstate(),
     *  which anchors on the newest keyframe and extrapolates: the newest state
     *  is the least optimized one, the oldest is the most.
     *
     *  Only keyframes holding a pose or odometry observation are returned:
     *  the others (e.g. IMU, twist or GNSS readings only) have a poorly
     *  observed attitude and would add jitter between the front end's poses.
     *
     *  Requires `keep_finalized_trajectory` (off by default). An odometry
     *  frame is served through the latest estimate of that frame's transform,
     *  held constant over the trajectory. In the ENU frame, each keyframe goes
     *  through the T_enu_to_map estimated when it left the window, since
     *  {map} itself moves with respect to ENU while that estimate is refined;
     *  keyframes finalized before the geo-reference converged, and those still
     *  in the window, use the latest one.
     */
    [[nodiscard]] std::optional<mrpt::poses::CPose3DInterpolator> estimated_trajectory(
        const mrpt::Clock::time_point& start_time, const mrpt::Clock::time_point& end_time,
        const std::string& frame_id) override;

    /// Returns a list of known odometry frame_ids:
    [[nodiscard]] auto known_odometry_frame_ids() -> std::set<std::string>;

    /// Gets the latest estimated transform of T_enu_to_map
    [[nodiscard]] std::optional<mrpt::poses::CPose3DPDFGaussian> estimated_T_enu_to_map() const;

    /** Gets the latest estimated IMU attitude yaw bias, as mean and sigma [rad], or empty if
     *  it is not estimated (`imu_attitude_yaw_bias_sigma_deg` = 0) or not solved yet.
     *  A reading's ENU yaw is the true one plus this bias. */
    [[nodiscard]] std::optional<std::pair<double, double>> estimated_imu_yaw_bias() const;

    /// Gets the latest estimated transform of "T_map_to_odometry_frame_i", by frame ID name.
    [[nodiscard]] std::optional<mrpt::poses::CPose3DPDFGaussian> estimated_T_map_to_odometry_frame(
        const std::string& frame_id) const;

    /// Implements NavStateFilter::has_converged_localization
    [[nodiscard]] bool has_converged_localization(mrpt::poses::CPose3DPDFGaussian& pose) const
#if MOLA_VERSION_CHECK(2, 5, 0)
        override
#endif
        ;

    /// Returns the current georeferencing, if available
    [[nodiscard]] std::optional<mola::Georeferencing> current_georeferencing() const;

    /** Number of constant-velocity kinematic factors currently live in the
     *  underlying factor graph (2 per kinematic link).
     *
     *  For tests only. It counts the real GTSAM graph rather than this class's own
     *  link bookkeeping, so that a test can tell whether a factor removal actually
     *  reached the solver. Returns a plain count so GTSAM stays out of this header.
     */
    [[nodiscard]] size_t count_const_vel_factors_for_testing() const;

    /** Identifies each constant-velocity kinematic link currently live in the
     *  underlying factor graph by its endpoint timestamps, one entry per link
     *  (not per factor: each link emits two factors, linear and angular).
     *
     *  For tests only, to check the graph topology survives a splice as expected
     *  (e.g. that the direct a->b link was replaced by a->n and n->b, not left
     *  alongside them).
     */
    [[nodiscard]] std::set<std::pair<mrpt::Clock::time_point, mrpt::Clock::time_point>>
        const_vel_factor_links_for_testing() const;

    /** @} */

    // Implementation of RawDataConsumer
    void onNewObservation(const CObservation::ConstPtr& o) override;

   private:
    Parameters params_;

    // everything related to gtsam is hidden in the public API via pimpl
    // to reduce compilation dependencies, and build time and memory usage.
    struct GtsamImpl;

    using odometry_frameid_t = uint8_t;
    using frame_index_t      = uint32_t;

    struct FrameState
    {
        mrpt::poses::CPose3D    pose;  //!< in the reference frame
        mrpt::math::TTwist3D    twist;  //!< in the local frame of reference
        std::set<frame_index_t> kinematic_links_to;
    };

    // Accesses to this struct values in state_ must be protected by stateMutex_
    /// Decimates IMU readings to one per imu_min_sample_period. Defined in the
    /// .cpp only, so this header's layout does not depend on the version of
    /// mola_imu_preintegration it is compiled against.
    struct ImuDecimator;

    struct State
    {
        State();

        mrpt::pimpl<GtsamImpl> gtsam;

        /// The next numeric ID to assign to a new frame, for usage in GTSAM symbols P(i), v(i)...
        frame_index_t next_frame_index = 0;

        /// A bimap of timestamps <=> frame indices. Updated by
        mrpt::containers::bimap<mrpt::Clock::time_point, frame_index_t> stamp2frame_index;

        /// A bimap of known odometry "frame_id" <=> "numeric IDs":
        mrpt::containers::bimap<std::string, odometry_frameid_t> known_odom_frames;

        /// Monotonic counter for odometry frame IDs; avoids collisions if entries are ever removed.
        odometry_frameid_t next_odom_frame_id = 1;

        /// The latest values from the estimator; updated in process_pending_gtsam_updates()
        std::map<frame_index_t, FrameState> last_estimated_states;

        /// Keyframes with a pose or odometry factor (marked when it is added).
        /// The others (e.g. IMU, twist or GNSS readings only) are left out of
        /// estimated_trajectory().
        std::set<frame_index_t> pose_keyframes;

        /// Low-pass-filtered twist of the newest keyframe, used as the
        /// extrapolation velocity for short-term prediction when
        /// `predict_twist_filter_enabled`. Damps the boundary keyframe's raw
        /// velocity (the least-constrained node), which otherwise injects jitter
        /// into a front end's motion prior. Reset with the rest of `State`.
        std::optional<mrpt::math::TTwist3D>    filtered_predict_twist;
        std::optional<mrpt::Clock::time_point> filtered_predict_twist_stamp;

        /// The latest values from the estimator; updated in process_pending_gtsam_updates().
        /// The covariances of odometry frames are filled in on first use, see
        /// complete_odom_frame_covariance_locked().
        mutable std::map<odometry_frameid_t, mrpt::poses::CPose3DPDFGaussian> last_estimated_frames;
        /// T_enu_to_map as last solved, once the whole solve succeeded
        /// (`solving_T_enu_to_map` holds it until then).
        std::optional<mrpt::poses::CPose3D> last_solved_T_enu_to_map;
        std::optional<mrpt::poses::CPose3D> solving_T_enu_to_map;

        /// The last raw pose received from each odometry source, expressed in
        /// that source's OWN frame {odom_i} (NOT in {map}), with its timestamp.
        /// `estimated_navstate(t, {odom_i})` extrapolates from this anchor so the
        /// short-term prediction stays continuous in the front end's own frame,
        /// instead of reconstructing it globally as X(kf) (-) T_map_to_odom_i
        /// (which would leak {map} corrections -- geo-ref / loop closure /
        /// per-solve jitter -- into the prediction; here the fixed-lag window
        /// keeps that leak small, but anchoring on the raw pose removes it).
        struct RawSourcePose
        {
            mrpt::Clock::time_point         stamp;
            mrpt::poses::CPose3DPDFGaussian pose;  //!< in {odom_i}
        };
        std::map<odometry_frameid_t, RawSourcePose> last_raw_pose_by_source;

        /// Per odometry source: running mean of the squared one-step prediction
        /// error per unit time squared, in twist order (vx vy vz wx wy wz).
        struct PredictionResiduals
        {
            std::array<double, 6> varRate{};
            size_t                count = 0;
            /// The first errors, whose per-axis median seeds varRate.
            std::vector<std::array<double, 6>> warmup;
        };
        std::map<odometry_frameid_t, PredictionResiduals> prediction_residuals;

        /// Per-source bookkeeping for the fuse_pose() odometry frames, which
        /// are always fused as relative increments: the
        /// keyframe carrying that source's one absolute anchor factor, and the
        /// tail of its chain of increment factors.
        struct RelativePoseChain
        {
            std::optional<frame_index_t>                   anchor_kf;
            std::optional<frame_index_t>                   last_kf;
            std::optional<mrpt::poses::CPose3DPDFGaussian> last_pose_in_odom;

            /// Timestamp of the last accepted sample: readings not strictly
            /// newer are dropped, since the chain tail is what the next
            /// increment is measured from.
            std::optional<mrpt::Clock::time_point> last_stamp;
        };
        std::map<odometry_frameid_t, RelativePoseChain> relative_pose_chains;

        /// Per fuse_pose() source, the timestamp of the last reading that was
        /// allowed through, for Parameters::pose_min_sample_period.
        std::map<odometry_frameid_t, mrpt::Clock::time_point> last_kept_pose_stamp;

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

        std::optional<mrpt::poses::CPose2D> last_wheels_odometry;
        std::optional<std::string>          last_wheels_odometry_name;

        /// Odometry frame the wheel odometry is registered under: its label,
        /// unless a pose source uses a frame of that same name.
        std::optional<std::string> wheels_odometry_frame;

        /// Stamp of the last wheel-odometry reading that was actually KEPT
        /// (turned into a keyframe/factor). Used by odometry_min_sample_period
        /// to merge higher-rate readings: while a reading arrives within that
        /// period of this stamp it is dropped without advancing the pose anchor
        /// (last_wheels_odometry), so the next kept reading fuses the whole
        /// accumulated increment.
        std::optional<mrpt::Clock::time_point> last_wheels_odometry_stamp;

        /// Absolute dead-reckoned wheel-odometry pose in its own {odom_i}
        /// frame, with the uncertainty accumulated over the whole history, for
        /// the one factor resolving T_map_to_odom_i and for the source's
        /// own-frame pose estimated_navstate() extrapolates from. The pose is
        /// the same one the source reports, and the covariance is the composed
        /// motion-model covariance of every increment since the first reading.
        /// Reset with the estimator.
        std::optional<mrpt::poses::CPose3DPDFGaussian> wheels_odometry_accumulated;

        /// Tail keyframe of the wheel-odometry relative-factor chain. Reset
        /// with the estimator.
        std::optional<frame_index_t> last_wheels_odometry_kf;

        /// Odometry reading representing last_wheels_odometry_kf: the first
        /// one that landed on it. The next increment factor starts here, so
        /// later readings on the same keyframe lose no motion.
        std::optional<mrpt::poses::CPose2D> last_wheels_odometry_at_kf;

        /// Keyframe carrying the single absolute pose-in-{odom_i} factor that
        /// resolves T_map_to_odom_i for wheel odometry. Set once
        /// and never renewed: the fixed-lag smoother marginalizes keyframes
        /// rather than dropping their factors, so that information survives its
        /// own keyframe. See fuse_odometry_relative_locked().
        std::optional<frame_index_t> wheels_odometry_anchor_kf;

        /// Created on first use. See ImuDecimator.
        std::shared_ptr<ImuDecimator> imu_decimator;

        /// Per IMU sensor label, stamp of its last fused attitude reading
        std::map<std::string, mrpt::Clock::time_point> last_imu_attitude_stamp;

        /// Per IMU sensor label, accelerometer readings being averaged for the next
        /// gravity factor (see imu_gravity_min_sample_period)
        struct GravityAverage
        {
            std::optional<mrpt::Clock::time_point> first_stamp;
            mrpt::math::TVector3D                  sum{0, 0, 0};
            size_t                                 count = 0;
        };
        std::map<std::string, GravityAverage> imu_gravity_average;

        /** Refer to Parameters for possible sources of this.
         * Anyways: this will always hold either the estimated or the fixed (externally set)
         * georeferencing parameters.
         * When this is still empty, it means we are still waiting for someone external to
         * send us the georeferencing data, or our internal estimator didn't obtained a quality
         * estimation yet.
         */
        std::optional<mola::Georeferencing> geo_reference;

        /// Will be populated with the first GNSS coords when in active estimation mode.
        std::optional<mrpt::topography::TGeodeticCoords> tentative_geo_coord_reference;

        /// Estimated geo-reference (datum + T_enu_to_map) as of the last solve.
        std::optional<mola::Georeferencing> lastSolvedGeoref;

        /// GNSS gate bookkeeping.
        struct GnssGate
        {
            /// A fix as a geocentric point, for fix-to-fix consistency tests.
            struct Fix
            {
                mrpt::math::TPoint3D              ecef;
                double                            sigma = 0;  //!< isotropic [m]
                mrpt::Clock::time_point           t;
                mrpt::topography::TGeodeticCoords geo;
            };
            std::optional<Fix> lastAccepted;  //!< used before there is a prediction
            std::optional<mrpt::math::TPoint3D> lastAcceptedAntennaMap;  //!< odometry at it
            /// Last fix time with a solved antenna position (odometry evidence).
            std::optional<mrpt::Clock::time_point> lastOdometryEvidence;
            std::vector<Fix>   datumCandidates;  //!< consecutive consistent fixes
            std::optional<Fix> lastRejected;  //!< last of the current rejected run
            size_t             rejectedRunLength = 0;  //!< mutually consistent rejections

            /// Receiver horizontal sigmas of recent fixes, for gnss_max_sigma_ratio.
            std::deque<std::pair<mrpt::Clock::time_point, double>> recentSigmaXY;

            /// Without a prediction: first fix of the current rejected run, and the
            /// antenna position in {map} at that time, if any (odometry evidence).
            std::optional<Fix>                  runStartFix;
            std::optional<mrpt::math::TPoint3D> runStartAntennaMap;

            /// First fix of the current rejected run and its prediction (ENU).
            std::optional<mrpt::math::TPoint3D> runStartFixEnu;
            std::optional<mrpt::math::TPoint3D> runStartPredEnu;

            // Prediction uncertainty model:
            double                              postSigma         = -1;  //!< < 0: not initialized
            double                              pathSinceAccepted = 0;  //!< [m]
            std::optional<mrpt::math::TPoint3D> lastPredictedAntenna;  //!< in {map}

            /// With a fixed geo-reference: the vehicle position has converged
            /// (sticky), so there is a prediction worth gating against.
            bool relocalized = false;

            /// After a reacquisition, the gate stays open while the estimate
            /// catches up with the fixes.
            bool reacquiring = false;
        };
        GnssGate gnssGate;

        /// Flag to track if we've already published the estimated geo-ref
        bool estimated_georef_published = false;

        // To be built from parameters strings when changed.
        RegexCache do_process_imu_labels_re;
        RegexCache do_process_odometry_labels_re;
        RegexCache do_process_gnss_labels_re;
    };

    State      state_;
    std::mutex stateMutex_;

    /// Copy of the latest solved IMU yaw bias (mean, sigma) [rad], with its own
    /// mutex so readers never wait for a solve holding stateMutex_.
    std::optional<std::pair<double, double>> lastYawBias_;
    mutable std::mutex                       lastYawBiasMutex_;

    /// Latest geo-reference convergence state (see publish_georef_convergence).
    struct GeorefConvergence
    {
        double pos_sigma      = 0;  //!< [m]
        double ori_sigma_deg  = 0;
        bool   azimuth_pinned = false;
        bool   converged      = false;
    };
    std::optional<GeorefConvergence> lastGeorefConvergence_;
    mutable std::mutex               lastGeorefConvergenceMutex_;

#if defined(MOLA_SMOOTHER_HAS_IMU_BIAS_SOURCE)
    /// IMU bias tracking (imu_bias_tracking), with its own mutex: fed from the raw
    /// data threads, read by the solver and other modules. Never lock stateMutex_
    /// while holding it.
    mola::imu::ImuBiasTracker imuBiasTracker_;
    mutable std::mutex        imuBiasMutex_;
    RegexCache                imuBiasLabelsRe_;

    /// The reference pose of each odometry input, for motion hints.
    struct MotionHintRef
    {
        double               t = 0;  //!< [s]
        mrpt::poses::CPose3D pose;
    };
    std::map<std::string, MotionHintRef> imuBiasHintRefs_;

    [[nodiscard]] std::optional<ImuBias> imu_bias_impl(
        const std::string&                            sensorLabel,
        const std::optional<mrpt::Clock::time_point>& t) const override;

    /// Converts a tracker estimate (imuBiasMutex_ held).
    [[nodiscard]] ImuBias to_imu_bias(
        const std::string& label, const mola::imu::ImuBiasTracker::Estimate& e) const;
    /// IMUs updated since the last advertise_imu_biases() (imuBiasMutex_).
    std::set<std::string> imuBiasUpdatedLabels_;
    void                  queue_imu_bias_updates_locked(const std::vector<std::string>& labels);
    /// A motion hint to the tracker, counted (imuBiasMutex_ held).
    void imu_bias_add_hint_locked(double t, bool isStatic);
#endif
    std::atomic_size_t imuBiasHintsStatic_{0};
    std::atomic_size_t imuBiasHintsMoving_{0};
    /// Advertises (from spinOnce()) the estimates updated since the last call.
    void advertise_imu_biases();
    /// Feeds a raw IMU reading to the bias tracker, if enabled.
    void imu_bias_add_reading(const mrpt::obs::CObservationIMU& imu);
    /// Motion hint from a new pose of an odometry input, if enabled.
    void imu_bias_add_pose(
        const mrpt::Clock::time_point& t, const mrpt::poses::CPose3D& pose,
        const std::string& source);
    /// Motion hint from a twist, if enabled.
    void imu_bias_add_twist(const mrpt::Clock::time_point& t, const mrpt::math::TTwist3D& tw);
    /// Removes the tracked gyroscope bias from a reading (fused angular velocity).
    void imu_bias_correct(mrpt::obs::CObservationIMU& imu) const;
    void publish_imu_bias_diagnostics();

    /// Poses of the keyframes already marginalized out, in the reference frame,
    /// each recorded at the moment it left the window. Only filled when
    /// `keep_finalized_trajectory` is set. Kept outside State so that a reset
    /// of the factor graph does not discard the history already collected.
    mrpt::poses::CPose3DInterpolator finalizedTrajectory_;
    /// Recent accelerometer readings of the preintegrated IMU (stamp [s], in
    /// the vehicle frame), read by level_enu_to_map_locked(). Kept outside
    /// State: a front end that levels itself from the IMU resets the estimator
    /// right after averaging it, and those readings are still valid.
    std::deque<std::pair<double, mrpt::math::TVector3D>> recentAccel_;
    /// Takes the preintegrated IMU's accelerometer readings to m/s^2 (0 while
    /// undecided), from imu_preintegration.accel_scale or the mean norm of the
    /// first readings (accNormSum_ over accNormCount_, since accFirstStamp_).
    /// Kept outside State, as recentAccel_: the units of the same sensor
    /// (accScaleLabel_) do not change on a reset.
    std::string accScaleLabel_;
    double      accScale_      = 0;
    double      accNormSum_    = 0;
    size_t      accNormCount_  = 0;
    double      accFirstStamp_ = 0;
    /// Same keyframes in ENU, through the T_enu_to_map estimated when each one
    /// left the window (the one it was optimized against). Only those finalized
    /// once the geo-reference was fixed or had converged.
    mrpt::poses::CPose3DInterpolator finalizedTrajectoryEnu_;
    /// GNSS gate counters; kept outside State so a reset does not clear them.
    GnssGateStats gnssGateStats_;
    /// Set by reset_motion() once keyframes had defined {map}: {map} keeps its
    /// origin and its tilt, so the first keyframe after it is not linked to the
    /// origin, odometry frames return where they were, and T_enu_to_map keeps
    /// the rotation below. A full reset() clears it, and so does a second
    /// reset_motion() with no successful solve in between.
    bool mapDefinedBeforeReset_ = false;
    /// Live T_map_to_odom_i of each odometry frame, by name, as of
    /// the last reset that kept the frames: a frame created again starts
    /// anchored there.
    std::map<std::string, mrpt::poses::CPose3D> keptOdomFrames_;
    /// Rotation of T_enu_to_map kept by reset_motion(): the leveling that
    /// preintegration used, with the azimuth as last solved. Leveling again
    /// while moving would read the vehicle's own acceleration as gravity.
    std::optional<mrpt::poses::CPose3D> keptEnuToMap_;
    bool                                params_loaded_ = false;

    /// Lock-free read model for async_backend mode. Populated at the end of each
    /// solve; queried by estimated_navstate()/spinOnce() without stateMutex_.
    std::unique_ptr<FastPredictor> fastPredictor_;

    /// Builds an immutable Snapshot of the current solution (newest keyframe
    /// anchor + frame transforms + georef). Assumes stateMutex_ is held.
    /// Returns nullptr if there is no keyframe/state to snapshot yet.
    [[nodiscard]] std::shared_ptr<const Snapshot> build_snapshot_locked() const;

    // ---- async backend (only used when params_.async_backend) ----
    std::thread                                                           backendThread_;
    std::mutex                                                            queueMutex_;
    std::condition_variable                                               queueCv_;
    std::deque<std::pair<mrpt::Clock::time_point, std::function<void()>>> ingestQueue_;
    std::atomic<bool>                                                     backendStop_{false};
    /// Covariances of queued fuse_pose_deferred() calls, guarded by
    /// queueMutex_: the backend waits for them before taking stateMutex_.
    std::vector<std::shared_future<mrpt::math::CMatrixDouble66>> deferredCovs_;
    bool                                                         backendRunning_ = false;

    /// Bumped by reset_locked() so a batch already detached by backend_loop()
    /// (pre-reset) is discarded instead of repopulating the fresh graph with
    /// stale measurements (e.g. across set_geo_reference()).
    std::atomic<uint64_t> resetEpoch_{0};

    /// Appends a work item to the backend queue and wakes the backend thread.
    void enqueue_async(const mrpt::Clock::time_point& stamp, std::function<void()> fn);

    /// Backend thread body: drains the queue, applies measurements in timestamp
    /// order under stateMutex_, runs one solve, and publishes a snapshot.
    void backend_loop();

    /// Starts the backend thread if async mode is on and it is not running yet.
    void start_backend_thread();

    /// Signals the backend thread to stop and joins it.
    void stop_backend_thread();

    /// Creates a new frame index for timestamp t, or returns the existing one if close enough.
    /// This also is in charge of the complex task of finding nearby existing frames and adding the
    /// kinematic factors to ensure smooth motion estimation.
    [[nodiscard]] frame_index_t create_or_get_keyframe_by_timestamp(
        const mrpt::Clock::time_point& t,
        const std::optional<double>&   overrideCloseEnough = std::nullopt);

    /// Creates or returns the existing ID, for an odometry frame_id:
    [[nodiscard]] odometry_frameid_t add_or_get_odom_frame_id(const std::string& frame_id_name);

    /// Name of the odometry frame for the wheel odometry labeled `odomName`.
    [[nodiscard]] std::string wheels_odometry_frame_locked(const std::string& odomName);

    /// Moves the wheel odometry out of the frame `frame_id`, if it uses it.
    void move_wheels_odometry_frame_aside_locked(const std::string& frame_id);

    /// While T_map_to_odom_i is still pending insertion into the smoother, sets
    /// its initial value consistent with a reading `poseInOdom` of keyframe `kf`.
    void seed_odom_frame_locked(
        odometry_frameid_t frame_id_idx, frame_index_t kf, const mrpt::poses::CPose3D& poseInOdom);

    /// Some measurement now observes {map}: its gauge anchor is no longer needed.
    void mark_map_observed_locked();

    /// False while a fixed geo-reference is set but no source has observed the
    /// position of {map} yet (it is only the first odometry frame then). Runs a pending
    /// synchronous solve, as estimated_navstate() does. Takes the locks
    /// it needs: not to be called with stateMutex_ held.
    [[nodiscard]] bool reference_frame_position_observed();
    /// A source observes the vehicle roll/pitch and/or yaw in {map}
    void mark_orientation_observed_locked(bool rollPitch, bool yaw);
    /// Replaces the orientation entries of a pose covariance that only a gauge
    /// anchor pins (not observed) with "unknown" ones
    void inflate_pinned_orientation_cov_locked(mrpt::math::CMatrixDouble66& cov) const;

    // ---- _locked variants: assume stateMutex_ is already held by the caller ----
    /// keepFrames: see reset_motion().
    void reset_locked(bool keepFrames);
    void reinitialize_gtsam_locked();
    void fuse_pose_locked(
        const mrpt::Clock::time_point& timestamp, const mrpt::poses::CPose3DPDFGaussian& pose,
        const std::string& frame_id);

    /// Wheel-odometry fusion: BetweenFactors between consecutive odometry
    /// keyframes, plus one absolute factor, added once, to resolve
    /// T_map_to_odom_i.
    void fuse_odometry_relative_locked(
        const mrpt::obs::CObservationOdometry& odom, const std::string& odomName,
        const mrpt::poses::CPose3DPDFGaussian& absolutePoseInOdom);
    void fuse_odometry_locked(
        const mrpt::obs::CObservationOdometry& odom, const std::string& odomName);
    void fuse_imu_locked(const mrpt::obs::CObservationIMU& imu);
    /// Creates the yaw bias variable of keyframe `kf`, if it does not exist yet
    void add_imu_yaw_bias_variable_locked(frame_index_t kf);
    void fuse_gnss_locked(const mrpt::obs::CObservationGPS& gps);
    void fuse_twist_locked(
        const mrpt::Clock::time_point& timestamp, const mrpt::math::TTwist3D& twist,
        const mrpt::math::CMatrixDouble66& twistCov);
    [[nodiscard]] frame_index_t create_or_get_keyframe_by_timestamp_locked(
        const mrpt::Clock::time_point& t,
        const std::optional<double>& overrideCloseEnough = std::nullopt, bool pickClosest = false);
    /// Runs the solve; never throws: on any failure it logs and resets the smoother.
    void                                              process_pending_gtsam_updates_locked();
    void                                              solve_locked();
    [[nodiscard]] std::optional<mola::Georeferencing> get_geo_reference_locked() const;
    /// Geo-reference to express {map} poses in ENU, or empty if they cannot be.
    [[nodiscard]] std::optional<mola::Georeferencing> georef_for_enu_poses_locked() const;
    /// Geo-reference from the current graph state (datum + T_enu_to_map).
    [[nodiscard]] std::optional<mola::Georeferencing> compute_geo_reference_locked() const;

    /// Solved antenna position in {map} at time t: the keyframe closest in time
    /// (within maxDt), moved by its twist to t. With solvedOnly, keyframes not
    /// solved yet (seeded with a neighbor's pose) are skipped.
    [[nodiscard]] std::optional<mrpt::math::TPoint3D> predicted_antenna_in_map_locked(
        const mrpt::Clock::time_point& t, const mrpt::math::TPoint3D& lever, double maxDt,
        bool solvedOnly);

    /// Admission tests and innovation gate of a GNSS fix (see Parameters.h).
    /// Returns false if it must not be fused. Updates state_.gnssGate.
    [[nodiscard]] bool gnss_gate_locked(
        const mrpt::obs::CObservationGPS& gps, const mrpt::topography::TGeodeticCoords& geo,
        const std::array<double, 3>& enuVarFloored);

    [[nodiscard]] std::optional<mrpt::poses::CPose3DPDFGaussian> estimated_T_enu_to_map_locked()
        const;

    /// Adds new factors to the smoother, optimizes it, and saves the variable values into
    /// state_.last_estimated_state
    void process_pending_gtsam_updates();

    /** Sigma [rad/s] for the constant-angular-velocity factor spanning `dt` seconds.
     *  Combines the random-walk model with the gyro measurement noise present at both
     *  endpoints when raw gyro readings are fused (see the implementation). */
    double angular_const_vel_sigma(double dt) const;

    /// Implementation of Eqs (1),(4) in the MOLA RSS2019 paper.
    void addFactor(const AbsFactorConstVelKinematics& f);
    void addFactor(const AbsFactorTricycleKinematics& f);

    /// Delete out-of-window entries in stamp2frame_index and last_estimated_state
    void delete_too_old_entries();

    void publishEstimatedGeoreferencing();

    /// Advertises the REP-105 correction reference_frame -> {odom} from the
    /// estimator's own T_map_to_odom_i (see publish_map_to_odom_tf).
    void publishMapToOdom(const mrpt::Clock::time_point& stamp, const std::string& methodLabel);

    /// Advertises one `reference_frame -> childFrame` correction (see publishMapToOdom).
    void advertiseMapToOdom(
        const mrpt::Clock::time_point& stamp, const std::string& methodLabel,
        const std::string& childFrame, const mrpt::poses::CPose3DPDFGaussian& T_map_to_odom);

    using pair_nearby_frame_iterators_t = std::pair<
        std::map<mrpt::Clock::time_point, frame_index_t>::const_iterator,
        std::map<mrpt::Clock::time_point, frame_index_t>::const_iterator>;

    [[nodiscard]] pair_nearby_frame_iterators_t find_before_after(
        const mrpt::Clock::time_point& t, bool allow_exact_match);

    void initialize_new_frame(frame_index_t id, const pair_nearby_frame_iterators_t& closestFrames);

    [[nodiscard]] std::optional<frame_index_t> pick_closest(
        const pair_nearby_frame_iterators_t& closestFrames,
        const mrpt::Clock::time_point&       stamp) const;

    void add_kinematic_factor_between(const frame_index_t from, const frame_index_t to);

    /** Withdraws the kinematic link between two keyframes, undoing
     *  add_kinematic_factor_between(). Needed when a new keyframe is spliced
     *  between them: the direct link they used to share must go away, or its
     *  constraint would be counted a second time alongside the two links that now
     *  replace it. Cheap if the link has not reached the solver yet; otherwise its
     *  factors are queued for removal at the next update().
     */
    /// IMU preintegration: keeps the raw readings of the IMU in use.
    void buffer_raw_imu_locked(const mrpt::obs::CObservationIMU& imu);

    /// IMU preintegration: before T_enu_to_map reaches the solver, sets its
    /// roll and pitch from the accelerometer, so a tilted {map} starts right.
    void level_enu_to_map_locked();

    /// Whether the first keyframe is linked to the {map} origin
    /// (link_first_pose_to_reference_origin_sigma), which only holds until a
    /// pose has defined {map}.
    bool link_first_pose_to_origin() const;
    /// Pins {map} to the first odometry frame, if nothing anchors it yet.
    void anchor_map_to_first_odom_frame_locked();

    /// IMU preintegration: adds the factor of every link whose interval the
    /// raw readings now cover.
    void attach_imu_factors_locked();

    void remove_kinematic_factor_between(const frame_index_t from, const frame_index_t to);

    /// Gets the latest state of a pose wrt the reference frame ("map")
    [[nodiscard]] NavState get_latest_state_and_covariance(const frame_index_t idx) const;

    /// Updates the low-pass-filtered predict twist with the newest keyframe's
    /// optimized twist (dt-aware exponential moving average). No-op unless
    /// `predict_twist_filter_enabled`. See `State::filtered_predict_twist`.
    /// Compares a new reading of an odometry source against what was predicted
    /// for it, for predict_covariance_from_residuals. Call before storing it as
    /// the source's last raw pose.
    void update_prediction_residuals_locked(
        odometry_frameid_t src, const mrpt::Clock::time_point& t, const mrpt::poses::CPose3D& pose);

    void update_predict_twist_filter_locked(
        const mrpt::math::TTwist3D& rawTwist, const mrpt::Clock::time_point& stamp);

    /// Gets the latest estimated transform of "T_map_to_odometry_frame_i", by frame ID name.
    [[nodiscard]] std::optional<mrpt::poses::CPose3DPDFGaussian>
        get_estimated_T_map_to_odometry_frame(const frame_index_t idx) const;

    /// Fills in the covariance of odometry frame `idx` in `last_estimated_frames`
    /// if the latest solve left it pending. Call before reading it. Returns false
    /// if its marginal cannot be computed (an under-constrained graph).
    [[nodiscard]] bool complete_odom_frame_covariance_locked(odometry_frameid_t idx) const;

    /** Abstract representation of a constant-velocity kinematic motion model factor
     * between two key frames.
     */
    class AbsFactorConstVelKinematics
    {
       public:
        AbsFactorConstVelKinematics() = default;

        /** Creates relative pose constraint of KF `to` as seem from `from`. */
        AbsFactorConstVelKinematics(id_t kf_from, id_t kf_to, double delta_time)  // NOLINT
            : from_kf(kf_from), to_kf(kf_to), deltaTime(delta_time)
        {
        }

        id_t from_kf = INVALID_ID, to_kf = INVALID_ID;

        /** Elapsed time between "from_kf" and "to_kf" [seconds] */
        double deltaTime = .0;
    };

    /** Abstract representation of a constant-velocity tricycle kinematic motion
     * model factor between two key frames.
     */
    class AbsFactorTricycleKinematics
    {
       public:
        AbsFactorTricycleKinematics() = default;

        /** Creates relative pose constraint of KF `to` as seem from `from`. */
        AbsFactorTricycleKinematics(id_t kf_from, id_t kf_to, double delta_time)  // NOLINT
            : from_kf(kf_from), to_kf(kf_to), deltaTime(delta_time)
        {
        }

        id_t from_kf = INVALID_ID, to_kf = INVALID_ID;

        /** Elapsed time between "from_kf" and "to_kf" [seconds] */
        double deltaTime = .0;
    };

    VizInterface::Ptr visualizer_;
};

}  // namespace mola::state_estimation_smoother
