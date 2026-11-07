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
 * @file   StateEstimationSmoother.cpp
 * @brief  Fuse of odometry, IMU, and SE(3) pose/twist estimations.
 * @author Jose Luis Blanco Claraco
 * @date   Jan 22, 2024
 */

// MOLA & MRPT:
#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/core/get_env.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/math/gtsam_wrappers.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CObservationRobotPose.h>
#include <mrpt/poses/Lie/SO.h>
#include <mrpt/poses/gtsam_wrappers.h>
#include <mrpt/topography/conversions.h>

// GTSAM:
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/expressions.h>
// IncrementalFixedLagSmoother moved from gtsam_unstable to gtsam in newer
// releases; prefer the stable header when available to avoid a deprecation
// warning, falling back to the unstable one for older GTSAM versions.
#if __has_include(<gtsam/nonlinear/IncrementalFixedLagSmoother.h>)
#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
#else
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#endif

// Custom factors:
#include <gtsam/navigation/ImuBias.h>
#include <mola_gtsam_factors/FactorAngularVelocityIntegration.h>
#include <mola_gtsam_factors/FactorConstLocalVelocity.h>
#include <mola_gtsam_factors/FactorGnssMapEnu.h>
#include <mola_gtsam_factors/FactorGnssMapEnuConstVel.h>
#include <mola_gtsam_factors/FactorImuPreintegrationBodyVel.h>
#include <mola_gtsam_factors/FactorTrapezoidalIntegrator.h>
#include <mola_gtsam_factors/FactorTricycleKinematic.h>
#include <mola_gtsam_factors/MeasuredGravityFactor.h>
#include <mola_gtsam_factors/Pose3RotationFactor.h>
#include <mola_gtsam_factors/Pose3RotationYawBiasFactor.h>
#include <mola_gtsam_factors/imu_helpers.h>
#include <mola_gtsam_factors/mrpt_gtsam_compat.h>

#include <deque>
#include <regex>
#include <set>

#if __has_include(<mola_imu_preintegration/ImuAverager.h>)
#include <mola_imu_preintegration/ImuAverager.h>
/** Feature macro: mola_imu_preintegration provides mola::imu::ImuAverager, used
 *  to average IMU readings when decimating them (imu_min_sample_period). Older
 *  releases fall back to keeping one raw reading per period. */
#define MOLA_SMOOTHER_HAS_IMU_AVERAGER 1
#endif

#include "FastPredictor.h"
#include "Snapshot.h"
#include "extrapolation.h"

// std:
#include <algorithm>
#include <cctype>
#include <chrono>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

// arguments: class_name, parent_class, class namespace
IMPLEMENTS_MRPT_OBJECT(
    StateEstimationSmoother, mola::ExecutableBase, mola::state_estimation_smoother)

struct mola::state_estimation_smoother::StateEstimationSmoother::ImuDecimator
{
    /// Returns the reading to fuse for this input, if any. Each sensor label is
    /// decimated on its own: readings from IMUs mounted differently must never
    /// be averaged together.
    std::optional<mrpt::obs::CObservationIMU> add(
        const mrpt::obs::CObservationIMU& imu, double period)
    {
        auto& s = bySensor[imu.sensorLabel];
#if defined(MOLA_SMOOTHER_HAS_IMU_AVERAGER)
        return s.averager.add(imu, period);
#else
        if (period > 0 && s.lastStamp.has_value() &&
            mrpt::system::timeDifference(*s.lastStamp, imu.timestamp) < period)
        {
            return std::nullopt;
        }
        s.lastStamp = imu.timestamp;
        return imu;
#endif
    }

    struct PerSensor
    {
#if defined(MOLA_SMOOTHER_HAS_IMU_AVERAGER)
        mola::imu::ImuAverager averager;
#else
        std::optional<mrpt::Clock::time_point> lastStamp;
#endif
    };
    std::map<std::string, PerSensor> bySensor;
};

namespace
{
constexpr double ENU2MAP_WEAK_SIGMA          = 1e4;
constexpr double INIT_ODOM_FRAME_POSE_SIGMA  = 1e3;
constexpr double FIRST_POSE_WEAK_PRIOR_SIGMA = 1e6;
constexpr double GAUGE_ANCHOR_SIGMA          = 1e-3;
// T_enu_to_map angles with a larger sigma [rad] are unobserved (only their weak prior):
constexpr double UNOBSERVED_ENU2MAP_ANGLE_SIGMA = 1.0;
constexpr double PLANAR_XY_SIGMA                = 1e10;
constexpr double PLANAR_Z_SIGMA                 = 1e-4;
constexpr double TRICYCLE_LARGE_SIGMAS          = 1e6;

// Appended to a wheel odometry's label when a pose source uses a frame of that name.
constexpr const char* WHEELS_ODOMETRY_FRAME_SUFFIX = "_wheels";

// A frame name for a wheel odometry labeled `label` that no frame uses yet:
// "<label>_wheels", or with a counter after it if that is taken too.
template <class FRAMES_MAP>
std::string unused_wheels_frame_name(
    const FRAMES_MAP& frames, const std::string& label, const std::string& referenceFrame)
{
    const std::string base = label + WHEELS_ODOMETRY_FRAME_SUFFIX;
    std::string       name = base;
    for (int i = 2; frames.hasKey(name) || name == referenceFrame; i++)
    {
        name = base + std::to_string(i);
    }
    return name;
}

/// Huber threshold [whitened units] for raw gyro observations; the standard
/// value giving ~95% efficiency under Gaussian noise.
constexpr double GYRO_HUBER_K = 1.345;

// Accelerometer norm above which a reading is taken to be in m/s^2. Readings
// normalized to g stay well below it, so terms in m/s^2 must not touch them.
constexpr double ACC_NORM_MIN_FOR_SI_UNITS = 3.0;

void enforce_planar_pose(mrpt::poses::CPose3D& p)
{
    p.z(0);
    p.setYawPitchRoll(p.yaw(), .0, .0);
}
void enforce_planar_twist(mrpt::math::TTwist3D& tw)
{
    tw.vz = 0;
    tw.wx = 0;
    tw.wy = 0;
}

// Seconds on a monotone, always-positive axis (the clock's own tick origin),
// for the GTSAM fixed-lag smoother's key timestamps.
//
// NOT mrpt::Clock::toDouble(): that rebases onto the UNIX epoch with an
// UNSIGNED subtraction, so a timestamp even a millisecond before it reads
// ~1.8e12 s instead of a small negative number. Datasets whose clock starts at
// zero do produce such timestamps, and one of them among the key stamps moves
// the smoother's notion of "now" 58000 years into the future, marginalizing
// every genuine keyframe on arrival. Only differences of these values matter
// to GTSAM, so the origin is free; being consistent is what is not.
double key_stamp_seconds(const mrpt::Clock::time_point& t)
{
    return std::chrono::duration<double>(t.time_since_epoch()).count();
}

// Debug instrumentation: dumps every estimated_navstate() result (the pose +
// covariance that LIO uses as ICP initial guess AND prior, plus the returned
// twist) so the prior-vs-data weighting and the velocity feedback can be
// analyzed offline. Same file format and same env var as the lightweight
// estimator's, so one script reads both. Disabled unless MOLA_NAVSTATE_DUMP is
// set to a path.
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

// One row of the CSV above. `dt` is the extrapolation interval from the anchor
// keyframe; a row is written only when a state was actually produced, so a gap
// in the file marks a scan the front end ran with no motion model at all.
void navstate_dump_row(
    const mrpt::Clock::time_point& timestamp, double dt, const mola::NavState& ns)
{
    std::ofstream* st = navstate_dump_stream();
    if (!st)
    {
        return;
    }

    // estimated_navstate() is callable concurrently and, in async mode, holds no
    // lock of its own, so two queries could otherwise interleave halves of a row.
    static std::mutex           s_dumpMutex;
    std::lock_guard<std::mutex> lck(s_dumpMutex);

    const auto& m = ns.pose.mean;
    (*st) << mrpt::format("%.6f", mrpt::Clock::toDouble(timestamp)) << "," << dt << ","  //
          << m.x() << "," << m.y() << "," << m.z() << "," << m.yaw() << "," << m.pitch() << ","
          << m.roll();

    const auto& tw = ns.twist;
    (*st) << "," << tw.vx << "," << tw.vy << "," << tw.vz << "," << tw.wx << "," << tw.wy << ","
          << tw.wz;

    const mrpt::math::CMatrixDouble66 cov = ns.pose.cov_inv.inverse_LLt();
    for (int i = 0; i < 6; i++)
    {
        (*st) << "," << cov(i, i);
    }
    for (int i = 0; i < 6; i++)
    {
        (*st) << "," << ns.pose.cov_inv(i, i);
    }
    (*st) << "\n";
    st->flush();
}

/// Wraps a pose factor's Gaussian noise model in a Huber m-estimator when
/// `threshold` (in whitened units) is > 0. Huber is convex, so unlike a
/// redescending kernel it cannot switch off a correct measurement whose
/// variable starts far from it, as a new keyframe seeded at its neighbor's
/// pose does.
gtsam::SharedNoiseModel with_huber(const gtsam::SharedNoiseModel& base, double threshold)
{
    if (threshold <= 0)
    {
        return base;
    }
    return gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(threshold), base);
}

/// Wheel-odometry increment with the uncertainty of the configured
/// probabilistic motion model.
mrpt::poses::CPose3DPDFGaussian odometry_increment_pdf(
    const mola::state_estimation_smoother::Parameters& p,
    const mrpt::poses::CPose2D&                        odometryIncrement)
{
    mrpt::obs::CActionRobotMovement2D odoAct;
    auto&                             mm = odoAct.motionModelConfiguration;
    mm.modelSelection                    = mrpt::obs::CActionRobotMovement2D::mmGaussian;
    mm.gaussianModel.a1                  = p.odom_motion_model_a1;
    mm.gaussianModel.a2                  = p.odom_motion_model_a2;
    mm.gaussianModel.a3                  = p.odom_motion_model_a3;
    mm.gaussianModel.a4                  = p.odom_motion_model_a4;
    mm.gaussianModel.minStdXY            = p.odom_motion_model_min_std_xy;
    mm.gaussianModel.minStdPHI           = mrpt::DEG2RAD(p.odom_motion_model_min_std_phi_deg);

    odoAct.computeFromOdometry(odometryIncrement, mm);

    mrpt::poses::CPose3DPDFGaussian incrementPdf;
    incrementPdf.copyFrom(*odoAct.poseChange);
    // Ensure as minimal uncertainty in all 3D DOFs to prevent numerical issues:
    incrementPdf.cov.asEigen().diagonal().array() += 1e-4;
    // ...plus the configured one in those a planar odometry does not measure
    // (MRPT order: x y z yaw pitch roll):
    incrementPdf.cov(2, 2) += mrpt::square(p.odom_out_of_plane_std_z);
    incrementPdf.cov(4, 4) += mrpt::square(p.odom_out_of_plane_std_roll_pitch);
    incrementPdf.cov(5, 5) += mrpt::square(p.odom_out_of_plane_std_roll_pitch);
    return incrementPdf;
}

gtsam::noiseModel::mEstimator::Base::shared_ptr make_gnss_kernel(
    const std::string& name, double scale)
{
    using namespace gtsam::noiseModel::mEstimator;
    if (name == "cauchy")
    {
        return Cauchy::Create(scale);
    }
    if (name == "geman_mcclure")
    {
        return GemanMcClure::Create(scale);
    }
    if (name == "dcs")
    {
        return DCS::Create(scale);
    }
    return Huber::Create(scale);
}

// Accepted GGA qualities from a "4 5" or "4,5" list; empty = any.
std::set<int> parse_fix_qualities(const std::string& s)
{
    std::set<int> ret;
    std::string   tok;
    for (const char c : s + " ")
    {
        if (c == ' ' || c == ',' || c == '\t')
        {
            if (!tok.empty())
            {
                ret.insert(std::stoi(tok));
                tok.clear();
            }
            continue;
        }
        tok += c;
    }
    return ret;
}

}  // namespace

namespace mola::state_estimation_smoother
{

const bool   NAVSTATE_PRINT_FG        = mrpt::get_env<bool>("NAVSTATE_PRINT_FG", false);
const bool   NAVSTATE_PRINT_FG_ERRORS = mrpt::get_env<bool>("NAVSTATE_PRINT_FG_ERRORS", false);
const double NAVSTATE_PRINT_FG_ERRORS_THRESHOLD =
    mrpt::get_env<double>("NAVSTATE_PRINT_FG_ERRORS_THRESHOLD", 0.1);

using gtsam::symbol_shorthand::F;  // Frame of references (Pose3)
                                   // F(0): T_enu_to_map
                                   // F(i): T_map_to_odometry_frame_i
const auto symbol_T_enu_to_map         = F(0);
const auto symbol_T_map_to_odom_i_base = F(0);  // odom[i] = thisSymbol + i (with i>=1)

using gtsam::symbol_shorthand::B;  // IMU attitude yaw bias [rad]   (double)
using gtsam::symbol_shorthand::T;  // Poses                          (Pose3)
using gtsam::symbol_shorthand::V;  // Lin velocity (body frame)      (Point3)
using gtsam::symbol_shorthand::W;  // Ang velocity (body frame)      (Point3)
using gtsam::symbol_shorthand::Z;  // IMU biases (imuBias::ConstantBias), with imu_preintegration
//  TODO: IMU bias

constexpr unsigned int REFERENCE_FRAME_ID = 0;  // (for symbol_T_enu_to_map)

// -------- GtsamImpl -------

// everything related to gtsam is hidden in the public API via pimpl
// to reduce compilation dependencies, and build time and memory usage.
struct StateEstimationSmoother::GtsamImpl
{
    GtsamImpl() = default;

    /// Identifies the kinematic link between two keyframes. Links are undirected,
    /// so the pair is normalized and (a,b) and (b,a) name the same link.
    using frame_pair_t = std::pair<frame_index_t, frame_index_t>;

    static frame_pair_t link_key(const frame_index_t a, const frame_index_t b)
    {
        return {std::min(a, b), std::max(a, b)};
    }

    // This is initialized in initialize(), once we have the parameters
    std::optional<gtsam::IncrementalFixedLagSmoother> smoother;

    // Queue of pending updates for incremental iSAM2:
    gtsam::NonlinearFactorGraph              newFactors;
    gtsam::Values                            newValues;
    gtsam::FixedLagSmoother::KeyTimestampMap newKeyStamps;

    /** Kinematic links built but not yet handed to iSAM2, keyed by the keyframe
     *  pair they connect. Keeping them keyed, instead of appending straight into
     *  newFactors, is what allows a link to be withdrawn again before it ever
     *  reaches the solver, when a keyframe gets spliced between its endpoints.
     */
    std::map<frame_pair_t, gtsam::NonlinearFactorGraph> pendingKinematic;

    /** iSAM2 factor indices of the kinematic links already inside the smoother,
     *  so a link can be removed on a later splice. */
    std::map<frame_pair_t, gtsam::FactorIndices> flushedKinematic;

    /** Queued for removal at the next update(). */
    gtsam::FactorIndices factorsToRemove;

    /** A strong prior pinning a gauge freedom: a direction of the state that no
     *  measurement observes yet, e.g. where {map} is before any source gives
     *  poses in it. A weak prior would leave the system nearly singular, which
     *  some GTSAM versions reject during Cholesky elimination. Anchors are only
     *  placed on variables that are never marginalized, so they can always be
     *  withdrawn once measurements start observing that direction.
     */
    struct GaugeAnchor
    {
        std::optional<size_t>             pendingIndex;  //!< Position in newFactors
        std::optional<gtsam::FactorIndex> isamIndex;  //!< Once inside the smoother
    };

    /// Defines {map} as odometry frame `mapAnchorFrame` while no source observes {map}
    std::optional<GaugeAnchor> mapAnchor;
    odometry_frameid_t         mapAnchorFrame = 0;
    bool                       mapObserved    = false;
    /// mapObserved, as of the last solve (the observation is in the estimate)
    bool mapObservedSolved = false;
    /// Whether some source observes the position of {map} (GNSS through a
    /// fixed geo-reference, or a pose in {map}), not just its orientation
    /// (absolute attitude), and its value as of the last solve:
    bool mapPositionObserved       = false;
    bool mapPositionObservedSolved = false;

    /// Pins the azimuth of T_enu_to_map while no IMU attitude or GNSS observes it
    std::optional<GaugeAnchor>   enuYawAnchor;
    std::optional<gtsam::Point2> firstGnssEnuXY;
    /// Where the vehicle had the GNSS antenna in {map} at that first fix
    std::optional<gtsam::Point2> firstGnssMapXY;

    /** Once {map} is observed only in position (e.g. GNSS through a fixed
     *  geo-reference, with no attitude source), pins the orientation of the
     *  odometry frame `orientationAnchorKey`, which nothing observes yet.
     *  Roll/pitch and yaw are released independently, as sources observe them.
     */
    std::optional<GaugeAnchor>   orientationAnchor;
    std::optional<gtsam::Key>    orientationAnchorKey;
    bool                         rollPitchObserved = false;
    bool                         yawObserved       = false;
    std::optional<gtsam::Point2> firstGnssXYForYaw;

    /// Tail of the chain of IMU attitude yaw bias variables B(k), and its key stamp
    std::optional<frame_index_t> lastYawBiasKf;
    double                       lastYawBiasStamp = 0;

    /// Latest solved IMU attitude yaw bias: mean and sigma [rad], and the key stamp of
    /// its last sigma update
    std::optional<std::pair<double, double>> yawBiasEstimate;
    double                                   yawBiasSigmaStamp = 0;

    /// Whether the reported orientation is pinned by a gauge anchor, not observed
    [[nodiscard]] bool rollPitchPinned() const { return orientationAnchor && !rollPitchObserved; }
    [[nodiscard]] bool yawPinned() const { return orientationAnchor && !yawObserved; }

    /// Re-creates the orientation anchor for what is still unobserved
    void update_orientation_anchor()
    {
        withdraw_anchor(orientationAnchor);
        if (!orientationAnchorKey || (rollPitchObserved && yawObserved))
        {
            return;
        }
        // Pose3 tangent: rotation (x, y, z), then translation.
        const double   rp = rollPitchObserved ? INIT_ODOM_FRAME_POSE_SIGMA : GAUGE_ANCHOR_SIGMA;
        const double   y  = yawObserved ? INIT_ODOM_FRAME_POSE_SIGMA : GAUGE_ANCHOR_SIGMA;
        gtsam::Vector6 sigmas;
        sigmas << rp, rp, y, INIT_ODOM_FRAME_POSE_SIGMA, INIT_ODOM_FRAME_POSE_SIGMA,
            INIT_ODOM_FRAME_POSE_SIGMA;
        add_anchor<gtsam::PriorFactor<gtsam::Pose3>>(
            orientationAnchor, *orientationAnchorKey, gtsam::Pose3::Identity(),
            gtsam::noiseModel::Diagonal::Sigmas(sigmas));
    }

    /// Queues a new anchor factor, built in place from `args`
    template <class FACTOR, class... Args>
    void add_anchor(std::optional<GaugeAnchor>& anchor, Args&&... args)
    {
        anchor = GaugeAnchor{newFactors.size(), std::nullopt};
        newFactors.emplace_shared<FACTOR>(std::forward<Args>(args)...);
    }

    /// Drops the anchor, whether still pending or already inside the smoother
    void withdraw_anchor(std::optional<GaugeAnchor>& anchor)
    {
        if (!anchor)
        {
            return;
        }
        if (anchor->pendingIndex)
        {
            const size_t idx = *anchor->pendingIndex;
            newFactors.erase(newFactors.begin() + static_cast<std::ptrdiff_t>(idx));
            for (auto* other : {&mapAnchor, &enuYawAnchor, &orientationAnchor})
            {
                if (*other && (*other)->pendingIndex && *(*other)->pendingIndex > idx)
                {
                    --*(*other)->pendingIndex;
                }
            }
        }
        else if (anchor->isamIndex)
        {
            factorsToRemove.push_back(*anchor->isamIndex);
        }
        anchor.reset();
    }

#if !GTSAM_USES_BOOST
    // ---- IMU preintegration (Parameters::imu_preintegration) ----
    struct RawImu
    {
        double         t = 0;
        gtsam::Vector3 acc;
        gtsam::Vector3 gyr;
    };
    std::deque<RawImu>                           rawImu;  //!< oldest first
    std::string                                  rawImuLabel;  //!< the IMU in use
    std::optional<std::regex>                    rawImuLabelRe;
    std::shared_ptr<gtsam::PreintegrationParams> pimParams;
    std::set<frame_pair_t>                       imuLinksAwaiting;
    gtsam::imuBias::ConstantBias                 lastBias;
    double                                       lastBiasStamp = 0;
#endif

    /// Records the iSAM2 index of anchors passed in the last update()
    void on_update_done(const gtsam::FactorIndices& newFactorsIndices)
    {
        for (auto* anchor : {&mapAnchor, &enuYawAnchor, &orientationAnchor})
        {
            if (*anchor && (*anchor)->pendingIndex)
            {
                (*anchor)->isamIndex = newFactorsIndices.at(*(*anchor)->pendingIndex);
                (*anchor)->pendingIndex.reset();
            }
        }
    }
};

// -------- StateEstimationSmoother::State -------
StateEstimationSmoother::State::State()
    : gtsam(mrpt::make_impl<StateEstimationSmoother::GtsamImpl>())
{
}

// -------- StateEstimationSmoother -------
StateEstimationSmoother::StateEstimationSmoother()
{  //
    profiler_.setName("StateEstimationSmoother");
    ExecutableBase::setModuleInstanceName("StateEstimationSmoother");
    fastPredictor_ = std::make_unique<FastPredictor>();
}

StateEstimationSmoother::~StateEstimationSmoother()
{
    // Join the backend thread before any member it touches is destroyed.
    stop_backend_thread();

    if (!params_.save_final_georef_to_file.empty())
    {
        try
        {
            if (!save_geo_reference_to_file(params_.save_final_georef_to_file))
            {
                std::cerr << "[StateEstimationSmoother] No geo-reference written to '"
                          << params_.save_final_georef_to_file
                          << "': none available or the file could not be written.\n";
            }
        }
        catch (const std::exception& e)
        {
            std::cerr << "[StateEstimationSmoother] Error saving the geo-reference: " << e.what()
                      << "\n";
        }
    }
}

void StateEstimationSmoother::initialize(const mrpt::containers::yaml& cfg)
{
    // Initialize parent:
    mola::NavStateFilter::initialize(cfg);

    this->mrpt::system::COutputLogger::setLoggerName("StateEstimationSmoother");

    MRPT_LOG_DEBUG_STREAM("initialize() called with:\n" << cfg << "\n");
    ENSURE_YAML_ENTRY_EXISTS(cfg, "params");

    auto lck = mrpt::lockHelper(stateMutex_);

    // This also resets the GTSAM pimpl unique_ptr in state_
    reset_locked();

    // Load params:
    params_.loadFrom(cfg["params"]);

    if (params_.imu_bias_tracking)
    {
#if defined(MOLA_SMOOTHER_HAS_IMU_BIAS_SOURCE)
        auto lckBias = mrpt::lockHelper(imuBiasMutex_);
        imuBiasTracker_.reset();
        imuBiasTracker_.parameters.load_from(params_.imu_bias_tracker);
        imuBiasHintRefs_.clear();
        imuBiasUpdatedLabels_.clear();
        try
        {
            (void)imuBiasLabelsRe_.get_regex(params_.imu_bias_labels_re);
        }
        catch (const std::regex_error& e)
        {
            THROW_EXCEPTION_FMT(
                "Invalid imu_bias_labels_re '%s': %s", params_.imu_bias_labels_re.c_str(),
                e.what());
        }
#else
        MRPT_LOG_WARN(
            "imu_bias_tracking ignored: built against mola_kernel or mola_imu_preintegration "
            "versions without ImuBiasSource / ImuBiasTracker.");
#endif
    }

    if (params_.imu_preintegration.enabled)
    {
        try
        {
            (void)std::regex(params_.imu_preintegration.sensor_label_re);
        }
        catch (const std::regex_error& e)
        {
            THROW_EXCEPTION_FMT(
                "Invalid imu_preintegration.sensor_label_re '%s': %s",
                params_.imu_preintegration.sensor_label_re.c_str(), e.what());
        }
    }

#if !defined(MOLA_SMOOTHER_HAS_IMU_AVERAGER)
    if (params_.imu_min_sample_period > 0)
    {
        MRPT_LOG_WARN(
            "Built against a mola_imu_preintegration without ImuAverager: "
            "imu_min_sample_period keeps one raw IMU reading per period instead of "
            "averaging them, so platform vibration may alias into the IMU factors.");
    }
#endif

    if (auto vizMods = ExecutableBase::findService<mola::VizInterface>(); !vizMods.empty())
    {
        visualizer_ = std::dynamic_pointer_cast<mola::VizInterface>(*vizMods.begin());
        if (visualizer_)
        {
            MRPT_LOG_DEBUG_STREAM("Connected to visualizer module");
        }
    }

    if (visualizer_)
    {
        this->mrpt::system::COutputLogger::logRegisterCallback(
            [&](std::string_view msg, const mrpt::system::VerbosityLevel level,
                std::string_view loggerName, const mrpt::Clock::time_point timestamp)
            {
                using namespace std::string_literals;

                if (!params_.visualization.show_console_messages)
                {
                    return;
                }

                if (level < this->getMinLoggingLevel())
                {
                    return;
                }

                visualizer_->output_console_message(
                    "["s + mrpt::system::timeLocalToString(timestamp) + "|"s +
                    mrpt::typemeta::enum2str(level) + " |"s + std::string(loggerName) + "]"s +
                    std::string(msg));
            });
    }

    params_loaded_ = true;
    reinitialize_gtsam_locked();

    // In async mode, spin up the backend solver thread. It immediately waits on
    // an empty queue, so starting it while stateMutex_ is held is safe.
    start_backend_thread();
}

void StateEstimationSmoother::reinitialize_gtsam_locked()
{
    // Forward parameters to GTSAM smoother & iSAM2:
    gtsam::ISAM2Params isam2Params;
    isam2Params.findUnusedFactorSlots = true;  // Important, must be set for fixed-lag smoother
    // Poses are relinearized at a fine threshold: at the default 0.1, the
    // solution can stay away from the optimum by more than an accurate pose
    // source's own error. Velocities and biases keep the coarse one: they do
    // not need it, and their weakly observed directions (e.g. before the first
    // pose reading) make relinearizing them often numerically fragile.
    {
        const double poseThr   = params_.isam2_relinearize_threshold;
        const double coarseThr = 0.1;

        // Keyed by symbol character, which must cover every variable type:
        const auto chr = [](gtsam::Key k) { return gtsam::Symbol(k).chr(); };

        gtsam::FastMap<char, gtsam::Vector> thresholds;
        thresholds[chr(T(0))]            = gtsam::Vector6::Constant(poseThr);
        thresholds[chr(F(0))]            = gtsam::Vector6::Constant(poseThr);
        thresholds[chr(V(0))]            = gtsam::Vector3::Constant(coarseThr);
        thresholds[chr(W(0))]            = gtsam::Vector3::Constant(coarseThr);
        thresholds[chr(Z(0))]            = gtsam::Vector6::Constant(coarseThr);
        thresholds[chr(B(0))]            = gtsam::Vector1::Constant(coarseThr);
        isam2Params.relinearizeThreshold = thresholds;
    }
    isam2Params.relinearizeSkip = 1;
    // isam2Params.optimizationParams    = gtsam::ISAM2DoglegParams();

    state_.gtsam->smoother.emplace(params_.sliding_window_length, isam2Params);

    // Initialize georeference-related gtsam variables:
    // Even if not using a geo-referenced map, even if not using GPS sensors,
    // do define the T_enu_to_map transform variable, so it can be used for gravity-alignment
    // via IMU accelerometer, at least:
    auto           enu2map     = gtsam::Pose3::Identity();
    gtsam::Matrix6 enu2map_cov = gtsam::Matrix6::Identity() * mrpt::square(ENU2MAP_WEAK_SIGMA);

    if (params_.fixed_geo_reference.has_value())
    {
        state_.geo_reference = *params_.fixed_geo_reference;

        // A zero covariance (e.g. a default-constructed "exactly known" geo-reference)
        // is singular for GTSAM: floor it to the tiny variance used for YAML-defined ones.
        auto T_enu_to_map = state_.geo_reference->T_enu_to_map;
        for (int i = 0; i < 6; i++)
        {
            T_enu_to_map.cov(i, i) = std::max(T_enu_to_map.cov(i, i), 1e-6);
        }

        mola::gtsam_compat::to_gtsam_se3_cov6(T_enu_to_map, enu2map, enu2map_cov);

        // Update into last_estimated_frames too, so estimated_T_enu_to_map() returns it:
        state_.last_estimated_frames[REFERENCE_FRAME_ID] = state_.geo_reference->T_enu_to_map;
    }

    // Initial value:
    state_.gtsam->newValues.insert(symbol_T_enu_to_map, enu2map);
    // Weak prior factor:
    state_.gtsam->newFactors.addPrior(symbol_T_enu_to_map, enu2map, enu2map_cov);

    // Gravity alone observes its roll and pitch, never its azimuth:
    if (!params_.fixed_geo_reference.has_value())
    {
        gtsam::Vector6 sigmas;
        sigmas << ENU2MAP_WEAK_SIGMA, ENU2MAP_WEAK_SIGMA, GAUGE_ANCHOR_SIGMA, ENU2MAP_WEAK_SIGMA,
            ENU2MAP_WEAK_SIGMA, ENU2MAP_WEAK_SIGMA;
        state_.gtsam->add_anchor<gtsam::PriorFactor<gtsam::Pose3>>(
            state_.gtsam->enuYawAnchor, symbol_T_enu_to_map, enu2map,
            gtsam::noiseModel::Diagonal::Sigmas(sigmas));
    }
}

#if defined(MOLA_KERNEL_NAVSTATE_FILTER_HAS_GEO_REFERENCE)
void StateEstimationSmoother::set_geo_reference(const mola::Georeferencing& georef)
{
    auto lck = mrpt::lockHelper(stateMutex_);

    MRPT_LOG_INFO_STREAM(
        "[set_geo_reference] Anchoring T_enu_to_map to a fixed, known value: "
        << georef.T_enu_to_map.mean.asString());

    params_.fixed_geo_reference = georef;

    // reset_locked() clears state_ (including the gtsam pimpl's pending
    // newValues/newFactors/newKeyStamps buffers, which already hold a
    // symbol_T_enu_to_map entry from the load-params-time
    // reinitialize_gtsam_locked() call) before re-running it -- calling
    // reinitialize_gtsam_locked() directly here would try to insert that
    // same key a second time into the still-pending newValues and throw.
    reset_locked();
}
#endif

bool StateEstimationSmoother::reference_frame_position_observed()
{
    if (params_.async_backend)
    {
        const auto snap = fastPredictor_->snapshot();
        return snap && (!snap->geoReference.has_value() || snap->mapPositionObserved);
    }
    auto lck = mrpt::lockHelper(stateMutex_);
    if (!state_.geo_reference.has_value())
    {
        return true;
    }
    // The synchronous solve is lazy: run it, as estimated_navstate() would, but
    // only until the flag is set, so publishing does not solve twice.
    if (!state_.gtsam->mapPositionObservedSolved)
    {
        process_pending_gtsam_updates_locked();
    }
    return state_.gtsam->mapPositionObservedSolved;
}

void StateEstimationSmoother::spinOnce()
{
    const bool diagnosticsTime = module_is_time_to_publish_diagnostics();
    if (params_.publish_georef_convergence && params_.estimate_geo_reference && diagnosticsTime)
    {
        if (!params_.async_backend)
        {
            // The synchronous solve is lazy: run the pending one, so the values are
            // current even without localization subscribers.
            auto lck = mrpt::lockHelper(stateMutex_);
            process_pending_gtsam_updates_locked();
        }
        std::optional<GeorefConvergence> c;
        {
            std::lock_guard<std::mutex> lck(lastGeorefConvergenceMutex_);
            c = lastGeorefConvergence_;
        }
        const auto now = mrpt::Clock::now();
        module_publish_diagnostics(
            {now, "georef_max_pos_sigma_m", params_.convergence_max_position_sigma});
        module_publish_diagnostics(
            {now, "georef_max_ori_sigma_deg", params_.convergence_max_orientation_sigma_deg});
        module_publish_diagnostics({now, "georef_converged", c && c->converged ? 1.0 : 0.0});
        if (c)
        {
            module_publish_diagnostics({now, "georef_pos_sigma_m", c->pos_sigma});
            module_publish_diagnostics({now, "georef_ori_sigma_deg", c->ori_sigma_deg});
            module_publish_diagnostics(
                {now, "georef_azimuth_pinned", c->azimuth_pinned ? 1.0 : 0.0});
        }
    }

    if (params_.publish_imu_yaw_bias && diagnosticsTime)
    {
        if (const auto bias = estimated_imu_yaw_bias(); bias)
        {
            const auto now = mrpt::Clock::now();
            module_publish_diagnostics({now, "imu_yaw_bias_deg", mrpt::RAD2DEG(bias->first)});
            module_publish_diagnostics(
                {now, "imu_yaw_bias_sigma_deg", mrpt::RAD2DEG(bias->second)});
        }
    }

    advertise_imu_biases();
    if (params_.publish_imu_biases && diagnosticsTime)
    {
        publish_imu_bias_diagnostics();
    }

    // At the predefined module rate, publish the current estimation, if we have any subscriber:
    if (!anyUpdateLocalizationSubscriber())
    {
        return;
    }

    // Read the extrapolated stamp under the lock, then release before calling estimated_navstate()
    // to avoid recursive locking (estimated_navstate() acquires the mutex internally).
    std::optional<mrpt::Clock::time_point> tNowOpt;
    if (params_.async_backend)
    {
        // Lock-free: the fast predictor tracks the freshest observation stamp.
        tNowOpt = fastPredictor_->get_current_extrapolated_stamp();
    }
    else
    {
        auto lck = mrpt::lockHelper(stateMutex_);
        tNowOpt  = state_.get_current_extrapolated_stamp();
    }

    if (!tNowOpt)
    {
        MRPT_LOG_THROTTLE_WARN(5.0, "Cannot publish vehicle pose (no input data yet?)");
        return;
    }

    // With a fixed geo-reference (relocalization in a known map), the position
    // of {map} is only that of the first odometry frame until some source
    // observes it (a GNSS fix, or a pose in {map}; an absolute attitude only
    // observes its orientation): a pose in it would be the raw odometry one.
    // Do not publish it, nor map->odom, until then.
    if (!params_.estimate_geo_reference && !reference_frame_position_observed())
    {
        MRPT_LOG_THROTTLE_INFO(
            5.0,
            "Not publishing the vehicle pose yet: waiting for an observation of the "
            "geo-referenced reference frame (e.g. GNSS).");
        return;
    }

    const auto nv = estimated_navstate(*tNowOpt, params_.reference_frame_name);
    if (!nv)
    {
        MRPT_LOG_THROTTLE_WARN(5.0, "Cannot publish vehicle pose (stalled input data?)");
        return;
    }

    // Use just the YAML label part of the module instance name
    // (e.g. "state_estimation" from "FullClassName:state_estimation"),
    // since this string is used as a ROS topic prefix and filter key:
    const auto&       fullName = getModuleInstanceName();
    const auto        colonPos = fullName.rfind(':');
    const std::string methodLabel =
        (colonPos != std::string::npos) ? fullName.substr(colonPos + 1) : fullName;

    // Primary output: the fused vehicle pose in the reference ({map}) frame.
    LocalizationUpdate lu;
    lu.child_frame     = params_.vehicle_frame_name;
    lu.reference_frame = params_.reference_frame_name;
    lu.method          = methodLabel;
    lu.quality         = 1;
    lu.timestamp       = *tNowOpt;
    lu.pose            = nv->pose.getPoseMean().asTPose();
    lu.cov             = nv->pose.cov_inv.inverse();

    MRPT_LOG_DEBUG_FMT(
        "[spinOnce] Publishing timely pose estimate: t=%f pose=%s", mrpt::Clock::toDouble(*tNowOpt),
        lu.pose.asString().c_str());

    advertiseUpdatedLocalization(lu);

    // Optional REP-105 output: publish map->odom directly from the estimator's
    // own T_map_to_odom_i, so the bridge forwards it verbatim (no stale-odom
    // composition). Distinct method suffix so TF/odom source filters route it.
    if (params_.publish_map_to_odom_tf)
    {
        publishMapToOdom(*tNowOpt, methodLabel);
    }

    // Optional high-rate fused pose in a distinct child frame (never collides
    // with an odom->base_link chain on /tf).
    if (params_.publish_fused_vehicle_tf)
    {
        LocalizationUpdate fu = lu;
        fu.child_frame        = params_.fused_vehicle_frame_name;
        fu.method             = methodLabel + "/fused";
        advertiseUpdatedLocalization(fu);
    }
}

void StateEstimationSmoother::publishMapToOdom(
    const mrpt::Clock::time_point& stamp, const std::string& methodLabel)
{
    // Resolve which odometry SOURCE frame's T_map_to_odom to publish: the
    // configured one, or the single known source when unambiguous.
    const auto knownFrames = known_odometry_frame_ids();

    // Extra sources, each under its own frame_id:
    std::istringstream extraFrames(params_.map_to_odom_extra_frames);
    for (std::string frame; std::getline(extraFrames, frame, ',');)
    {
        std::istringstream words(frame);
        for (std::string f; words >> f;)
        {
            if (knownFrames.count(f) == 0)
            {
                continue;
            }
            if (const auto T = estimated_T_map_to_odometry_frame(f); T.has_value())
            {
                advertiseMapToOdom(stamp, methodLabel, f, *T);
            }
        }
    }

    const auto knownFramesList = [&knownFrames]()
    {
        std::string s;
        for (const auto& f : knownFrames)
        {
            s += (s.empty() ? "" : ", ") + f;
        }
        return s;
    };

    std::string odomFrame = params_.map_to_odom_frame_name;
    if (odomFrame.empty())
    {
        if (knownFrames.size() == 1)
        {
            odomFrame = *knownFrames.begin();
        }
        else
        {
            MRPT_LOG_THROTTLE_WARN_FMT(
                5.0,
                "[publishMapToOdom] map_to_odom_frame_name is empty and %zu odometry frames are "
                "known [%s]; cannot pick one. Set the parameter explicitly.",
                knownFrames.size(), knownFramesList().c_str());
            return;
        }
    }
    else if (knownFrames.count(odomFrame) == 0)
    {
        // Distinguish a misconfigured/unknown source from a known-but-unsolved
        // one: this is the silent failure mode that publishes NO map->odom and
        // leaves {map} disconnected from the odom /tf subtree. The source's
        // frame_id is its sensor label, which is often NOT the ROS odom frame.
        MRPT_LOG_THROTTLE_WARN_FMT(
            5.0,
            "[publishMapToOdom] map_to_odom_frame_name='%s' is not a known odometry source frame; "
            "known: [%s]. No map->odom will be published. Note the source frame is its sensor "
            "label (e.g. 'odom_wheels'), not necessarily the ROS odom /tf frame.",
            odomFrame.c_str(), knownFramesList().c_str());
        return;
    }

    const auto T_map_to_odom = estimated_T_map_to_odometry_frame(odomFrame);
    if (!T_map_to_odom.has_value())
    {
        // Known source, but no solved estimate yet: transient at startup.
        MRPT_LOG_THROTTLE_WARN_FMT(
            5.0,
            "[publishMapToOdom] odometry source '%s' is known but has no estimate yet (waiting for "
            "the first solve).",
            odomFrame.c_str());
        return;
    }

    // The published /tf child frame is independent of the source frame_id: it
    // must equal the REP-105 odom frame the external driver publishes
    // odom->base_link for, which is often not the source's sensor label.
    const std::string childFrame =
        params_.map_to_odom_child_frame.empty() ? odomFrame : params_.map_to_odom_child_frame;

    advertiseMapToOdom(stamp, methodLabel, childFrame, *T_map_to_odom);
}

void StateEstimationSmoother::advertiseMapToOdom(
    const mrpt::Clock::time_point& stamp, const std::string& methodLabel,
    const std::string& childFrame, const mrpt::poses::CPose3DPDFGaussian& T_map_to_odom)
{
    LocalizationUpdate lu;
    lu.child_frame     = childFrame;
    lu.reference_frame = params_.reference_frame_name;
    lu.method          = methodLabel + "/map_odom";
    lu.quality         = 1;
    lu.timestamp       = stamp;
    lu.pose            = T_map_to_odom.mean.asTPose();
    lu.cov             = T_map_to_odom.cov;

    advertiseUpdatedLocalization(lu);
}

void StateEstimationSmoother::reset()
{
    auto lck = mrpt::lockHelper(stateMutex_);
    reset_locked();
}

void StateEstimationSmoother::reset_locked()
{
    state_ = State();
    // The datum goes with the state, so ENU poses recorded so far may refer to
    // another one: keep only the {map} history.
    finalizedTrajectoryEnu_.clear();
    {
        std::lock_guard<std::mutex> lck(lastYawBiasMutex_);
        lastYawBias_.reset();
    }
    {
        std::lock_guard<std::mutex> lck(lastGeorefConvergenceMutex_);
        lastGeorefConvergence_.reset();
    }
    if (fastPredictor_)
    {
        fastPredictor_->clear();
    }
    // Drop any queued (and any already-detached) async work: it targets the old
    // graph and must not repopulate the fresh one (required across
    // set_geo_reference(), see coding guidelines).
    {
        std::lock_guard<std::mutex> lk(queueMutex_);
        ingestQueue_.clear();
        deferredCovs_.clear();
        resetEpoch_.fetch_add(1, std::memory_order_relaxed);
    }
    if (params_loaded_)
    {
        reinitialize_gtsam_locked();
    }
}

void StateEstimationSmoother::enqueue_async(
    const mrpt::Clock::time_point& stamp, std::function<void()> fn)
{
    // Safety bound: the backend coalesces (drains all pending each iteration), so
    // in normal operation the queue holds only one solve's worth of arrivals.
    // This cap only trips if the backend stalls; dropping the oldest keeps memory
    // bounded and is visible via the throttled warning.
    constexpr size_t kMaxIngestQueue = 10000;
    {
        std::lock_guard<std::mutex> lk(queueMutex_);
        if (ingestQueue_.size() >= kMaxIngestQueue)
        {
            ingestQueue_.pop_front();
            MRPT_LOG_THROTTLE_WARN(
                5.0,
                "[async] ingest queue full: backend not keeping up, dropping oldest measurement");
        }
        ingestQueue_.emplace_back(stamp, std::move(fn));
    }
    queueCv_.notify_one();
}

void StateEstimationSmoother::backend_loop()
{
    while (!backendStop_.load())
    {
        std::deque<std::pair<mrpt::Clock::time_point, std::function<void()>>> batch;
        std::vector<std::shared_future<mrpt::math::CMatrixDouble66>>          covs;
        uint64_t                                                              batchEpoch = 0;
        {
            std::unique_lock<std::mutex> lk(queueMutex_);
            queueCv_.wait(lk, [this] { return backendStop_.load() || !ingestQueue_.empty(); });
            if (backendStop_.load())
            {
                break;
            }
            batch.swap(ingestQueue_);
            covs.swap(deferredCovs_);
            batchEpoch = resetEpoch_.load(std::memory_order_relaxed);
        }

        // Covariances still being computed by their producer: wait for them
        // before taking the state lock, so nobody else is blocked meanwhile.
        for (const auto& c : covs)
        {
            c.wait();
        }

        // Apply in timestamp order, so most residual out-of-order arrivals become
        // in-order; cross-batch stragglers still splice, handled by the graph.
        std::stable_sort(
            batch.begin(), batch.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });

        auto lck = mrpt::lockHelper(stateMutex_);

        // If a reset happened after this batch was detached, it targets a graph
        // that no longer exists: discard it rather than corrupt the fresh one.
        if (resetEpoch_.load(std::memory_order_relaxed) != batchEpoch)
        {
            continue;
        }

        // A measurement callback or the solve can throw (e.g. an
        // under-constrained marginal). In the backend thread that would escape
        // to std::terminate, so contain it: log and rebuild the smoother.
        try
        {
            for (auto& [stamp, fn] : batch)
            {
                fn();
            }
            // Runs the solve and, in async mode, publishes a fresh snapshot to
            // the fast predictor.
            process_pending_gtsam_updates_locked();
        }
        catch (const std::exception& e)
        {
            MRPT_LOG_ERROR_STREAM(
                "[backend_loop] Exception while applying a batch; resetting smoother:\n"
                << e.what());
            reset_locked();
        }
    }
}

void StateEstimationSmoother::start_backend_thread()
{
    if (!params_.async_backend || backendRunning_)
    {
        return;
    }
    backendStop_.store(false);
    backendRunning_ = true;
    backendThread_  = std::thread(&StateEstimationSmoother::backend_loop, this);
}

void StateEstimationSmoother::stop_backend_thread()
{
    if (!backendRunning_)
    {
        return;
    }
    backendStop_.store(true);
    queueCv_.notify_all();
    if (backendThread_.joinable())
    {
        backendThread_.join();
    }
    backendRunning_ = false;
}

void StateEstimationSmoother::fuse_odometry(
    const mrpt::obs::CObservationOdometry& odom, const std::string& odomName)
{
    imu_bias_add_pose(odom.timestamp, mrpt::poses::CPose3D(odom.odometry), odomName);
    if (params_.async_backend)
    {
        fastPredictor_->note_observation_stamp(odom.timestamp);
        const auto odomCopy = odom;
        enqueue_async(
            odom.timestamp,
            [this, odomCopy, odomName] { fuse_odometry_locked(odomCopy, odomName); });
        return;
    }
    auto lck = mrpt::lockHelper(stateMutex_);
    fuse_odometry_locked(odom, odomName);
}

void StateEstimationSmoother::fuse_odometry_locked(
    const mrpt::obs::CObservationOdometry& odom, const std::string& odomName)
{
    // Integrates new wheels-based odometry observations into the estimator.
    //  This is a convenience method that internally ends up calling
    //  fuse_pose(), but computing the uncertainty of odometry increments
    //  according to a given motion model.

    mrpt::poses::CPose2D lastOdom;
    if (state_.last_wheels_odometry_name.has_value())
    {
        ASSERTMSG_(
            *state_.last_wheels_odometry_name == odomName,
            "More than one different 'odomName's received for wheels odometry!");

        ASSERT_(state_.last_wheels_odometry.has_value());
        lastOdom = *state_.last_wheels_odometry;

        // High-rate decimation/merge: if this reading arrives too soon after
        // the last *kept* one, drop it WITHOUT advancing the pose anchor
        // (last_wheels_odometry) nor the kept stamp. The next kept reading then
        // fuses the accumulated increment (lastOdom -> current) with its
        // accumulated motion-model covariance, merging several consecutive
        // odometry readings into a single keyframe/factor without losing motion.
        if (params_.odometry_min_sample_period > 0 && state_.last_wheels_odometry_stamp.has_value())
        {
            const double dt =
                mrpt::system::timeDifference(*state_.last_wheels_odometry_stamp, odom.timestamp);
            if (dt < params_.odometry_min_sample_period)
            {
                return;
            }
        }
    }
    else
    {
        // This is the first time we have wheels odometry.
        // Store the pose but skip factor creation: the increment is zero,
        // which would produce a stiff near-zero BetweenFactor.
        //
        // This reading also defines the origin of the accumulation below: at
        // the very first sample no dead reckoning has happened yet, so the
        // accumulated uncertainty is zero (T_map_to_odom absorbs wherever the
        // source happens to start).
        state_.last_wheels_odometry_name  = odomName;
        state_.last_wheels_odometry       = odom.odometry;
        state_.last_wheels_odometry_stamp = odom.timestamp;
        state_.wheels_odometry_accumulated.emplace();
        state_.wheels_odometry_accumulated->mean = mrpt::poses::CPose3D(odom.odometry);
        state_.wheels_odometry_accumulated->cov.setZero();
        return;
    }
    // Use a probabilistic motion model:
    const auto incrementPdf = odometry_increment_pdf(params_, odom.odometry - lastOdom);

    // Keep the absolute dead-reckoned pose in {odom_i} with its accumulated
    // covariance. Motion enters the graph only as relative increments (see
    // fuse_odometry_relative_locked()); this pose is used for the one factor
    // resolving T_map_to_odom_i and as the source's own-frame pose that
    // estimated_navstate() extrapolates from, and both need the uncertainty
    // of the whole dead-reckoning history, not that of one increment.
    ASSERT_(state_.wheels_odometry_accumulated.has_value());
    *state_.wheels_odometry_accumulated = *state_.wheels_odometry_accumulated + incrementPdf;

    // Take the mean from the source directly rather than from the composition:
    // they agree analytically, and this keeps a long run free of accumulated
    // round-off in a quantity the source reports exactly.
    state_.wheels_odometry_accumulated->mean = mrpt::poses::CPose3D(odom.odometry);

    const mrpt::poses::CPose3DPDFGaussian& newOdomPosePdf = *state_.wheels_odometry_accumulated;

    MRPT_LOG_DEBUG_FMT(
        "[fuse_odometry]: t=%f name=%s pose=%s poseChange=%s accum_sigma_xy=%.03f m "
        "accum_sigma_yaw=%.03f deg",
        mrpt::Clock::toDouble(odom.timestamp), odomName.c_str(), odom.odometry.asString().c_str(),
        incrementPdf.mean.asString().c_str(),
        std::sqrt(newOdomPosePdf.cov(0, 0) + newOdomPosePdf.cov(1, 1)),
        mrpt::RAD2DEG(std::sqrt(newOdomPosePdf.cov(5, 5))));

    // Save for next iteration (advance the anchor: this reading was kept):
    state_.last_wheels_odometry_name  = odomName;
    state_.last_wheels_odometry       = odom.odometry;
    state_.last_wheels_odometry_stamp = odom.timestamp;

    // Wheel odometry is always fused as relative increments: an absolute
    // dead-reckoned pose asserts that the whole odometry trajectory relates to
    // {map} by one rigid transform, which slip and scale errors break.
    fuse_odometry_relative_locked(odom, odomName, newOdomPosePdf);
}

// Relative formulation: the motion goes into BetweenFactors between consecutive
// odometry keyframes, and exactly ONE absolute pose-in-{odom_i} factor is ever
// added, on the first kept reading of each source, to resolve T_map_to_odom_i.
// It is the only formulation used for wheel odometry.
//
// One is all it takes, and it stays that way for the whole run. T_map_to_odom_i
// is a single rigid variable: given that anchor plus the relative chain, every
// later "keyframe k is at odom pose p_k" is already implied, up to the odometry
// drift in between. A second absolute factor observes the frame no better -- it
// re-injects the accumulated dead-reckoning error into the map poses.
//
// Nor does the anchor need renewing when its keyframe ages out. Keyframes leave
// through IncrementalFixedLagSmoother's MARGINALIZATION, not through
// factorsToRemove (which this class uses only for kinematic-link splicing), so
// the anchor's information survives as a linear marginal on T_map_to_odom_i
// after its keyframe is gone. Re-anchoring per window would therefore add
// information the graph has already kept: the same double count, only slower.
// T_map_to_odom_i itself is never marginalized -- its key timestamp is bumped to
// the newest observation on every update, so it stays inside the lag forever.
void StateEstimationSmoother::fuse_odometry_relative_locked(
    const mrpt::obs::CObservationOdometry& odom, const std::string& odomName,
    const mrpt::poses::CPose3DPDFGaussian& absolutePoseInOdom)
{
    const auto frame_id_idx = add_or_get_odom_frame_id(wheels_odometry_frame_locked(odomName));
    const auto this_kf_id   = create_or_get_keyframe_by_timestamp_locked(odom.timestamp);

    // Unconditional, and independent of which factor is added below: this is the
    // anchor estimated_navstate() extrapolates from when a front end asks for a
    // pose in the source's OWN frame, so it has to track every kept reading.
    update_prediction_residuals_locked(frame_id_idx, odom.timestamp, absolutePoseInOdom.mean);
    state_.last_raw_pose_by_source[frame_id_idx] =
        State::RawSourcePose{odom.timestamp, absolutePoseInOdom};

    if (!state_.wheels_odometry_anchor_kf.has_value())
    {
        gtsam::Pose3   pose_out;
        gtsam::Matrix6 cov_out;
        mola::gtsam_compat::to_gtsam_se3_cov6(absolutePoseInOdom, pose_out, cov_out);

        seed_odom_frame_locked(frame_id_idx, this_kf_id, absolutePoseInOdom.mean);
        state_.gtsam->newFactors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            symbol_T_map_to_odom_i_base + frame_id_idx, T(this_kf_id), pose_out,
            gtsam::noiseModel::Gaussian::Covariance(cov_out));

        state_.wheels_odometry_anchor_kf = this_kf_id;

        MRPT_LOG_DEBUG_FMT(
            "[fuse_odometry] odom frame '%s' anchored at KF %u", odomName.c_str(),
            static_cast<unsigned>(this_kf_id));
    }

    // Several readings can land on one keyframe. The first of them represents
    // that keyframe, both as the end of its incoming increment and as the start
    // of its outgoing one; the later ones only move the source further, so the
    // chain anchor is held and their motion goes into the next increment.
    const auto prev = state_.last_wheels_odometry_kf;
    if (prev.has_value() && *prev == this_kf_id)
    {
        return;
    }

    const auto prevOdom               = state_.last_wheels_odometry_at_kf;
    state_.last_wheels_odometry_kf    = this_kf_id;
    state_.last_wheels_odometry_at_kf = odom.odometry;

    if (!prev.has_value() || !prevOdom.has_value())
    {
        return;  // first reading of the chain
    }

    // The previous keyframe may have been marginalized out of the sliding window
    // (a long gap in the stream, or a reset in between). Unlike the anchor above,
    // whose information the marginalization keeps, a NEW factor naming a
    // marginalized variable would resurrect it as a free, unconstrained state --
    // so skip it and let the chain continue from here.
    if (state_.last_estimated_states.count(*prev) == 0)
    {
        MRPT_LOG_THROTTLE_DEBUG_FMT(
            5.0,
            "[fuse_odometry] relative factor skipped: KF %u is no longer in the window; "
            "the odometry chain continues from KF %u",
            static_cast<unsigned>(*prev), static_cast<unsigned>(this_kf_id));
        return;
    }

    // A relative transform is the same quantity in {map} and in {odom_i}: the
    // two chains differ by one constant rigid T_map_to_odom_i, which cancels in
    // T_prev^-1 * T_now. So the motion model's increment over the span between
    // both keyframes is directly the measurement this factor needs, with
    // directly the covariance it computes.
    const auto increment = odometry_increment_pdf(params_, odom.odometry - *prevOdom);

    gtsam::Pose3   incr_out;
    gtsam::Matrix6 incrCov_out;
    mola::gtsam_compat::to_gtsam_se3_cov6(increment, incr_out, incrCov_out);

    state_.gtsam->newFactors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        T(*prev), T(this_kf_id), incr_out, gtsam::noiseModel::Gaussian::Covariance(incrCov_out));
}

void StateEstimationSmoother::fuse_imu(const mrpt::obs::CObservationIMU& imu)
{
    if (params_.async_backend)
    {
        fastPredictor_->note_observation_stamp(imu.timestamp);
        const auto imuCopy = imu;
        enqueue_async(imu.timestamp, [this, imuCopy] { fuse_imu_locked(imuCopy); });
        return;
    }
    auto lck = mrpt::lockHelper(stateMutex_);
    fuse_imu_locked(imu);
}

void StateEstimationSmoother::fuse_imu_locked(const mrpt::obs::CObservationIMU& rawImu)
{
#if !GTSAM_USES_BOOST
    // Preintegration needs every raw reading, before any decimation below.
    if (params_.imu_preintegration.enabled)
    {
        buffer_raw_imu_locked(rawImu);
    }
#endif

    // Ignore an IMU reading with no usable content up front, before touching the
    // decimation stamp or creating a keyframe: otherwise an empty sample would
    // consume the decimation interval (skipping the next, useful one) and add a
    // factor-less keyframe.
    const bool hasAttitude =
        rawImu.has(mrpt::obs::IMU_ORI_QUAT_W) && params_.imu_attitude_sigma_deg > 0;
    const bool hasGravity =
        rawImu.has(mrpt::obs::IMU_X_ACC) && params_.imu_normalized_gravity_alignment_sigma > 0;
    const auto hasAngularVelocity = [this](const mrpt::obs::CObservationIMU& o)
    {
        return o.has(mrpt::obs::IMU_WX) && o.has(mrpt::obs::IMU_WY) && o.has(mrpt::obs::IMU_WZ) &&
               params_.imu_angular_velocity_sigma > 0;
    };
    if (!hasAttitude && !hasGravity && !hasAngularVelocity(rawImu))
    {
        return;
    }

    // High-rate decimation: fuse one reading per imu_min_sample_period, the
    // average of all readings in that period. Keeping one raw reading instead
    // would alias vibration (motors, propellers) into the factors below.
    if (!state_.imu_decimator)
    {
        state_.imu_decimator = std::make_shared<ImuDecimator>();
    }
    auto decimated = state_.imu_decimator->add(rawImu, params_.imu_min_sample_period);
    if (!decimated)
    {
        return;
    }
    imu_bias_correct(*decimated);
    const mrpt::obs::CObservationIMU& imu = *decimated;
    // From the fused reading: an average can carry channels its newest reading lacks.
    const bool fuseAngularVelocity = hasAngularVelocity(imu);

    // Absolute attitude readings, at most one per imu_attitude_min_sample_period:
    bool fuseAttitude = imu.has(mrpt::obs::IMU_ORI_QUAT_W) && params_.imu_attitude_sigma_deg > 0;
    if (fuseAttitude && params_.imu_attitude_min_sample_period > 0)
    {
        const auto it = state_.last_imu_attitude_stamp.find(imu.sensorLabel);
        if (it != state_.last_imu_attitude_stamp.end() &&
            std::abs(mrpt::system::timeDifference(it->second, imu.timestamp)) <
                params_.imu_attitude_min_sample_period)
        {
            fuseAttitude = false;
        }
    }

    // Gravity: the average of the accelerometer over imu_gravity_min_sample_period, if
    // that period just completed:
    std::optional<gtsam::Vector3> measuredGravity;
    if (imu.has(mrpt::obs::IMU_X_ACC) && params_.imu_normalized_gravity_alignment_sigma > 0)
    {
        mrpt::math::TVector3D acc = {
            imu.get(mrpt::obs::IMU_X_ACC), imu.get(mrpt::obs::IMU_Y_ACC),
            imu.get(mrpt::obs::IMU_Z_ACC)};

        if (params_.imu_gravity_compensate_centripetal && imu.has(mrpt::obs::IMU_WX) &&
            imu.has(mrpt::obs::IMU_WY) && imu.has(mrpt::obs::IMU_WZ) &&
            acc.norm() > ACC_NORM_MIN_FOR_SI_UNITS && !state_.stamp2frame_index.empty())
        {
            const auto newestKf = state_.stamp2frame_index.getDirectMap().rbegin()->second;
            if (const auto it = state_.last_estimated_states.find(newestKf);
                it != state_.last_estimated_states.end())
            {
                // Vehicle velocity in the IMU frame (lever arm neglected):
                const auto& tw = it->second.twist;
                const auto  vS = imu.sensorPose.inverseRotateVector({tw.vx, tw.vy, tw.vz});
                const mrpt::math::TVector3D wS = {
                    imu.get(mrpt::obs::IMU_WX), imu.get(mrpt::obs::IMU_WY),
                    imu.get(mrpt::obs::IMU_WZ)};
                // a = w x v, for a velocity constant in the body frame:
                const mrpt::math::TVector3D centripetal = {
                    wS.y * vS.z - wS.z * vS.y, wS.z * vS.x - wS.x * vS.z,
                    wS.x * vS.y - wS.y * vS.x};
                if (std::isfinite(centripetal.x) && std::isfinite(centripetal.y) &&
                    std::isfinite(centripetal.z))
                {
                    acc -= centripetal;
                }
            }
        }

        auto& avg = state_.imu_gravity_average[imu.sensorLabel];
        if (!avg.first_stamp)
        {
            avg.first_stamp = imu.timestamp;
        }
        avg.sum += acc;
        avg.count++;

        // Each decimated reading already covers about imu_min_sample_period:
        const double covered = mrpt::system::timeDifference(*avg.first_stamp, imu.timestamp) +
                               params_.imu_min_sample_period;
        if (covered >= params_.imu_gravity_min_sample_period - 1e-6)
        {
            const auto mean = avg.sum * (1.0 / static_cast<double>(avg.count));
            measuredGravity = gtsam::Vector3(mean.x, mean.y, mean.z);
            avg             = {};
        }
    }
    const bool fuseGravity = measuredGravity.has_value();

    if (!fuseAttitude && !fuseGravity && !fuseAngularVelocity)
    {
        return;  // a keyframe without factors would only add cost
    }

    // Create a new KF id (or reuse a very close match):
    const auto this_kf_id = create_or_get_keyframe_by_timestamp_locked(
        imu.timestamp, params_.imu_nearby_keyframe_stamp_tolerance);

    MRPT_LOG_DEBUG_FMT(
        "[fuse_imu]: t=%f  this_kf_id=%zu ", mrpt::Clock::toDouble(imu.timestamp),
        static_cast<size_t>(this_kf_id));

    // Shared by every branch below (at least one runs, given the early returns above):
    const auto sensorOnVehicle = mrpt::gtsam_wrappers::toPose3(imu.sensorPose);

    // Direct azimuth observation?
    // -------------------------------------------------
    if (fuseAttitude)
    {
        const double qw = imu.get(mrpt::obs::IMU_ORI_QUAT_W);
        const double qx = imu.get(mrpt::obs::IMU_ORI_QUAT_X);
        const double qy = imu.get(mrpt::obs::IMU_ORI_QUAT_Y);
        const double qz = imu.get(mrpt::obs::IMU_ORI_QUAT_Z);

        if (!mola::factors::imu_quaternion_looks_valid(qw, qx, qy, qz))
        {
            MRPT_LOG_THROTTLE_WARN(
                60.0, "Ignoring invalid (NaN or non-normalized) IMU orientation quaternion");
        }
        else
        {
            // GTSAM uses w,x,y,z quaternion order:
            const auto rawAttitude = gtsam::Rot3::Quaternion(qw, qx, qy, qz);

            // Sanity check: the "up" direction implied by the attitude must agree
            // with the accelerometer, or the two IMU factors will fight each other.
            if (imu.has(mrpt::obs::IMU_X_ACC))
            {
                const gtsam::Vector3 acc = {
                    imu.get(mrpt::obs::IMU_X_ACC), imu.get(mrpt::obs::IMU_Y_ACC),
                    imu.get(mrpt::obs::IMU_Z_ACC)};
                if (mola::factors::imu_accel_looks_like_gravity(acc))
                {
                    const gtsam::Vector3 upFromAttitude =
                        rawAttitude.unrotate(gtsam::Vector3(0, 0, 1));
                    const double     cosAngle    = upFromAttitude.dot(acc.normalized());
                    constexpr double maxAngleDeg = 30.0;
                    if (cosAngle < std::cos(mrpt::DEG2RAD(maxAngleDeg)))
                    {
                        MRPT_LOG_THROTTLE_WARN_FMT(
                            30.0,
                            "IMU attitude disagrees with its own accelerometer by %.1f deg: the "
                            "driver may be publishing a placeholder orientation. Consider setting "
                            "'imu_attitude_sigma_deg: 0' to ignore it.",
                            mrpt::RAD2DEG(std::acos(std::clamp(cosAngle, -1.0, 1.0))));
                    }
                }
            }

            const auto measuredRotation = mola::factors::imu_apply_enu_azimuth_correction(
                rawAttitude, params_.imu_attitude_azimuth_offset_deg);

            // Create noise model for rotation (3 DOF: roll, pitch, yaw)
            auto rotationNoise = gtsam::noiseModel::Isotropic::Sigma(
                3, mrpt::DEG2RAD(params_.imu_attitude_sigma_deg));

            state_.last_imu_attitude_stamp[imu.sensorLabel] = imu.timestamp;

            if (params_.imu_attitude_yaw_bias_sigma_deg > 0)
            {
                add_imu_yaw_bias_variable_locked(this_kf_id);
                state_.gtsam->newFactors.emplace_shared<mola::factors::Pose3RotationYawBiasFactor>(
                    symbol_T_enu_to_map, T(this_kf_id), B(this_kf_id), sensorOnVehicle,
                    measuredRotation, rotationNoise);
            }
            else
            {
                state_.gtsam->newFactors.emplace_shared<mola::factors::Pose3RotationFactor>(
                    symbol_T_enu_to_map, T(this_kf_id), sensorOnVehicle, measuredRotation,
                    rotationNoise);
            }

            // Absolute attitude observes the azimuth of T_enu_to_map, and through
            // a fixed one, the orientation of {map}:
            state_.gtsam->withdraw_anchor(state_.gtsam->enuYawAnchor);
            if (params_.fixed_geo_reference.has_value())
            {
                mark_map_observed_locked();
                mark_orientation_observed_locked(true, true);
            }
        }
    }

    // Gravity-aligned acceleration observation?
    // -------------------------------------------------
    // Not while the azimuth of an estimated geo-reference is still pinned by its
    // gauge anchor: these factors tie T_enu_to_map to the vehicle, and once
    // marginalized, at the pinned azimuth, which the real one may be far from
    // (see fuse_gnss_locked()). Meanwhile the vehicle attitude comes from odometry.
    if (fuseGravity && !(params_.estimate_geo_reference && state_.gtsam->enuYawAnchor))
    {
        // TODO: Use ImuTransformer, etc.

        // Some IMU drivers publishes normalized acc:
        if (mola::factors::imu_accel_looks_like_gravity(*measuredGravity))
        {
            const gtsam::Vector3 measuredGravityNormalized = measuredGravity->normalized();

            // Create noise model for gravity alignment:
            auto accNoise = gtsam::noiseModel::Isotropic::Sigma(
                3, mrpt::DEG2RAD(params_.imu_normalized_gravity_alignment_sigma));

            state_.gtsam->newFactors.emplace_shared<mola::factors::MeasuredGravityFactor>(
                symbol_T_enu_to_map, T(this_kf_id), sensorOnVehicle, measuredGravityNormalized,
                accNoise);

            // Gravity observes roll and pitch:
            if (params_.fixed_geo_reference.has_value())
            {
                mark_orientation_observed_locked(true, false);
            }
        }
    }

    // Angular velocity (gyroscope) observation: a direct prior on this keyframe's
    // body-frame angular-velocity variable, so a genuine, fast rotation is represented
    // in the graph immediately instead of only through the constant-velocity kinematic
    // factor between keyframes (see imu_angular_velocity_sigma's docstring).
    if (fuseAngularVelocity)
    {
        const gtsam::Vector3 measuredW_sensor = {
            imu.get(mrpt::obs::IMU_WX), imu.get(mrpt::obs::IMU_WY), imu.get(mrpt::obs::IMU_WZ)};

        if (!measuredW_sensor.allFinite())
        {
            MRPT_LOG_THROTTLE_WARN(
                60.0, "Ignoring invalid (NaN or Inf) IMU angular velocity reading");
        }
        else
        {
            // Angular velocity is a free vector under a fixed rigid rotation (no lever-arm
            // term, unlike linear velocity):
            const gtsam::Vector3 measuredW_vehicle = sensorOnVehicle.rotation() * measuredW_sensor;

            // Robust kernel: this is one raw, un-averaged sample of a noisy, often
            // vibration-contaminated signal, used as a direct observation of a state
            // variable. A single outlier (or a sigma set optimistically for the actual
            // platform) would otherwise drag the whole sliding window through the
            // body-frame coupling of the constant-velocity factor, which can only
            // absorb the disagreement by rotating the poses. Huber bounds how far any
            // one reading can pull the solution while leaving well-behaved samples
            // fully informative.
            auto gyroNoise = gtsam::noiseModel::Robust::Create(
                gtsam::noiseModel::mEstimator::Huber::Create(GYRO_HUBER_K),
                gtsam::noiseModel::Isotropic::Sigma(3, params_.imu_angular_velocity_sigma));

            state_.gtsam->newFactors.addPrior(W(this_kf_id), measuredW_vehicle, gyroNoise);
        }
    }
}

// One yaw bias variable per keyframe with attitude readings, all of them linked
// as a first-order Gauss-Markov process, so it can slowly change over time. Its
// stationary sigma is that of the prior on the first variable of the chain.
void StateEstimationSmoother::add_imu_yaw_bias_variable_locked(const frame_index_t kf)
{
    auto&            g   = *state_.gtsam;
    const gtsam::Key key = B(kf);
    if (g.newValues.exists(key) || g.smoother->getLinearizationPoint().exists(key))
    {
        return;  // another reading on the same keyframe
    }

    const double stamp  = key_stamp_seconds(state_.stamp2frame_index.inverse(kf));
    const double sigmaB = mrpt::DEG2RAD(params_.imu_attitude_yaw_bias_sigma_deg);

    double initialValue = 0;

    // Same check as for other chains: a factor on a marginalized variable would
    // bring it back as a free one.
    if (g.lastYawBiasKf && state_.last_estimated_states.count(*g.lastYawBiasKf) != 0)
    {
        const double dt  = std::abs(stamp - g.lastYawBiasStamp);
        const double phi = std::exp(-dt / params_.imu_attitude_yaw_bias_time_constant);
        // Floor: two keyframes may be (almost) simultaneous.
        constexpr double minSigma = 1e-6;
        const double     sigmaW   = std::max(sigmaB * std::sqrt(1.0 - phi * phi), minSigma);

        if (g.yawBiasEstimate)
        {
            initialValue = phi * g.yawBiasEstimate->first;
        }

        const gtsam::Expression<double> bPrev(B(*g.lastYawBiasKf));
        const gtsam::Expression<double> bNew(key);
        g.newFactors.emplace_shared<gtsam::ExpressionFactor<double>>(
            gtsam::noiseModel::Isotropic::Sigma(1, sigmaW), 0.0, bNew - phi * bPrev);
    }
    else
    {
        g.newFactors.addPrior(key, 0.0, gtsam::noiseModel::Isotropic::Sigma(1, sigmaB));
    }

    g.newValues.insert(key, initialValue);
    g.newKeyStamps[key] = stamp;
    g.lastYawBiasKf     = kf;
    g.lastYawBiasStamp  = stamp;
}

void StateEstimationSmoother::fuse_gnss(const mrpt::obs::CObservationGPS& gps)
{
    if (params_.async_backend)
    {
        fastPredictor_->note_observation_stamp(gps.timestamp);
        const auto gpsCopy = gps;
        enqueue_async(gps.timestamp, [this, gpsCopy] { fuse_gnss_locked(gpsCopy); });
        return;
    }
    auto lck = mrpt::lockHelper(stateMutex_);
    fuse_gnss_locked(gps);
}

void StateEstimationSmoother::fuse_gnss_locked(const mrpt::obs::CObservationGPS& gps)
{
    if (!gps.has_GGA_datum())
    {
        MRPT_LOG_DEBUG("[fuse_gnss]: Ignoring reading since it has no GGA data.");
        return;
    }
    const auto& gga = gps.getMsgByClass<mrpt::obs::gnss::Message_NMEA_GGA>();

    if (!gga.fields.fix_quality)
    {
        MRPT_LOG_DEBUG("[fuse_gnss]: Ignoring reading. GGA has no valid datum (fix_quality)");
        return;
    }

    const auto geoCoords = gga.getAsStruct<mrpt::topography::TGeodeticCoords>();

    if (!gps.covariance_enu.has_value())
    {
        MRPT_LOG_THROTTLE_WARN(
            5.0, "Discarding GNSS (GPS) reading since it does not have ENU covariance.");
        return;
    }

    // Receiver covariance with the configured floors, used for everything below
    // (gate, factor noise and azimuth baselines alike):
    Eigen::Matrix3d enuCov = gps.covariance_enu->asEigen();
    {
        const double minVarXY = mrpt::square(params_.gnss_min_sigma_floor_xy);
        const double minVarZ  = mrpt::square(params_.gnss_min_sigma_floor_z);
        enuCov(0, 0) += std::max(0.0, minVarXY - enuCov(0, 0));
        enuCov(1, 1) += std::max(0.0, minVarXY - enuCov(1, 1));
        enuCov(2, 2) += std::max(0.0, minVarZ - enuCov(2, 2));
    }
    const double sigmaXY = std::sqrt(std::max(enuCov(0, 0), enuCov(1, 1)));

    if (!gnss_gate_locked(gps, geoCoords, {enuCov(0, 0), enuCov(1, 1), enuCov(2, 2)}))
    {
        return;
    }

    std::optional<mrpt::topography::TGeodeticCoords> refGeoCoords;

    // Determine the reference to use
    if (state_.geo_reference.has_value())
    {
        // We have a finalized reference (either fixed or already estimated)
        refGeoCoords = state_.geo_reference->geo_coord;
    }
    else if (params_.estimate_geo_reference)
    {
        // We are still in the "tentative" phase
        if (!state_.tentative_geo_coord_reference)
        {
            state_.tentative_geo_coord_reference = geoCoords;

            MRPT_LOG_DEBUG_STREAM(
                "[fuse_gnss]: Defining as geodetic reference: lat="
                << geoCoords.lat.getAsString() << ", lon=" << geoCoords.lon.getAsString()
                << ", h=" << geoCoords.height);
        }
        refGeoCoords = state_.tentative_geo_coord_reference;
    }

    if (!refGeoCoords.has_value())
    {
        MRPT_LOG_DEBUG(
            "[fuse_gnss]: Ignoring reading; no fixed or tentative geo-reference available.");
        return;
    }

    mrpt::math::TPoint3D ENU_point;
    mrpt::topography::geodeticToENU_WGS84(geoCoords, ENU_point, *refGeoCoords);

    // The azimuth of T_enu_to_map becomes observable only once the fixes span a
    // horizontal baseline long enough for the configured convergence accuracy.
    // Until then (no attitude source) it is pinned by its gauge anchor at an
    // arbitrary value, and fixes are not fused: they would tie T_enu_to_map to the
    // vehicle at that azimuth, and once marginalized (e.g. while the vehicle stands
    // still for longer than the sliding window), at a linearization point far from
    // the real one. Starting from there, the solver does not reach the real azimuth
    // (it may be up to 180 deg away, and the robust GNSS kernel weakens the large
    // residuals), so the anchor is replaced by a prior at the azimuth measured by
    // the baseline: the GNSS displacement vs. the antenna displacement in {map}.
    if (state_.gtsam->enuYawAnchor)
    {
        // Solved antenna position in {map} at the fix time: from the solved keyframe
        // closest in time, moved by its twist to that time.
        std::optional<gtsam::Point2> antennaMapXY;
        {
            constexpr double MAX_DT = 0.25;  // [s]
            if (const auto p = predicted_antenna_in_map_locked(
                    gps.timestamp, gps.sensorPose.translation(), MAX_DT);
                p)
            {
                antennaMapXY = gtsam::Point2(p->x, p->y);
            }
        }
        if (!antennaMapXY)
        {
            return;  // no solved vehicle pose near the fix time yet
        }

        const gtsam::Point2 xy(ENU_point.x, ENU_point.y);
        auto&               g = *state_.gtsam;
        if (!g.firstGnssEnuXY)
        {
            g.firstGnssEnuXY = xy;
            g.firstGnssMapXY = antennaMapXY;
            return;
        }
        const auto   enuBaseline  = xy - *g.firstGnssEnuXY;
        const auto   mapBaseline  = *antennaMapXY - *g.firstGnssMapXY;
        const double baselineNorm = enuBaseline.norm();
        const double minBaseline =
            std::sqrt(2.0) * sigmaXY / mrpt::DEG2RAD(params_.convergence_max_orientation_sigma_deg);
        if (baselineNorm < minBaseline)
        {
            return;
        }

        // Odometry and GNSS must agree on the distance, or the azimuth is not
        // reliable: keep the anchor and measure a new baseline from here.
        if (std::abs(mapBaseline.norm() - baselineNorm) >= 0.5 * baselineNorm)
        {
            g.firstGnssEnuXY = xy;
            g.firstGnssMapXY = antennaMapXY;
            return;
        }

        g.withdraw_anchor(g.enuYawAnchor);
        {
            const double yaw = std::atan2(enuBaseline.y(), enuBaseline.x()) -
                               std::atan2(mapBaseline.y(), mapBaseline.x());
            // Its accuracy, from the two fixes, with margin for the odometry:
            const double yawSigma = 2.0 * std::sqrt(2.0) * sigmaXY / baselineNorm;

            const auto  current = estimated_T_enu_to_map_locked();
            const auto& cur     = current ? current->mean : mrpt::poses::CPose3D();
            const auto  rot = mrpt::poses::CPose3D::FromYawPitchRoll(yaw, cur.pitch(), cur.roll());
            const auto  antennaMap = mrpt::math::TPoint3D(antennaMapXY->x(), antennaMapXY->y(), 0);
            const auto  rotated    = rot.rotateVector(antennaMap);
            const auto  mean       = mrpt::poses::CPose3D::FromXYZYawPitchRoll(
                       ENU_point.x - rotated.x, ENU_point.y - rotated.y, cur.z(), yaw, cur.pitch(),
                       cur.roll());

            // Its horizontal position follows from the same fix, and roll/pitch are
            // held loosely until gravity observes them (not fused while pinned): tied
            // loosely, so the solver starts near them instead of from the pinned values.
            // Pose3 tangent: rotation (x, y, z), then translation:
            constexpr double TILT_SIGMA     = 0.2;  // [rad]
            constexpr double POSITION_SIGMA = 1.0;  // [m]
            gtsam::Vector6   sigmas;
            sigmas << TILT_SIGMA, TILT_SIGMA, yawSigma, POSITION_SIGMA, POSITION_SIGMA,
                ENU2MAP_WEAK_SIGMA;
            g.newFactors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
                symbol_T_enu_to_map, mrpt::gtsam_wrappers::toPose3(mean),
                gtsam::noiseModel::Diagonal::Sigmas(sigmas));

            MRPT_LOG_DEBUG_FMT(
                "[fuse_gnss] Azimuth of T_enu_to_map observable: %.2f deg (sigma %.2f deg) "
                "from a %.2f m GNSS baseline",
                mrpt::RAD2DEG(yaw), mrpt::RAD2DEG(yawSigma), baselineNorm);
        }
    }

    // Create a new KF id (or reuse a very close match):
    const auto this_kf_id = create_or_get_keyframe_by_timestamp_locked(
        gps.timestamp, params_.gnss_nearby_keyframe_stamp_tolerance, true /*pickClosest*/);

    MRPT_LOG_DEBUG_FMT(
        "[fuse_gnss]: t=%f this_kf_id=%zu ENU=%s", mrpt::Clock::toDouble(gps.timestamp),
        static_cast<size_t>(this_kf_id), ENU_point.asString().c_str());

    // Add geo-ref factor:
    const auto sensorOnVehicle = mrpt::gtsam_wrappers::toPoint3(gps.sensorPose.translation());
    const auto observedEnu     = mrpt::gtsam_wrappers::toPoint3(ENU_point);

    const auto enuNoise = gtsam::noiseModel::Gaussian::Covariance(enuCov);

    gtsam::SharedNoiseModel enuNoiseModel;
    if (params_.gnss_huber_threshold > 0)
    {
        enuNoiseModel = gtsam::noiseModel::Robust::Create(
            make_gnss_kernel(params_.gnss_robust_kernel, params_.gnss_huber_threshold), enuNoise);
    }
    else
    {
        enuNoiseModel = enuNoise;
    }

    // The fix may be up to gnss_nearby_keyframe_stamp_tolerance away from its
    // keyframe: move the antenna to the fix time with the keyframe's twist, or a
    // moving vehicle would be pulled by speed * dt towards where it was not.
    const double fixDt =
        mrpt::system::timeDifference(state_.stamp2frame_index.inverse(this_kf_id), gps.timestamp);
    if (std::abs(fixDt) > 1e-6)
    {
        state_.gtsam->newFactors.emplace_shared<mola::factors::FactorGnssMapEnuConstVel>(
            symbol_T_enu_to_map, T(this_kf_id), V(this_kf_id), W(this_kf_id), sensorOnVehicle,
            fixDt, observedEnu, enuNoiseModel);
    }
    else
    {
        state_.gtsam->newFactors.emplace_shared<mola::factors::FactorGnssMapEnu>(
            symbol_T_enu_to_map, T(this_kf_id), sensorOnVehicle, observedEnu, enuNoiseModel);
    }

    // Through a fixed T_enu_to_map, GNSS observes where {map} is:
    if (params_.fixed_geo_reference.has_value())
    {
        mark_map_observed_locked();
        state_.gtsam->mapPositionObserved = true;
    }

    // Likewise, a pinned yaw becomes observable once the fixes span a baseline
    // long enough for a few degrees of accuracy (static noise must not count):
    if (state_.gtsam->yawPinned())
    {
        constexpr double    YAW_FROM_BASELINE_ACCURACY_DEG = 5.0;
        const gtsam::Point2 xy(ENU_point.x, ENU_point.y);
        if (!state_.gtsam->firstGnssXYForYaw)
        {
            state_.gtsam->firstGnssXYForYaw = xy;
        }
        else
        {
            const double minBaseline =
                std::sqrt(2.0) * sigmaXY / mrpt::DEG2RAD(YAW_FROM_BASELINE_ACCURACY_DEG);
            if ((xy - *state_.gtsam->firstGnssXYForYaw).norm() >= minBaseline)
            {
                mark_orientation_observed_locked(false, true);
            }
        }
    }
}

void StateEstimationSmoother::fuse_pose(
    const mrpt::Clock::time_point& timestamp, const mrpt::poses::CPose3DPDFGaussian& pose,
    const std::string& frame_id)
{
    imu_bias_add_pose(timestamp, pose.mean, frame_id);
    if (params_.async_backend)
    {
        fastPredictor_->note_observation_stamp(timestamp);
        enqueue_async(
            timestamp,
            [this, timestamp, pose, frame_id] { fuse_pose_locked(timestamp, pose, frame_id); });
        return;
    }
    auto lck = mrpt::lockHelper(stateMutex_);
    fuse_pose_locked(timestamp, pose, frame_id);
}

#if defined(MOLA_KERNEL_NAVSTATE_FILTER_HAS_FUSE_POSE_DEFERRED)
void StateEstimationSmoother::fuse_pose_deferred(
    const mrpt::Clock::time_point& timestamp, const mrpt::poses::CPose3D& mean,
    const std::shared_future<mrpt::math::CMatrixDouble66>& cov, const std::string& frame_id)
{
    if (!params_.async_backend)
    {
        fuse_pose(timestamp, mrpt::poses::CPose3DPDFGaussian(mean, cov.get()), frame_id);
        return;
    }
    imu_bias_add_pose(timestamp, mean, frame_id);
    fastPredictor_->note_observation_stamp(timestamp);
    {
        std::lock_guard<std::mutex> lk(queueMutex_);
        deferredCovs_.push_back(cov);
    }
    enqueue_async(
        timestamp,
        [this, timestamp, mean, cov, frame_id]
        {
            // A failed producer only drops its own reading, not the whole batch:
            mrpt::math::CMatrixDouble66 c;
            try
            {
                c = cov.get();
            }
            catch (const std::exception& e)
            {
                MRPT_LOG_WARN_STREAM(
                    "fuse_pose_deferred(): covariance unavailable, reading dropped: " << e.what());
                return;
            }
            fuse_pose_locked(timestamp, mrpt::poses::CPose3DPDFGaussian(mean, c), frame_id);
        });
}
#endif

void StateEstimationSmoother::fuse_pose_locked(
    const mrpt::Clock::time_point& timestamp, const mrpt::poses::CPose3DPDFGaussian& pose,
    const std::string& frame_id)
{
    // get this numerical frame_id :
    move_wheels_odometry_frame_aside_locked(frame_id);
    const auto frame_id_idx = add_or_get_odom_frame_id(frame_id);

    // numerical sanity: replace zero-variance entries (common in
    // nav_msgs/Odometry messages with unfilled covariance) with a
    // reasonable default so the factor graph remains well-conditioned.
    auto poseSanitized = pose;
    bool patched       = false;
    for (int i = 0; i < 6; i++)
    {
        if (poseSanitized.cov(i, i) <= .0)
        {
            // Default sigmas: 1 m for position (i<3), 0.1 rad (~6 deg) for orientation
            const double defaultSigma = (i < 3) ? 1.0 : 0.1;
            poseSanitized.cov(i, i)   = defaultSigma * defaultSigma;
            patched                   = true;
        }
    }
    if (patched)
    {
        MRPT_LOG_THROTTLE_WARN_FMT(
            5.0,
            "[fuse_pose] frame='%s': zero diagonal covariance entries patched with defaults "
            "(source may not be publishing covariance)",
            frame_id.c_str());
    }

    // High-rate decimation: drop this reading if it arrives too soon after the
    // last kept one of the same source. Done BEFORE the keyframe is created, so
    // a dropped reading costs nothing and, for an odometry frame,
    // leaves the source's chain tail where it was: the next kept reading then
    // asserts the whole merged span as one increment, losing no motion.
    if (params_.pose_min_sample_period > 0)
    {
        if (auto it = state_.last_kept_pose_stamp.find(frame_id_idx);
            it != state_.last_kept_pose_stamp.end())
        {
            const double dt = mrpt::system::timeDifference(it->second, timestamp);
            if (dt < params_.pose_min_sample_period)
            {
                // Not fused, but still the freshest pose of this source in its
                // own frame, which is what the frame-local estimated_navstate()
                // extrapolates from.
                if (frame_id_idx != REFERENCE_FRAME_ID)
                {
                    auto itRaw = state_.last_raw_pose_by_source.find(frame_id_idx);
                    if (itRaw != state_.last_raw_pose_by_source.end() &&
                        timestamp > itRaw->second.stamp)
                    {
                        itRaw->second = State::RawSourcePose{timestamp, poseSanitized};
                    }
                }
                return;
            }
        }
        state_.last_kept_pose_stamp[frame_id_idx] = timestamp;
    }

    // Create a new KF id (or reuse a very close match):
    const auto this_kf_id = create_or_get_keyframe_by_timestamp_locked(timestamp);

    MRPT_LOG_DEBUG_FMT(
        "[fuse_pose]: kf_idx=%zu t=%f frame='%s' (idx=%zu) p=%s sigmas=%.02e %.02e %.02e (m) %.02e "
        "%.02e %.02e (deg)",
        static_cast<std::size_t>(this_kf_id), mrpt::Clock::toDouble(timestamp), frame_id.c_str(),
        static_cast<std::size_t>(frame_id_idx), pose.mean.asString().c_str(),
        std::sqrt(pose.cov(0, 0)), std::sqrt(pose.cov(1, 1)), std::sqrt(pose.cov(2, 2)),
        mrpt::RAD2DEG(std::sqrt(pose.cov(3, 3))), mrpt::RAD2DEG(std::sqrt(pose.cov(4, 4))),
        mrpt::RAD2DEG(std::sqrt(pose.cov(5, 5))));

    // Add factor:
    gtsam::Pose3   pose_out;
    gtsam::Matrix6 cov_out;
    mola::gtsam_compat::to_gtsam_se3_cov6(poseSanitized, pose_out, cov_out);

    // reference frame ("map") or "odom_i"?
    if (frame_id_idx == REFERENCE_FRAME_ID)
    {
        // ref is "map":
        mark_map_observed_locked();
        state_.gtsam->mapPositionObserved = true;
        mark_orientation_observed_locked(true, true);
        state_.gtsam->newFactors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
            T(this_kf_id), pose_out,
            with_huber(
                gtsam::noiseModel::Gaussian::Covariance(cov_out),
                params_.pose_robust_huber_threshold));
    }
    else
    {
        // An odometry frame: every such source drifts, so only the increments
        // between consecutive readings are asserted, plus one absolute factor
        // added once to resolve T_map_to_odom_i. Mirrors
        // fuse_odometry_relative_locked(), generalized to any fuse_pose()
        // source.
        auto& chain = state_.relative_pose_chains[frame_id_idx];

        // Only strictly newer readings may extend the chain: an out-of-order or
        // repeated sample would otherwise rewind the tail, and the next fresh
        // reading would then assert the whole rewound span as ONE increment,
        // with a one-increment covariance.
        if (chain.last_stamp.has_value() && timestamp <= *chain.last_stamp)
        {
            MRPT_LOG_THROTTLE_WARN_FMT(
                5.0,
                "[fuse_pose] frame='%s': dropping non-monotonic reading (t=%f, chain tail t=%f)",
                frame_id.c_str(), mrpt::Clock::toDouble(timestamp),
                mrpt::Clock::toDouble(*chain.last_stamp));
            return;
        }

        if (!chain.anchor_kf.has_value())
        {
            seed_odom_frame_locked(frame_id_idx, this_kf_id, poseSanitized.mean);
            state_.gtsam->newFactors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
                symbol_T_map_to_odom_i_base + frame_id_idx, T(this_kf_id), pose_out,
                gtsam::noiseModel::Gaussian::Covariance(cov_out));
            chain.anchor_kf = this_kf_id;

            MRPT_LOG_DEBUG_FMT(
                "[fuse_pose] frame '%s' anchored at KF %u (relative formulation)", frame_id.c_str(),
                static_cast<unsigned>(this_kf_id));
        }

        // A relative transform is the same quantity in {map} and in {odom_i}:
        // the two chains differ by one constant rigid T_map_to_odom_i, which
        // cancels in T_prev^-1 * T_now.
        if (chain.last_kf.has_value() && chain.last_pose_in_odom.has_value() &&
            *chain.last_kf != this_kf_id)
        {
            // A NEW factor naming a marginalized variable would resurrect it as
            // a free, unconstrained state: skip it and continue the chain here.
            if (state_.last_estimated_states.count(*chain.last_kf) == 0)
            {
                MRPT_LOG_THROTTLE_DEBUG_FMT(
                    5.0,
                    "[fuse_pose] relative factor skipped: KF %u is no longer in the window; "
                    "the '%s' chain continues from KF %u",
                    static_cast<unsigned>(*chain.last_kf), frame_id.c_str(),
                    static_cast<unsigned>(this_kf_id));
            }
            else
            {
                const mrpt::poses::CPose3D increment =
                    poseSanitized.mean - chain.last_pose_in_odom->mean;

                // A drifting source publishes the covariance of its absolute
                // dead-reckoned pose, which says nothing about one increment,
                // so the configured per-increment accuracy is asserted instead.
                // Slips and skids are independent events along the path, so
                // the variance of a dead-reckoned increment grows linearly
                // with its size (a random walk). Unlike a sigma proportional
                // to the size, this gives the same total variance however the
                // path is split into increments, i.e. whatever the keyframe
                // rate or pose_min_sample_period. The flat value is the floor
                // asserted while standing still.
                const double dL = increment.translation().norm();
                const double dA =
                    mrpt::poses::Lie::SO<3>::log(increment.getRotationMatrix()).norm();

                const double varLin =
                    mrpt::square(params_.relative_pose_increment_sigma_lin) +
                    mrpt::square(params_.relative_pose_increment_sigma_per_sqrt_meter) * dL;
                const double varAng =
                    mrpt::square(params_.relative_pose_increment_sigma_ang) +
                    mrpt::square(params_.relative_pose_increment_sigma_per_sqrt_rad) * dA;

                // Isotropic in the GTSAM tangent (rx,ry,rz,tx,ty,tz) of the increment:
                const gtsam::Pose3 incr_out    = mrpt::gtsam_wrappers::toPose3(increment);
                gtsam::Matrix6     incrCov_out = gtsam::Matrix6::Zero();
                for (int i = 0; i < 3; i++)
                {
                    incrCov_out(i, i)         = varAng;
                    incrCov_out(i + 3, i + 3) = varLin;
                }
                if (params_.relative_pose_increment_from_source_covariance)
                {
                    // The covariance of the reading, in the right tangent of the
                    // current pose: with the previous pose held fixed, that is
                    // also the right tangent of the increment.
                    incrCov_out = cov_out;
                }

                state_.gtsam->newFactors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
                    T(*chain.last_kf), T(this_kf_id), incr_out,
                    with_huber(
                        gtsam::noiseModel::Gaussian::Covariance(incrCov_out),
                        params_.pose_robust_huber_threshold));
            }
        }

        // Advance the chain's measurement anchor ONLY when the keyframe
        // actually changed. Several readings can land on the same keyframe
        // (min_time_difference_to_create_new_frame merges near-simultaneous
        // ones), and no factor is created for those; moving the anchor anyway
        // would leave the motion between the first and last of them out of
        // every increment, so the chain would report less motion than the
        // source measured. Holding the anchor makes the next increment span
        // the whole keyframe-to-keyframe interval.
        if (!chain.last_kf.has_value() || *chain.last_kf != this_kf_id)
        {
            chain.last_kf           = this_kf_id;
            chain.last_pose_in_odom = poseSanitized;
        }
        chain.last_stamp = timestamp;

        // Remember this source's own last raw pose (in {odom_i}), the anchor
        // estimated_navstate() extrapolates from to keep the short-term
        // prediction continuous in the front end's own frame.
        update_prediction_residuals_locked(frame_id_idx, timestamp, poseSanitized.mean);
        state_.last_raw_pose_by_source[frame_id_idx] =
            State::RawSourcePose{timestamp, poseSanitized};
    }
}

void StateEstimationSmoother::fuse_twist(
    const mrpt::Clock::time_point& timestamp, const mrpt::math::TTwist3D& twist,
    const mrpt::math::CMatrixDouble66& twistCov)
{
    imu_bias_add_twist(timestamp, twist);
    if (params_.async_backend)
    {
        fastPredictor_->note_observation_stamp(timestamp);
        enqueue_async(
            timestamp,
            [this, timestamp, twist, twistCov] { fuse_twist_locked(timestamp, twist, twistCov); });
        return;
    }
    auto lck = mrpt::lockHelper(stateMutex_);
    fuse_twist_locked(timestamp, twist, twistCov);
}

void StateEstimationSmoother::fuse_twist_locked(
    const mrpt::Clock::time_point& timestamp, const mrpt::math::TTwist3D& twist,
    const mrpt::math::CMatrixDouble66& twistCov)
{
    const gtsam::Vector3 v    = {twist.vx, twist.vy, twist.vz};
    const gtsam::Vector3 w    = {twist.wx, twist.wy, twist.wz};
    gtsam::Matrix3       vCov = twistCov.asEigen().block<3, 3>(0, 0);
    gtsam::Matrix3       wCov = twistCov.asEigen().block<3, 3>(3, 3);

    // Create a new KF id (or reuse a very close match):
    const auto this_kf_id = create_or_get_keyframe_by_timestamp_locked(timestamp);

    {
        auto                                noiseV = gtsam::noiseModel::Gaussian::Covariance(vCov);
        gtsam::noiseModel::Base::shared_ptr robNoiseV;
#if 0
        if (params_.robust_param > 0)
        {
            robNoiseV = gtsam::noiseModel::Robust::Create(
                gtsam::noiseModel::mEstimator::GemanMcClure::Create(params_.robust_param), noiseV);
        }
        else
#endif
        {
            robNoiseV = noiseV;
        }

        state_.gtsam->newFactors.addPrior(V(this_kf_id), v, robNoiseV);
    }
    {
        auto                                noiseW = gtsam::noiseModel::Gaussian::Covariance(wCov);
        gtsam::noiseModel::Base::shared_ptr robNoiseW;
#if 0
        if (params_.robust_param > 0)
        {
            robNoiseW = gtsam::noiseModel::Robust::Create(
                gtsam::noiseModel::mEstimator::GemanMcClure::Create(params_.robust_param), noiseW);
        }
        else
#endif
        {
            robNoiseW = noiseW;
        }

        state_.gtsam->newFactors.addPrior(W(this_kf_id), w, robNoiseW);
    }

    MRPT_LOG_DEBUG_FMT(
        "[fuse_twist]: t=%f this_kf_id=%zu twist=%s sigmas=%.02e %.02e %.02e (m) %.02e %.02e "
        "%.02e (deg)",
        mrpt::Clock::toDouble(timestamp), static_cast<std::size_t>(this_kf_id),
        twist.asString().c_str(), std::sqrt(twistCov(0, 0)), std::sqrt(twistCov(1, 1)),
        std::sqrt(twistCov(2, 2)), mrpt::RAD2DEG(std::sqrt(twistCov(3, 3))),
        mrpt::RAD2DEG(std::sqrt(twistCov(4, 4))), mrpt::RAD2DEG(std::sqrt(twistCov(5, 5))));
}

std::optional<NavState> StateEstimationSmoother::estimated_navstate(
    const mrpt::Clock::time_point& timestamp, const std::string& frame_id)
{
    // Async mode: served sub-ms by the lock-free fast predictor, without running
    // a solve or taking stateMutex_ (the backend thread may be holding it).
    if (params_.async_backend)
    {
        mrpt::Clock::time_point anchorStamp;
        auto ret = fastPredictor_->predict(timestamp, frame_id, params_, &anchorStamp);
        if (ret.has_value())
        {
            // Extrapolation interval, i.e. how far behind the query the backend's
            // last completed solve sits. In this path it is a function of host
            // load, not of the data. Taken from the very snapshot this prediction
            // used, not from a second snapshot() call that could race the backend.
            navstate_dump_row(
                timestamp, mrpt::system::timeDifference(anchorStamp, timestamp), *ret);
        }
        return ret;
    }

    auto lck = mrpt::lockHelper(stateMutex_);

    // 1) Make sure we processed all pending sensor data, and have updated the cached values from
    //    GTSAM values
    process_pending_gtsam_updates_locked();

    // 2) Get the vehicle state from cached optimized values:
    // Look for the closest frame and extrapolate.

    std::optional<double>        closestFrameDt;
    double                       closestFrameDtSigned = 0;
    std::optional<frame_index_t> closesFrameIdx;

    const auto closestPrior = find_before_after(timestamp, true);
    for (const auto& it : {closestPrior.first, closestPrior.second})
    {
        if (it == state_.stamp2frame_index.getDirectMap().end())
        {
            continue;
        }
        const auto& [existing_t, frame_idx] = *it;

        const double dt    = mrpt::system::timeDifference(existing_t, timestamp);
        const double dtAbs = std::abs(dt);
        if (!closestFrameDt.has_value() || dtAbs < *closestFrameDt)
        {
            closestFrameDt       = dtAbs;
            closesFrameIdx       = frame_idx;
            closestFrameDtSigned = dt;
        }
    }

    // Check maximum extrapolation time:
    if (!closesFrameIdx.has_value() || closestFrameDt > params_.max_time_to_use_velocity_model)
    {
        MRPT_LOG_DEBUG_FMT(
            "[estimated_navstate] Could not find any nearby frame near requested t=%.03f",
            mrpt::Clock::toDouble(timestamp));
        return {};
    }

    // Recover the closest state *in the reference frame*:
    //
    // get_latest_state_and_covariance() can throw if the factor graph is
    // still under-constrained (e.g. GTSAM's marginalCovariance() failing
    // with IndeterminantLinearSystemException): expected during the first
    // instants of any fusion (not enough diverse sensor data yet to fully
    // observe all 6 DOF), not a fatal error. Treat it the same as the other
    // "not ready yet" early returns above rather than letting it propagate
    // and take down the whole module thread.
    NavState retKf;
    try
    {
        const auto tleMarg = mola::ProfilerEntry(profiler_, "estimated_navstate.marginals.anchor");
        retKf              = get_latest_state_and_covariance(*closesFrameIdx);
    }
    catch (const std::exception& e)
    {
        MRPT_LOG_DEBUG_FMT(
            "[estimated_navstate] State for KF %u not ready yet (factor graph likely still "
            "under-constrained): %s",
            static_cast<unsigned>(*closesFrameIdx), e.what());
        return {};
    }

    NavState ret = retKf;

    // The covariance propagation below (matrix inversions, information-form
    // conversions and Gaussian pose composition) can throw if a covariance is
    // not positive definite (e.g. an under-constrained factor graph in the
    // first instants of fusion). Treat that like the other "not ready yet"
    // early returns rather than letting it take down the caller's thread.
    try
    {
        // Anchor twist covariance (before random-walk growth), reused as the
        // current-velocity uncertainty of the pose increment below.
        const mrpt::math::CMatrixDouble66 anchorTwistCov = retKf.twist_inv_cov.inverse_LLt();

        // Twist uncertainty growth due to the acceleration random walk:
        {
            auto twist_cov = anchorTwistCov;
            for (int i = 0; i < 3; i++)
            {
                twist_cov(0 + i, 0 + i) += mrpt::square(
                    params_.sigma_random_walk_acceleration_linear * closestFrameDtSigned);

                twist_cov(3 + i, 3 + i) += mrpt::square(
                    params_.sigma_random_walk_acceleration_angular * closestFrameDtSigned);
            }
            ret.twist_inv_cov = twist_cov.inverse_LLt();
        }

        // Extrapolate the low-pass-filtered velocity instead of the boundary
        // keyframe's raw (noisy) twist, when anchoring on the newest keyframe
        // (mirrors the async fast-predictor path).
        if (params_.predict_twist_filter_enabled && state_.filtered_predict_twist.has_value() &&
            !state_.stamp2frame_index.empty() &&
            *closesFrameIdx == state_.stamp2frame_index.getDirectMap().rbegin()->second)
        {
            ret.twist = *state_.filtered_predict_twist;
        }

        // 3) Produce the pose in the requested frame.
        if (frame_id == params_.reference_frame_name)
        {
            // Reference ({map}) frame: extrapolate the closest keyframe pose
            // forward with the configured kinematic model, propagating covariance.
            mrpt::poses::CPose3DPDFGaussian anchorPose;
            anchorPose.copyFrom(retKf.pose);
            auto mapPdf = extrapolate_pose_pdf(
                params_, anchorPose, ret.twist, anchorTwistCov, closestFrameDtSigned);
            apply_pose_sigma_floor(params_, mapPdf);
            ret.pose.copyFrom(mapPdf);
            navstate_dump_row(timestamp, closestFrameDtSigned, ret);
            return ret;
        }

        // ENU frame: the {map} prediction through the current T_enu_to_map.
        if (frame_id == params_.enu_frame_name)
        {
            const auto georef = georef_for_enu_poses_locked();
            if (!georef)
            {
                return {};  // not geo-referenced (yet)
            }
            mrpt::poses::CPose3DPDFGaussian anchorPose;
            anchorPose.copyFrom(retKf.pose);
            const auto mapPdf = extrapolate_pose_pdf(
                params_, anchorPose, ret.twist, anchorTwistCov, closestFrameDtSigned);
            auto enuPdf = georef->T_enu_to_map + mapPdf;
            apply_pose_sigma_floor(params_, enuPdf);
            ret.pose.copyFrom(enuPdf);
            navstate_dump_row(timestamp, closestFrameDtSigned, ret);
            return ret;
        }

        // The requested odometry frame may not have been registered yet (e.g.
        // the very first query of a brand-new frame_id, before any fuse_pose()/
        // fuse_odometry() call has registered it). Treat that as "not ready yet".
        const auto it = state_.known_odom_frames.find_key(frame_id);
        if (it == state_.known_odom_frames.getDirectMap().end())
        {
            MRPT_LOG_THROTTLE_WARN_FMT(
                5.0, "[estimated_navstate] Requested unknown odometry frame_id='%s'",
                frame_id.c_str());
            return {};
        }
        const auto requestedFrameIdx = it->second;

        // Non-reference odometry frame {odom_i}: anchor on the source's OWN last
        // raw pose in {odom_i} and extrapolate by the body-twist increment,
        // instead of reconstructing it globally as X(kf) (-) T_map_to_odom_i. The
        // fixed-lag window keeps that global reconstruction's {map}-correction
        // leak small here, but anchoring on the raw pose removes it and keeps the
        // prediction immune to geo-ref / loop-closure / per-solve jitter.
        const auto itRaw = state_.last_raw_pose_by_source.find(requestedFrameIdx);
        if (itRaw == state_.last_raw_pose_by_source.end())
        {
            // No raw pose received from this source yet: fall back to the global
            // conversion (correct while {map} and {odom_i} still coincide).
            const auto itFrame = state_.last_estimated_frames.find(requestedFrameIdx);
            if (itFrame == state_.last_estimated_frames.end())
            {
                return {};
            }
            mrpt::poses::CPose3DPDFGaussian anchorPose;
            anchorPose.copyFrom(retKf.pose);
            const auto mapPred = extrapolate_pose_pdf(
                params_, anchorPose, ret.twist, anchorTwistCov, closestFrameDtSigned);
            // Transform the {map}-frame prediction into {odom_i}: pred (-) T_frame_wrt_map.
            // The floor goes on AFTER the conversion: it is a statement about the
            // frame the caller asked for, and the composition mixes the angular
            // block into the translation one through the lever arm.
            auto framePdf = mapPred - itFrame->second;
            apply_pose_sigma_floor(params_, framePdf);
            ret.pose.copyFrom(framePdf);
            navstate_dump_row(timestamp, closestFrameDtSigned, ret);
            return ret;
        }

        // Frame-local extrapolation from the source's last raw pose in {odom_i}:
        const auto&  rawAnchor = itRaw->second;
        const double dtPred    = mrpt::system::timeDifference(rawAnchor.stamp, timestamp);

        if (std::abs(dtPred) > params_.max_time_to_use_velocity_model)
        {
            return {};
        }

        // The anchor is the front end's own near-exact pose in {odom_i}, so
        // prediction uncertainty is dominated by the one-step extrapolation, not
        // the absolute {map}-frame keyframe covariance.
        mrpt::poses::CPose3DPDFGaussian rawPdf;
        const auto                      itRes = state_.prediction_residuals.find(requestedFrameIdx);
        if (params_.predict_covariance_from_residuals &&
            itRes != state_.prediction_residuals.end() &&
            itRes->second.count >= MIN_PREDICTION_RESIDUALS)
        {
            rawPdf = extrapolate_pose_pdf(
                params_, rawAnchor.pose, ret.twist, twist_cov_from_residuals(itRes->second.varRate),
                dtPred, false /* no random walk: in the residuals already */);
        }
        else
        {
            rawPdf =
                extrapolate_pose_pdf(params_, rawAnchor.pose, ret.twist, anchorTwistCov, dtPred);
        }
        apply_pose_sigma_floor(params_, rawPdf);
        ret.pose.copyFrom(rawPdf);
        navstate_dump_row(timestamp, dtPred, ret);

        return ret;
    }
    catch (const std::exception& e)
    {
        MRPT_LOG_DEBUG_FMT(
            "[estimated_navstate] Covariance propagation not ready yet (factor graph likely "
            "still under-constrained): %s",
            e.what());
        return {};
    }
}

std::optional<mrpt::poses::CPose3DInterpolator> StateEstimationSmoother::estimated_trajectory(
    const mrpt::Clock::time_point& start_time, const mrpt::Clock::time_point& end_time,
    const std::string& frame_id)
{
    if (!params_.keep_finalized_trajectory)
    {
        return {};
    }
    auto lck = mrpt::lockHelper(stateMutex_);

    // The stored poses are keyframe poses in the reference frame. For an
    // odometry frame they are converted with the LATEST estimate of
    // T_frame_wrt_map, held constant over the whole trajectory: exact while the
    // two frames coincide, which is the case of an offline odometry run, and
    // the best available answer otherwise, since no per-keyframe anchor is kept.
    std::optional<mrpt::poses::CPose3D> T_frame_wrt_map;
    // In the ENU frame, keyframes without a pose recorded in ENU when they were
    // finalized go through the latest T_enu_to_map:
    std::optional<mrpt::poses::CPose3D> T_enu_to_map;
    if (frame_id == params_.enu_frame_name)
    {
        const auto georef = georef_for_enu_poses_locked();
        if (!georef)
        {
            return {};
        }
        T_enu_to_map = georef->T_enu_to_map.mean;
    }
    else if (frame_id != params_.reference_frame_name)
    {
        const auto itId = state_.known_odom_frames.find_key(frame_id);
        if (itId == state_.known_odom_frames.getDirectMap().end())
        {
            return {};
        }
        const auto itFrame = state_.last_estimated_frames.find(itId->second);
        if (itFrame == state_.last_estimated_frames.end())
        {
            return {};
        }
        T_frame_wrt_map = itFrame->second.mean;
    }

    // Everything already marginalized out, plus what is still in the window at
    // its current value, so the tail of an offline run is not lost.
    mrpt::poses::CPose3DInterpolator ret;
    const auto                       inRange = [&](const mrpt::Clock::time_point& t)
    { return t >= start_time && t <= end_time; };

    const auto toRequestedFrame = [&](const mrpt::poses::CPose3D& pInMap)
    {
        if (T_enu_to_map)
        {
            return (*T_enu_to_map + pInMap).asTPose();
        }
        return T_frame_wrt_map ? (pInMap - *T_frame_wrt_map).asTPose() : pInMap.asTPose();
    };

    for (const auto& [t, p] : finalizedTrajectory_)
    {
        if (!inRange(t))
        {
            continue;
        }
        if (T_enu_to_map)
        {
            if (const auto itEnu = finalizedTrajectoryEnu_.find(t);
                itEnu != finalizedTrajectoryEnu_.end())
            {
                ret.insert(t, itEnu->second);
                continue;
            }
        }
        ret.insert(t, toRequestedFrame(mrpt::poses::CPose3D(p)));
    }
    for (const auto& [t, frame_idx] : state_.stamp2frame_index)
    {
        const auto it = state_.last_estimated_states.find(frame_idx);
        if (it == state_.last_estimated_states.end() || !inRange(t))
        {
            continue;
        }
        ret.insert(t, toRequestedFrame(it->second.pose));
    }

    if (ret.empty())
    {
        return {};
    }
    return ret;
}

std::set<std::string> StateEstimationSmoother::known_odometry_frame_ids()
{
    auto lck = mrpt::lockHelper(stateMutex_);

    std::set<std::string> ret;
    for (const auto& [name, id] : state_.known_odom_frames.getDirectMap())
    {
        ret.insert(name);
    }

    return ret;
}

void StateEstimationSmoother::onNewObservation(const CObservation::ConstPtr& o)
{
    const ProfilerEntry tle(profiler_, "onNewObservation");

    ASSERT_(o);

    // IMU:
    if (auto obsIMU = std::dynamic_pointer_cast<const mrpt::obs::CObservationIMU>(o); obsIMU)
    {
        imu_bias_add_reading(*obsIMU);
        if (std::regex_match(
                o->sensorLabel,
                state_.do_process_imu_labels_re.get_regex(params_.do_process_imu_labels_re)))
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
                                    params_.do_process_odometry_labels_re)))
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
        // Same label filter the CObservationOdometry branch above uses (and
        // that StateEstimationSimple already applies to this branch): without
        // it, EVERY robot-pose observation reaching this module is fused,
        // whatever it is.
        if (!std::regex_match(
                o->sensorLabel, state_.do_process_odometry_labels_re.get_regex(
                                    params_.do_process_odometry_labels_re)))
        {
            MRPT_LOG_DEBUG_FMT(
                "Skipping robot pose reading labeled '%s' for not passing regex",
                o->sensorLabel.c_str());
            return;
        }

        // MOLA's offline dataset sources publish the reference trajectory as a
        // CObservationRobotPose labeled "ground_truth" (KITTI, KITTI-360,
        // MulRan, Paris-Luco). Fusing it would silently make every accuracy
        // number measured against that same trajectory meaningless, so it takes
        // an explicit opt-in.
        if (o->sensorLabel == "ground_truth" && !params_.fuse_ground_truth_label)
        {
            MRPT_LOG_ONCE_WARN(
                "Ignoring observations labeled 'ground_truth': fusing a dataset's own reference "
                "trajectory would invalidate any accuracy evaluation against it. Set "
                "'fuse_ground_truth_label: true' if that is really what you want.");
            return;
        }

        auto sensedSensorPose = obsPose->pose;
        if (obsPose->sensorPose != mrpt::poses::CPose3D())
        {
            sensedSensorPose =
                sensedSensorPose + mrpt::poses::CPose3DPDFGaussian(-obsPose->sensorPose);
        }

        // Use sensorLabel as frame_id if available (e.g. "wheel_odom", "visual_odom"),
        // so each source gets its own odometry frame in the factor graph.
        // Falls back to reference_frame_name for backward compatibility
        // (e.g. ground truth robot pose observations without a label).
        std::string frameId = params_.reference_frame_name;
        if (!obsPose->sensorLabel.empty())
        {
            // Normalize: replace illegal chars (keep alphanumeric, '_', '-', '/')
            std::string normalized;
            normalized.reserve(obsPose->sensorLabel.size());
            for (char c : obsPose->sensorLabel)
                normalized += (std::isalnum(static_cast<unsigned char>(c)) || c == '_' ||
                               c == '-' || c == '/')
                                  ? c
                                  : '_';

            // Enforce max length
            constexpr std::size_t MAX_FRAME_ID_LEN = 64;
            if (normalized.size() > MAX_FRAME_ID_LEN) normalized.resize(MAX_FRAME_ID_LEN);

            // Reject reserved names
            if (normalized == params_.vehicle_frame_name || normalized == params_.enu_frame_name)
            {
                MRPT_LOG_WARN_FMT(
                    "CObservationRobotPose sensorLabel '%s' is a reserved frame name; "
                    "falling back to reference_frame_name '%s'",
                    obsPose->sensorLabel.c_str(), params_.reference_frame_name.c_str());
            }
            else
            {
                frameId = normalized;
            }
        }

        this->fuse_pose(obsPose->timestamp, sensedSensorPose, frameId);
    }
    // GNSS source:
    else if (auto obsGPS = std::dynamic_pointer_cast<const mrpt::obs::CObservationGPS>(o); obsGPS)
    {
        if (std::regex_match(
                o->sensorLabel,
                state_.do_process_gnss_labels_re.get_regex(params_.do_process_gnss_labels_re)))
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

double StateEstimationSmoother::angular_const_vel_sigma(double dt) const
{
    // Random-walk term: how much the angular velocity may drift on its own over dt.
    const double sigmaModel = params_.sigma_random_walk_acceleration_angular * dt;

    // Measurement term: when gyro readings are fused as direct observations of w
    // (imu_angular_velocity_sigma > 0), two consecutive keyframes hold two
    // INDEPENDENT noisy measurements, so their difference already has a spread of
    // sqrt(2)*sigma no matter how close in time they are. The random-walk term
    // alone does not account for that and vanishes with dt, while this graph
    // routinely produces keyframe pairs only ~10 ms apart (a pose observation
    // landing just past the merge threshold of an existing IMU keyframe). There
    // the model term is several times tighter than the sensor noise it is being
    // asked to explain, and the resulting conflict cannot be absorbed by w alone:
    // this factor's residual is R_i*w_i - R_j*w_j, so the optimizer can only
    // reduce it by rotating the poses, which blows up the whole window (seen as
    // an IndeterminantLinearSystemException on an unrelated pose/velocity
    // variable). Adding both terms in quadrature keeps the factor consistent with
    // the data it competes against, and reduces to the pure random-walk model
    // when no gyro is fused.
    const double sigmaMeas = std::sqrt(2.0) * params_.imu_angular_velocity_sigma;

    return std::hypot(sigmaModel, sigmaMeas);
}

/// Implementation of Eqs (1),(4) in the MOLA RSS2019 paper.
void StateEstimationSmoother::addFactor(const AbsFactorConstVelKinematics& f)
{
    MRPT_LOG_DEBUG_STREAM(
        "[addFactor] FactorConstVelKinematics: " << f.from_kf << " ==> " << f.to_kf
                                                 << " dt=" << f.deltaTime);

    // Add const-vel factor to gtsam itself:
    double dt = f.deltaTime;

    // trick to easily handle queries on exactly an existing keyframe:
    if (dt == 0)
    {
        dt = 1e-5;
    }

    ASSERT_GT_(dt, 0.);

    // errors in constant vel:
    // The IMU measures the accelerations this model denies: with
    // preintegration its linear part is only kept, weak, so that links without
    // IMU data stay determined.
    const double linScale      = params_.imu_preintegration.enabled
                                     ? params_.imu_preintegration.linear_kinematics_sigma_scale
                                     : 1.0;
    const double std_lin_vel   = params_.sigma_random_walk_acceleration_linear * linScale;
    const double sigma_ang_vel = angular_const_vel_sigma(dt);

    if (dt > params_.time_between_frames_to_warning)
    {
        MRPT_LOG_WARN_FMT("Constant-velocity kinematics factor added for large dT=%.03f s.", dt);
    }

    // Emit into this link's own graph rather than straight into newFactors, so the
    // whole link can be withdrawn as a unit if a keyframe is later spliced between
    // its endpoints. Flushed into newFactors at the next update.
    auto& sink = state_.gtsam->pendingKinematic[GtsamImpl::link_key(f.from_kf, f.to_kf)];

    // 1) Add GTSAM factors for constant velocity model
    // -------------------------------------------------
    const auto kTi  = T(f.from_kf);
    const auto kTj  = T(f.to_kf);
    const auto kbVi = V(f.from_kf);
    const auto kbVj = V(f.to_kf);
    const auto kbWi = W(f.from_kf);
    const auto kbWj = W(f.to_kf);

    // See line 3 of eq (4) in the MOLA RSS2019 paper
    // Modify to use velocity in local frame: reuse FactorConstLocalVelocity
    // here too:
    sink.emplace_shared<mola::factors::FactorConstLocalVelocityPose>(
        kTi, kbVi, kTj, kbVj, gtsam::noiseModel::Isotropic::Sigma(3, std_lin_vel * dt));

    // \omega is in the body frame, we need a special factor to rotate it:
    // See line 4 of eq (4) in the MOLA RSS2019 paper.
    sink.emplace_shared<mola::factors::FactorConstLocalVelocityPose>(
        kTi, kbWi, kTj, kbWj, gtsam::noiseModel::Isotropic::Sigma(3, sigma_ang_vel));

    // 2) Add kinematics / numerical integration factor
    // ---------------------------------------------------
    auto noise_kinematicsPosition =
        gtsam::noiseModel::Isotropic::Sigma(3, params_.sigma_integrator_position * linScale);

    auto noise_kinematicsOrientation =
        gtsam::noiseModel::Isotropic::Sigma(3, params_.sigma_integrator_orientation);

    // Impl. line 2 of eq (1) in the MOLA RSS2019 paper
    sink.emplace_shared<mola::factors::FactorTrapezoidalIntegratorPose>(
        kTi, kbVi, kTj, kbVj, dt, noise_kinematicsPosition);

    // Impl. line 1 of eq (4) in the MOLA RSS2019 paper.
    sink.emplace_shared<mola::factors::FactorAngularVelocityIntegrationPose>(
        kTi, kbWi, kTj, dt, noise_kinematicsOrientation);
}

void StateEstimationSmoother::addFactor(const AbsFactorTricycleKinematics& f)
{
    MRPT_LOG_DEBUG_STREAM(
        "[addFactor] FactorTricycleKinematics: " << f.from_kf << " ==> " << f.to_kf
                                                 << " dt=" << f.deltaTime);

    // Add const-vel factor to gtsam itself:
    double dt = f.deltaTime;

    // trick to easily handle queries on exactly an existing keyframe:
    if (dt == 0)
    {
        dt = 1e-5;
    }

    ASSERT_GT_(dt, 0.);

    // errors in constant vel:
    const double std_lin_vel   = params_.sigma_random_walk_acceleration_linear;
    const double sigma_ang_vel = angular_const_vel_sigma(dt);

    if (dt > params_.time_between_frames_to_warning)
    {
        MRPT_LOG_WARN_FMT("Tricycle kinematics factor added for large dT=%.03f s.", dt);
    }

    // See the note in the ConstantVelocity overload: emit into this link's own
    // graph so it can be withdrawn as a unit on a later splice.
    auto& sink = state_.gtsam->pendingKinematic[GtsamImpl::link_key(f.from_kf, f.to_kf)];

    // 1) Add GTSAM factors for constant velocity model
    // -------------------------------------------------
    const auto kTi  = T(f.from_kf);
    const auto kTj  = T(f.to_kf);
    const auto kbVi = V(f.from_kf);
    const auto kbVj = V(f.to_kf);
    const auto kbWi = W(f.from_kf);
    const auto kbWj = W(f.to_kf);

    // See line 3 of eq (4) in the MOLA RSS2019 paper
    // Modify to use velocity in local frame: reuse FactorConstLocalVelocity
    // here too:
    sink.emplace_shared<mola::factors::FactorConstLocalVelocityPose>(
        kTi, kbVi, kTj, kbVj, gtsam::noiseModel::Isotropic::Sigma(3, std_lin_vel * dt));

    // \omega is in the body frame, we need a special factor to rotate it:
    // See line 4 of eq (4) in the MOLA RSS2019 paper.
    sink.emplace_shared<mola::factors::FactorConstLocalVelocityPose>(
        kTi, kbWi, kTj, kbWj, gtsam::noiseModel::Isotropic::Sigma(3, sigma_ang_vel));

    // In the tricycle model, body v_y must be zero:
    {
        const Eigen::Vector3d sigmas = {
            TRICYCLE_LARGE_SIGMAS, std_lin_vel * dt, TRICYCLE_LARGE_SIGMAS};

        sink.emplace_shared<gtsam::PriorFactor<gtsam::Point3>>(
            kbVj, gtsam::Point3::Zero(), gtsam::noiseModel::Diagonal::Sigmas(sigmas));
    }
    // In the tricycle model, body w_x,w_y must be zero:
    {
        const Eigen::Vector3d sigmas = {sigma_ang_vel, sigma_ang_vel, TRICYCLE_LARGE_SIGMAS};

        sink.emplace_shared<gtsam::PriorFactor<gtsam::Point3>>(
            kbWj, gtsam::Point3::Zero(), gtsam::noiseModel::Diagonal::Sigmas(sigmas));
    }

    // 2) Add kinematics / numerical integration factor
    // ---------------------------------------------------
    gtsam::Vector6 sigmas;
    const auto     sigmaPos   = params_.sigma_integrator_position;
    const auto     sigmaAngle = params_.sigma_integrator_orientation;
    sigmas << sigmaAngle, sigmaAngle, sigmaAngle, sigmaPos, sigmaPos, sigmaPos;

    auto noise_kinematics = gtsam::noiseModel::Diagonal::Sigmas(sigmas);

    // (To be written in a report/paper!)
    sink.emplace_shared<mola::factors::FactorTricycleKinematic>(
        kTi, kbVi, kbWi, kTj, dt, noise_kinematics);
}

void StateEstimationSmoother::delete_too_old_entries()
{
    // auto lck = mrpt::lockHelper(stateMutex_); // this is assumed to be acquired by caller

    // Remove really old entries in our bimap. GTSAM fixed lag handles removing actual factors.
    // Ages are measured against the newest stamp with timeDifference(), not by
    // comparing two toDouble() values: see add_kinematic_factor_between() for
    // why that helper cannot be used on a pre-UNIX-epoch timestamp. Through it,
    // the oldest keyframes of a zero-based clock read as the NEWEST and were
    // never pruned.
    const auto& newestStamp = state_.stamp2frame_index.getDirectMap().rbegin()->first;

    std::set<mrpt::Clock::time_point>                stamps_to_erase;
    std::set<frame_index_t>                          ids_to_erase;
    std::map<mrpt::Clock::time_point, frame_index_t> stamp2erasedFrame;
    for (const auto& [existing_t, frame_idx] : state_.stamp2frame_index)
    {
        if (mrpt::system::timeDifference(existing_t, newestStamp) > params_.sliding_window_length)
        {
            stamps_to_erase.insert(existing_t);
            ids_to_erase.insert(frame_idx);
            stamp2erasedFrame[existing_t] = frame_idx;
        }
    }
    // A keyframe leaving the window carries the last value the smoother wrote
    // back for it, which is its final one: no future measurement can reach it
    // any more. Record it before it is dropped, so an offline run can read the
    // smoothed trajectory instead of the front end's registered poses.
    if (params_.keep_finalized_trajectory)
    {
        // Recorded in ENU only once an estimated geo-reference has converged:
        // until then T_enu_to_map is still far from its final value, and the
        // latest one is the better choice for those keyframes.
        std::optional<mola::Georeferencing> georef;
        if (!params_.estimate_geo_reference || state_.geo_reference.has_value())
        {
            georef = georef_for_enu_poses_locked();
        }
        for (const auto& [t_erase, frame_idx] : stamp2erasedFrame)
        {
            const auto it = state_.last_estimated_states.find(frame_idx);
            if (it == state_.last_estimated_states.end())
            {
                continue;
            }
            finalizedTrajectory_.insert(t_erase, it->second.pose.asTPose());
            if (georef)
            {
                finalizedTrajectoryEnu_.insert(
                    t_erase, (georef->T_enu_to_map.mean + it->second.pose).asTPose());
            }
        }
    }

    for (const auto& t_erase : stamps_to_erase)
    {
        state_.stamp2frame_index.erase_by_key(t_erase);
    }
    for (const auto& idx : ids_to_erase)
    {
        state_.last_estimated_states.erase(idx);
    }

    // Forget the kinematic bookkeeping of links touching a marginalized keyframe.
    // The fixed-lag smoother drops those factors itself, so their indices must
    // never be queued for removal again (they would refer to slots GTSAM has
    // already reused or freed). This also bounds the maps' growth.
    const auto touchesErased = [&ids_to_erase](const GtsamImpl::frame_pair_t& k)
    { return ids_to_erase.count(k.first) != 0 || ids_to_erase.count(k.second) != 0; };

    for (auto it = state_.gtsam->flushedKinematic.begin();
         it != state_.gtsam->flushedKinematic.end();)
    {
        it = touchesErased(it->first) ? state_.gtsam->flushedKinematic.erase(it) : std::next(it);
    }
    for (auto it = state_.gtsam->pendingKinematic.begin();
         it != state_.gtsam->pendingKinematic.end();)
    {
        it = touchesErased(it->first) ? state_.gtsam->pendingKinematic.erase(it) : std::next(it);
    }
#if !GTSAM_USES_BOOST
    for (auto it = state_.gtsam->imuLinksAwaiting.begin();
         it != state_.gtsam->imuLinksAwaiting.end();)
    {
        it = touchesErased(*it) ? state_.gtsam->imuLinksAwaiting.erase(it) : std::next(it);
    }
#endif
}

// Creates a new frame index for timestamp t, or returns the existing one if close enough.
// This also is in charge of the complex task of finding nearby existing frames and adding the
// kinematic factors to ensure smooth motion estimation.
StateEstimationSmoother::frame_index_t StateEstimationSmoother::create_or_get_keyframe_by_timestamp(
    const mrpt::Clock::time_point& t, const std::optional<double>& overrideCloseEnough)
{
    auto lck = mrpt::lockHelper(stateMutex_);
    return create_or_get_keyframe_by_timestamp_locked(t, overrideCloseEnough);
}

StateEstimationSmoother::frame_index_t
    StateEstimationSmoother::create_or_get_keyframe_by_timestamp_locked(
        const mrpt::Clock::time_point& t, const std::optional<double>& overrideCloseEnough,
        bool pickClosest)
{
    const auto tle = mola::ProfilerEntry(profiler_, "create_or_get_keyframe_by_timestamp");

    const double threshold = overrideCloseEnough ? *overrideCloseEnough
                                                 : params_.min_time_difference_to_create_new_frame;

    // See if we have an existing frame index close enough to t: the first one
    // found, or the closest one if requested (with a permissive threshold both
    // neighbors may qualify):
    const auto                   closestPrior = find_before_after(t, true);
    std::optional<frame_index_t> best;
    double                       bestDt = threshold;
    for (const auto& it : {closestPrior.first, closestPrior.second})
    {
        if (it == state_.stamp2frame_index.getDirectMap().end())
        {
            continue;
        }
        const auto& [existing_t, frame_idx] = *it;

        const double dt = std::abs(mrpt::system::timeDifference(existing_t, t));

        if (dt < bestDt)
        {
            bestDt = dt;
            best   = frame_idx;
            if (!pickClosest)
            {
                break;
            }
        }
    }
    if (best)
    {
        return *best;
    }

    // Create a new one:
    const auto newFrameIdx = state_.next_frame_index++;
    state_.stamp2frame_index.insert(t, newFrameIdx);

    // Advance the real-time extrapolation reference only when this keyframe is
    // actually the newest one.
    //
    // A late (out-of-order) measurement must not drag the reference backwards:
    // get_current_extrapolated_stamp() would then jump into the past and creep
    // forward again, so the stamp of every published pose would sawtooth. Keeping
    // it at the newest stamp seen also means the reference follows the
    // lowest-latency source, instead of whichever source happened to arrive last.
    if (!state_.last_observation_stamp.has_value() || t > *state_.last_observation_stamp)
    {
        state_.last_observation_stamp           = t;
        state_.last_observation_wallclock_stamp = mrpt::Clock::now();
    }

    // Look for the closest existing frames, and create kinematic pairs if they don't exist yet:
    const auto closestPost = find_before_after(t, false);

    // Create new GTSAM symbols for this keyframe:
    initialize_new_frame(newFrameIdx, closestPost);

    // If this keyframe is being spliced BETWEEN two existing ones, the direct link
    // those two share must be withdrawn first. Otherwise its constraint stays in
    // the graph alongside the two links that replace it below, so the motion model
    // over that span is counted twice: it over-stiffens the window, biases the
    // states and makes the covariance over-confident.
    const auto& stamp2frameEnd = state_.stamp2frame_index.getDirectMap().end();
    if (closestPost.first != stamp2frameEnd && closestPost.second != stamp2frameEnd)
    {
        remove_kinematic_factor_between(closestPost.first->second, closestPost.second->second);
    }

    if (closestPost.first != state_.stamp2frame_index.getDirectMap().end())
    {
        const auto [t_before, idx_before] = *closestPost.first;
        MRPT_LOG_DEBUG_FMT(
            "[add_or_get_timestamp_frame_index] New frame created: idx=%zu, t_before=%f (idx=%zu)",
            static_cast<size_t>(newFrameIdx), mrpt::Clock::toDouble(t_before),
            static_cast<size_t>(idx_before));

        // Add kinematic factors:
        add_kinematic_factor_between(idx_before, newFrameIdx);
    }

    if (closestPost.second != state_.stamp2frame_index.getDirectMap().end())
    {
        const auto [t_after, idx_after] = *closestPost.second;
        MRPT_LOG_DEBUG_FMT(
            "[add_or_get_timestamp_frame_index] New frame created: idx=%zu, t_after=%f (idx=%zu)",
            static_cast<size_t>(newFrameIdx), mrpt::Clock::toDouble(t_after),
            static_cast<size_t>(idx_after));

        // Add kinematic factors:
        add_kinematic_factor_between(newFrameIdx, idx_after);
    }

    // Pruning (and, with it, finalized-trajectory recording) is deferred to
    // process_pending_gtsam_updates_locked(), once the solver has written back
    // this batch's estimate: see delete_too_old_entries() for why.

    return newFrameIdx;
}

// Creates or returns the existing ID, for an odometry frame_id:
StateEstimationSmoother::odometry_frameid_t StateEstimationSmoother::add_or_get_odom_frame_id(
    const std::string& frame_id_name)
{
    const auto tle = mola::ProfilerEntry(profiler_, "add_or_get_odom_frame_id");

    // F(0): is special, it's the reference frame ("map"), not a floating "odometry" frame
    if (frame_id_name == params_.reference_frame_name)
    {
        return REFERENCE_FRAME_ID;
    }

    ASSERT_NOT_EQUAL_(frame_id_name, params_.vehicle_frame_name);
    ASSERT_NOT_EQUAL_(frame_id_name, params_.enu_frame_name);

    // auto lck = mrpt::lockHelper(stateMutex_); // acquired by caller

    // Existing frame?
    if (auto it = state_.known_odom_frames.find_key(frame_id_name);
        it != state_.known_odom_frames.getDirectMap().end())
    {
        return it->second;
    }

    // New one: starting at "1" (0=reserved for "map").
    // Use a monotonic counter so IDs stay unique even if entries are ever removed.
    const auto newId = state_.next_odom_frame_id++;
    state_.known_odom_frames.insert(frame_id_name, newId);

    // Initialize gtsam symbol and prior factor for the new frame:
    const gtsam::Pose3 initFramePose = gtsam::Pose3::Identity();

    ASSERT_GE_(newId, 1);

    state_.gtsam->newValues.insert(symbol_T_map_to_odom_i_base + newId, initFramePose);
    state_.gtsam->newFactors.addPrior(
        symbol_T_map_to_odom_i_base + newId, initFramePose,
        gtsam::noiseModel::Isotropic::Sigma(6, INIT_ODOM_FRAME_POSE_SIGMA));

    // Until some source observes {map}, define it as this first odometry frame,
    // which is also where the weak prior above would leave it:
    if (!state_.gtsam->mapObserved && !state_.gtsam->mapAnchor &&
        !params_.link_first_pose_to_reference_origin_sigma.has_value())
    {
        state_.gtsam->add_anchor<gtsam::PriorFactor<gtsam::Pose3>>(
            state_.gtsam->mapAnchor, symbol_T_map_to_odom_i_base + newId, initFramePose,
            gtsam::noiseModel::Isotropic::Sigma(6, GAUGE_ANCHOR_SIGMA));
        state_.gtsam->mapAnchorFrame = newId;
    }
    else if (state_.gtsam->mapObserved && !state_.gtsam->orientationAnchorKey)
    {
        // {map} is already observed, maybe in position only:
        state_.gtsam->orientationAnchorKey = symbol_T_map_to_odom_i_base + newId;
        state_.gtsam->update_orientation_anchor();
    }

    return newId;
}

// Two sources must never share an odometry frame: each one has its own origin
// and drift, and the frame also holds the source's last raw pose and its
// prediction errors. A pose source keeps the name it asked for, since that is
// also the frame the front end asks its predictions in; a wheel odometry
// labeled the same goes to a frame of its own.
std::string StateEstimationSmoother::wheels_odometry_frame_locked(const std::string& odomName)
{
    if (!state_.wheels_odometry_frame)
    {
        std::string name = odomName;
        if (name != params_.reference_frame_name && state_.known_odom_frames.hasKey(name))
        {
            name = unused_wheels_frame_name(
                state_.known_odom_frames, odomName, params_.reference_frame_name);
            MRPT_LOG_WARN_FMT(
                "Wheel odometry '%s' shares its name with the frame of another pose source: it "
                "is fused in a frame of its own, '%s'.",
                odomName.c_str(), name.c_str());
        }
        state_.wheels_odometry_frame = name;
    }
    return *state_.wheels_odometry_frame;
}

void StateEstimationSmoother::move_wheels_odometry_frame_aside_locked(const std::string& frame_id)
{
    if (!state_.wheels_odometry_frame || *state_.wheels_odometry_frame != frame_id ||
        frame_id == params_.reference_frame_name)
    {
        return;
    }
    const auto it = state_.known_odom_frames.find_key(frame_id);
    if (it == state_.known_odom_frames.getDirectMap().end())
    {
        return;
    }
    // Factors refer to the numeric id, so renaming the entry moves the wheel
    // odometry with all its history, and frees the name for the pose source.
    // The new name is free before anything changes, so the insert cannot fail.
    const auto        id = it->second;
    const std::string newName =
        unused_wheels_frame_name(state_.known_odom_frames, frame_id, params_.reference_frame_name);
    state_.known_odom_frames.erase_by_key(frame_id);
    state_.known_odom_frames.insert(newName, id);
    state_.wheels_odometry_frame = newName;
    MRPT_LOG_WARN_FMT(
        "Pose source in frame '%s' shares that name with the wheel odometry: the latter "
        "is moved to a frame of its own, '%s'.",
        frame_id.c_str(), newName.c_str());
}

void StateEstimationSmoother::mark_map_observed_locked()
{
    auto& g = *state_.gtsam;
    // The map anchor also pinned the orientation of its odometry frame:
    // keep pinning what no source observes yet.
    if (g.mapAnchor && !g.orientationAnchorKey)
    {
        g.orientationAnchorKey = symbol_T_map_to_odom_i_base + g.mapAnchorFrame;
        g.update_orientation_anchor();
    }
    g.mapObserved = true;
    g.withdraw_anchor(g.mapAnchor);
}

void StateEstimationSmoother::mark_orientation_observed_locked(bool rollPitch, bool yaw)
{
    auto& g = *state_.gtsam;
    if ((!rollPitch || g.rollPitchObserved) && (!yaw || g.yawObserved))
    {
        return;
    }
    g.rollPitchObserved = g.rollPitchObserved || rollPitch;
    g.yawObserved       = g.yawObserved || yaw;
    g.update_orientation_anchor();
}

// The identity initial value set above can be arbitrarily far from the truth
// (an odometry frame may start anywhere in {map}). iSAM2 takes only one
// Gauss-Newton step per update, so a poor linearization point may diverge.
void StateEstimationSmoother::seed_odom_frame_locked(
    odometry_frameid_t frame_id_idx, frame_index_t kf, const mrpt::poses::CPose3D& poseInOdom)
{
    const auto key = symbol_T_map_to_odom_i_base + frame_id_idx;
    if (!state_.gtsam->newValues.exists(key))
    {
        return;  // Already in the smoother
    }
    const auto it = state_.last_estimated_states.find(kf);
    if (it == state_.last_estimated_states.end())
    {
        return;
    }

    // While {map} is this very frame, the reading is the keyframe pose itself:
    if (state_.gtsam->mapAnchor && state_.gtsam->mapAnchorFrame == frame_id_idx)
    {
        if (state_.gtsam->newValues.exists(T(kf)))
        {
            state_.gtsam->newValues.update(T(kf), mrpt::gtsam_wrappers::toPose3(poseInOdom));
            it->second.pose = poseInOdom;
        }
        return;
    }

    const auto T_map_to_odom = it->second.pose + (-poseInOdom);
    state_.gtsam->newValues.update(key, mrpt::gtsam_wrappers::toPose3(T_map_to_odom));
}

void StateEstimationSmoother::process_pending_gtsam_updates()
{
    auto lck = mrpt::lockHelper(stateMutex_);
    process_pending_gtsam_updates_locked();
}

void StateEstimationSmoother::process_pending_gtsam_updates_locked()
{
    const auto tle = mola::ProfilerEntry(profiler_, "process_pending_gtsam_updates");

    // Until some source relates {map} to the vehicle (e.g. while IMU readings
    // arrive ahead of the first pose), its position and heading are held only
    // by weak priors, which some GTSAM versions refuse to factorize. Wait for
    // one, for up to a window length so that the pending data stays bounded.
    if (!state_.gtsam->mapObserved && !state_.gtsam->mapAnchor &&
        !params_.link_first_pose_to_reference_origin_sigma.has_value() &&
        state_.gtsam->smoother->timestamps().empty())
    {
        double oldestStamp = std::numeric_limits<double>::infinity();
        double newestStamp = -std::numeric_limits<double>::infinity();
        for (const auto& [_, stamp] : state_.gtsam->newKeyStamps)
        {
            oldestStamp = std::min(oldestStamp, stamp);
            newestStamp = std::max(newestStamp, stamp);
        }
        if (newestStamp - oldestStamp < params_.sliding_window_length)
        {
            return;
        }
    }

    // Even if we have no new factors/values, do update the stamps of "persistent" variables:
    if (state_.last_observation_stamp.has_value())
    {
        const auto lastObservationStamp_sec = key_stamp_seconds(*state_.last_observation_stamp);

        state_.gtsam->newKeyStamps[symbol_T_enu_to_map] = lastObservationStamp_sec;
        for (const auto& [_, frameId] : state_.known_odom_frames)
        {
            state_.gtsam->newKeyStamps[symbol_T_map_to_odom_i_base + frameId] =
                lastObservationStamp_sec;
        }
    }

    if (NAVSTATE_PRINT_FG)
    {
        state_.gtsam->smoother->getFactors().print("EXISTING FACTORS:");
        state_.gtsam->newFactors.print("NEW FACTORS:");
        state_.gtsam->newValues.print("NEW VALUES:");
#if 0
        fg.saveGraph("fg.dot");
#endif
    }

    auto& smoother = *state_.gtsam->smoother;

    // A key must not leave the lag window in the same update that inserts it:
    // some GTSAM versions look marginalizable keys up in the Bayes tree before
    // adding the new ones, and throw. Keep such a key until the next update.
    {
        double newestStamp = -std::numeric_limits<double>::infinity();
        for (const auto& [_, stamp] : smoother.timestamps())
        {
            newestStamp = std::max(newestStamp, stamp);
        }
        for (const auto& [_, stamp] : state_.gtsam->newKeyStamps)
        {
            newestStamp = std::max(newestStamp, stamp);
        }
        const double oldestKept = newestStamp - params_.sliding_window_length;
        for (auto& [key, stamp] : state_.gtsam->newKeyStamps)
        {
            if (stamp < oldestKept && !smoother.getLinearizationPoint().exists(key))
            {
                stamp = oldestKept;
            }
        }
    }

#if !GTSAM_USES_BOOST
    if (params_.imu_preintegration.enabled)
    {
        attach_imu_factors_locked();
    }
#endif

    // Flush the per-link kinematic factors into newFactors, remembering the range
    // each link occupies so its iSAM2 indices can be recovered below.
    std::vector<std::pair<GtsamImpl::frame_pair_t, std::pair<size_t, size_t>>> flushedRanges;
    for (const auto& [linkKey, linkFactors] : state_.gtsam->pendingKinematic)
    {
        const size_t start = state_.gtsam->newFactors.size();
        for (const auto& f : linkFactors)
        {
            state_.gtsam->newFactors.push_back(f);
        }
        flushedRanges.emplace_back(linkKey, std::make_pair(start, state_.gtsam->newFactors.size()));
    }
    state_.gtsam->pendingKinematic.clear();

    // Update the smoother with pending factors/values:
    try
    {
        if (!state_.gtsam->newFactors.empty() || !state_.gtsam->newValues.empty() ||
            !state_.gtsam->newKeyStamps.empty() || !state_.gtsam->factorsToRemove.empty())
        {
            const auto tleUpd = mola::ProfilerEntry(profiler_, "process_pending.iSAM2.update");
            smoother.update(
                state_.gtsam->newFactors, state_.gtsam->newValues, state_.gtsam->newKeyStamps,
                state_.gtsam->factorsToRemove);

            // newFactorsIndices is 1-to-1 with the factors just passed in, so each
            // link's range maps straight onto the indices it was given. Keep them:
            // that is what makes a later splice able to remove this link again.
            const auto& newIdx = smoother.getISAM2Result().newFactorsIndices;
            for (const auto& [linkKey, range] : flushedRanges)
            {
                // Append: a link's IMU factor can be flushed after the rest of it.
                auto& indices = state_.gtsam->flushedKinematic[linkKey];
                for (size_t i = range.first; i < range.second && i < newIdx.size(); i++)
                {
                    indices.push_back(newIdx[i]);
                }
            }
            state_.gtsam->on_update_done(newIdx);
            state_.gtsam->factorsToRemove.clear();
        }

        // Optional: Perform extra internal iterations for better accuracy
        if (params_.additional_isam2_update_steps > 1)
        {
            const auto tleExtra =
                mola::ProfilerEntry(profiler_, "process_pending.iSAM2.extra_updates");
            for (unsigned int i = 1; i < params_.additional_isam2_update_steps; ++i)
            {
                smoother.update();
            }
        }
    }
    catch (const std::exception& e)
    {
        MRPT_LOG_ERROR_STREAM(
            "[process_pending_gtsam_updates] GTSAM update failed (factor graph may be "
            "underconstrained or ill-conditioned). Resetting smoother state. Exception:\n"
            << e.what());

        // smoother.update() is not strongly exception-safe: a failed call can
        // leave the iSAM2 Bayes tree internally inconsistent, so every later
        // update would keep throwing the same way forever. Merely discarding
        // the pending (not-yet-applied) factors/values/stamps does not fix
        // that. Rebuild the smoother from scratch instead.
        reset_locked();
        return;
    }

    // Print debug info:
    MRPT_LOG_DEBUG_STREAM(
        "[process_pending_gtsam_updates] After update: "
        << smoother.getFactors().size() << " factors, " << smoother.getFactors().nrFactors()
        << " nr factors. New factors=" << state_.gtsam->newFactors.size()
        << ", new values=" << state_.gtsam->newValues.size());

    // Per-type factor counts in the current sliding window. Useful to diagnose
    // which sensor streams are actively contributing to the smoother.
    if (isLoggingLevelVisible(mrpt::system::LVL_DEBUG))
    {
        size_t nPosePrior = 0, nPoseBetween = 0, nTwistPrior = 0;
        size_t nImuAttitude = 0, nImuGravity = 0, nGnss = 0;
        size_t nConstVel = 0, nTrapInt = 0, nAngVelInt = 0, nTricycle = 0;
        size_t nOther = 0, nNullSlots = 0;

        for (const auto& f : smoother.getFactors())
        {
            if (!f)
            {
                ++nNullSlots;  // unused slots (findUnusedFactorSlots=true)
                continue;
            }
            const auto* p = f.get();
            if (dynamic_cast<const gtsam::PriorFactor<gtsam::Pose3>*>(p))
                ++nPosePrior;
            else if (dynamic_cast<const gtsam::BetweenFactor<gtsam::Pose3>*>(p))
                ++nPoseBetween;
            else if (dynamic_cast<const gtsam::PriorFactor<gtsam::Point3>*>(p))
                ++nTwistPrior;
            else if (dynamic_cast<const mola::factors::Pose3RotationFactor*>(p))
                ++nImuAttitude;
            else if (dynamic_cast<const mola::factors::Pose3RotationYawBiasFactor*>(p))
                ++nImuAttitude;
            else if (dynamic_cast<const mola::factors::MeasuredGravityFactor*>(p))
                ++nImuGravity;
            else if (
                dynamic_cast<const mola::factors::FactorGnssMapEnu*>(p) ||
                dynamic_cast<const mola::factors::FactorGnssMapEnuConstVel*>(p))
                ++nGnss;
            else if (dynamic_cast<const mola::factors::FactorConstLocalVelocityPose*>(p))
                ++nConstVel;
            else if (dynamic_cast<const mola::factors::FactorTrapezoidalIntegratorPose*>(p))
                ++nTrapInt;
            else if (dynamic_cast<const mola::factors::FactorAngularVelocityIntegrationPose*>(p))
                ++nAngVelInt;
            else if (dynamic_cast<const mola::factors::FactorTricycleKinematic*>(p))
                ++nTricycle;
            else
                ++nOther;
        }

        MRPT_LOG_DEBUG_FMT(
            "[sliding-window factors] KFs=%zu  odom-frames=%zu | "
            "pose priors=%zu  pose between (odom)=%zu  twist priors=%zu | "
            "IMU attitude=%zu  IMU gravity=%zu  GNSS=%zu | "
            "kinematics: const-vel=%zu trap-int=%zu ang-vel-int=%zu tricycle=%zu | "
            "other=%zu  null-slots=%zu  total-active=%zu",
            state_.last_estimated_states.size(), state_.known_odom_frames.size(), nPosePrior,
            nPoseBetween, nTwistPrior, nImuAttitude, nImuGravity, nGnss, nConstVel, nTrapInt,
            nAngVelInt, nTricycle, nOther, nNullSlots, smoother.getFactors().nrFactors());
    }

    gtsam::Values optValues;
    try
    {
        const auto tleCalc = mola::ProfilerEntry(profiler_, "process_pending.calculateEstimate");
        optValues          = smoother.calculateEstimate();
    }
    catch (const std::exception& e)
    {
        MRPT_LOG_ERROR_STREAM(
            "[process_pending_gtsam_updates] GTSAM calculateEstimate() failed. "
            "Resetting smoother state. Exception:\n"
            << e.what());

        // Same reasoning as the update() catch above: calculateEstimate()
        // failing after a successful update() still means the smoother is in
        // a state we can't trust going forward, so rebuild it from scratch.
        reset_locked();
        return;
    }

#if !GTSAM_USES_BOOST
    // The latest bias estimate is the linearization point of new links.
    if (params_.imu_preintegration.enabled)
    {
        auto& g = *state_.gtsam;
        for (const auto& [kfIdx, kf] : state_.last_estimated_states)
        {
            if (!optValues.exists(Z(kfIdx)))
            {
                continue;
            }
            const double ts = key_stamp_seconds(state_.stamp2frame_index.inverse(kfIdx));
            if (ts >= g.lastBiasStamp)
            {
                g.lastBias      = optValues.at<gtsam::imuBias::ConstantBias>(Z(kfIdx));
                g.lastBiasStamp = ts;
            }
        }
    }
#endif

    // Retrieve the latest estimate and save it into "state_.last_estimated_state":
    {
        const auto tleWriteback = mola::ProfilerEntry(profiler_, "process_pending.writeback");
        for (auto& [kfIdx, kf] : state_.last_estimated_states)
        {
            // This is a FIXED-LAG smoother: once a keyframe leaves the lag
            // window it is marginalized out, and its T/V/W variables leave
            // optValues with it. Reading them unconditionally threw
            // "Attempting to at the key tNNN, which does not exist in the
            // Values", and since the caller treats that as fatal, the whole
            // run stopped mid-sequence and discarded every later
            // observation. Seen on Newer College 2020 (t3478, ~14% in) and
            // on GEODE Offroad1_gamma (t335, ~7.5% in).
            //
            // Skipping is the right behavior rather than merely the safe
            // one: such an entry already holds the last estimate the
            // smoother produced for it before marginalizing, which IS that
            // keyframe's final value. There is nothing newer to write.
            if (!optValues.exists(T(kfIdx)) || !optValues.exists(V(kfIdx)) ||
                !optValues.exists(W(kfIdx)))
            {
                continue;
            }

            const auto pose = optValues.at<gtsam::Pose3>(T(kfIdx));
            const auto linV = optValues.at<gtsam::Vector3>(V(kfIdx));
            const auto angV = optValues.at<gtsam::Vector3>(W(kfIdx));

            kf.pose  = mrpt::poses::CPose3D(mrpt::gtsam_wrappers::toTPose3D(pose));
            kf.twist = {linV.x(), linV.y(), linV.z(), angV.x(), angV.y(), angV.z()};

            if (params_.enforce_planar_motion)
            {
                enforce_planar_pose(kf.pose);
                enforce_planar_twist(kf.twist);
            }
        }
    }

    if (const auto& kf = state_.gtsam->lastYawBiasKf; kf && optValues.exists(B(*kf)))
    {
        auto& g = *state_.gtsam;

        // Its marginal is as costly as a solve: refreshed at most once per second of data.
        constexpr double sigmaPeriod = 1.0;
        double           sigma       = g.yawBiasEstimate ? g.yawBiasEstimate->second : 0;
        if (!g.yawBiasEstimate || g.lastYawBiasStamp - g.yawBiasSigmaStamp >= sigmaPeriod)
        {
            sigma               = std::sqrt(smoother.marginalCovariance(B(*kf))(0, 0));
            g.yawBiasSigmaStamp = g.lastYawBiasStamp;
        }
        g.yawBiasEstimate = {optValues.at<double>(B(*kf)), sigma};

        std::lock_guard<std::mutex> lck(lastYawBiasMutex_);
        lastYawBias_ = g.yawBiasEstimate;
    }

    state_.gtsam->mapObservedSolved         = state_.gtsam->mapObserved;
    state_.gtsam->mapPositionObservedSolved = state_.gtsam->mapPositionObserved;

    // Age out keyframes that fell outside the window, now that the writeback
    // above has given every one of them its final solved value. Done here,
    // once per batch, rather than per fuse_*() call: in async mode several
    // keyframes can be created before this function next runs, and finalizing
    // one on creation would record a stale (or altogether missing) pose
    // instead of this batch's solve.
    if (!state_.stamp2frame_index.empty())
    {
        delete_too_old_entries();
    }

    // Drive the predict-twist low-pass with the newest keyframe's optimized
    // twist, so the short-term prediction extrapolates a damped velocity rather
    // than the boundary node's noisy raw estimate.
    if (params_.predict_twist_filter_enabled && !state_.stamp2frame_index.empty())
    {
        const auto& latestIt = *state_.stamp2frame_index.getDirectMap().rbegin();
        if (const auto itKf = state_.last_estimated_states.find(latestIt.second);
            itKf != state_.last_estimated_states.end())
        {
            update_predict_twist_filter_locked(itKf->second.twist, latestIt.first);
        }
    }

    // Retrieve latest enu_to_map for geo-referencing:
    if (params_.estimate_geo_reference)
    {
        const auto tleGeoref = mola::ProfilerEntry(profiler_, "process_pending.marginals.georef");

        const auto T_enu_to_map     = optValues.at<gtsam::Pose3>(symbol_T_enu_to_map);
        const auto T_enu_to_map_cov = smoother.marginalCovariance(symbol_T_enu_to_map);

        auto& pdf = state_.last_estimated_frames[REFERENCE_FRAME_ID];

        pdf = mola::gtsam_compat::to_mrpt_se3_pdf(T_enu_to_map, T_enu_to_map_cov);

        // Convert info matrix to covariance
        if (!state_.stamp2frame_index.empty())
        {
            const auto& latestIt       = state_.stamp2frame_index.getDirectMap().rbegin();
            const auto  latestFrameIdx = latestIt->second;

            const auto poseCov =
                gtsam::Matrix6(state_.gtsam->smoother->marginalCovariance(T(latestFrameIdx)));
            const mrpt::math::CMatrixDouble66 cov =
                mola::gtsam_compat::to_mrpt_se3_pdf(
                    optValues.at<gtsam::Pose3>(T(latestFrameIdx)), poseCov)
                    .cov;

            // Check MAP->BASE_LINK position uncertainty (diagonal elements 0,1,2 are x,y,z)
            const double pos_sigma_x   = std::sqrt(cov(0, 0));
            const double pos_sigma_y   = std::sqrt(cov(1, 1));
            const double pos_sigma_z   = std::sqrt(cov(2, 2));
            const double max_pos_sigma = std::max({pos_sigma_x, pos_sigma_y, pos_sigma_z});

            // Check MAP->BASE_LINK orientation uncertainty (diagonal elements 3,4,5 are
            // yaw,pitch,roll)
            const double ori_sigma_yaw   = std::sqrt(cov(3, 3));
            const double ori_sigma_pitch = std::sqrt(cov(4, 4));
            const double ori_sigma_roll  = std::sqrt(cov(5, 5));
            const double max_ori_sigma_rad =
                std::max({ori_sigma_yaw, ori_sigma_pitch, ori_sigma_roll});
            const double max_ori_sigma_deg = mrpt::RAD2DEG(max_ori_sigma_rad);

            // Check ENU->MAP position uncertainty (diagonal elements 0,1,2 are x,y,z)
            const double em_pos_sigma_x = std::sqrt(pdf.cov(0, 0));
            const double em_pos_sigma_y = std::sqrt(pdf.cov(1, 1));
            const double em_pos_sigma_z = std::sqrt(pdf.cov(2, 2));
            const double em_max_pos_sigma =
                std::max({em_pos_sigma_x, em_pos_sigma_y, em_pos_sigma_z});

            // Check ENU->MAP orientation uncertainty (diagonal elements 3,4,5 are yaw,pitch,roll)
            const double em_ori_sigma_yaw   = std::sqrt(pdf.cov(3, 3));
            const double em_ori_sigma_pitch = std::sqrt(pdf.cov(4, 4));
            const double em_ori_sigma_roll  = std::sqrt(pdf.cov(5, 5));
            // Roll and pitch that nothing observes (no IMU) keep their level prior,
            // so they do not count; the azimuth always does:
            const auto observedOrNot = [](double sigma)
            { return sigma < UNOBSERVED_ENU2MAP_ANGLE_SIGMA ? sigma : 0.0; };
            const double em_max_ori_sigma_rad = std::max(
                {em_ori_sigma_yaw, observedOrNot(em_ori_sigma_pitch),
                 observedOrNot(em_ori_sigma_roll)});
            const double em_max_ori_sigma_deg = mrpt::RAD2DEG(em_max_ori_sigma_rad);

            // Check against thresholds
            const bool converged = (std::max(max_pos_sigma, em_max_pos_sigma) <=
                                    params_.convergence_max_position_sigma) &&
                                   (std::max(max_ori_sigma_deg, em_max_ori_sigma_deg) <=
                                    params_.convergence_max_orientation_sigma_deg);

            MRPT_LOG_DEBUG_FMT(
                "[process_pending_gtsam_updates] Has converged georeferencing? %s: "
                "pos_sigmas=(%.3f,%.3f,%.3f) m, "
                "enu_pos_sigmas=(%.3f,%.3f,%.3f) m, "
                "ori_sigmas=(%.2f,%.2f,%.2f) deg, "
                "enu_ori_sigmas=(%.2f,%.2f,%.2f) deg, "
                "thresholds=(%.3f m, %.2f deg)",
                converged ? "YES" : "NO", pos_sigma_x, pos_sigma_y, pos_sigma_z, em_pos_sigma_x,
                em_pos_sigma_y, em_pos_sigma_z, mrpt::RAD2DEG(ori_sigma_yaw),
                mrpt::RAD2DEG(ori_sigma_pitch), mrpt::RAD2DEG(ori_sigma_roll),
                mrpt::RAD2DEG(em_ori_sigma_yaw), mrpt::RAD2DEG(em_ori_sigma_pitch),
                mrpt::RAD2DEG(em_ori_sigma_roll), params_.convergence_max_position_sigma,
                params_.convergence_max_orientation_sigma_deg);

            // While pinned by its anchor, the azimuth sigma says nothing about the data:
            const bool azimuthPinned = static_cast<bool>(state_.gtsam->enuYawAnchor);
            if (converged && state_.tentative_geo_coord_reference.has_value() && !azimuthPinned)
            {
                state_.geo_reference.emplace();
                state_.geo_reference->geo_coord    = state_.tentative_geo_coord_reference.value();
                state_.geo_reference->T_enu_to_map = pdf;
                state_.estimated_georef_published  = false;  // so it's re-published
            }

            {
                std::lock_guard<std::mutex> lck(lastGeorefConvergenceMutex_);
                lastGeorefConvergence_ = GeorefConvergence{
                    std::max(max_pos_sigma, em_max_pos_sigma),
                    std::max(max_ori_sigma_deg, em_max_ori_sigma_deg), azimuthPinned,
                    state_.geo_reference.has_value()};
            }
        }
    }

    // retrieve odometry frames:
    {
        const auto tleOdomMarg =
            mola::ProfilerEntry(profiler_, "process_pending.marginals.odom_frames");
        // A source fused as relative increments constrains its T_map_to_odom_i
        // only through the one anchor factor, so that variable keeps the
        // initial alignment while the source drifts away from it. The live
        // map->odom of such a source is the correction that makes it agree
        // with the fused pose at the tail keyframe of its chain.
        const auto liveRelativeFrame =
            [this](odometry_frameid_t idx) -> std::optional<mrpt::poses::CPose3D>
        {
            std::optional<frame_index_t>        tailKf;
            std::optional<mrpt::poses::CPose3D> tailPoseInOdom;
            if (const auto it = state_.relative_pose_chains.find(idx);
                it != state_.relative_pose_chains.end() && it->second.last_pose_in_odom)
            {
                tailKf         = it->second.last_kf;
                tailPoseInOdom = it->second.last_pose_in_odom->mean;
            }
            else if (state_.wheels_odometry_frame && state_.last_wheels_odometry_at_kf)
            {
                const auto itName =
                    state_.known_odom_frames.find_key(*state_.wheels_odometry_frame);
                if (itName != state_.known_odom_frames.getDirectMap().end() &&
                    itName->second == idx)
                {
                    tailKf         = state_.last_wheels_odometry_kf;
                    tailPoseInOdom = mrpt::poses::CPose3D(*state_.last_wheels_odometry_at_kf);
                }
            }
            if (!tailKf || !tailPoseInOdom)
            {
                return {};
            }
            const auto itKf = state_.last_estimated_states.find(*tailKf);
            if (itKf == state_.last_estimated_states.end())
            {
                return {};
            }
            return itKf->second.pose + (-*tailPoseInOdom);
        };

        for (const auto& [_, odomFrameIdx] : state_.known_odom_frames)
        {
            const auto symbolOdom        = symbol_T_map_to_odom_i_base + odomFrameIdx;
            const auto T_map2_odom_i     = optValues.at<gtsam::Pose3>(symbolOdom);
            const auto T_map2_odom_i_cov = smoother.marginalCovariance(symbolOdom);

            auto& pdf = state_.last_estimated_frames[odomFrameIdx];

            const auto live = liveRelativeFrame(odomFrameIdx);
            pdf             = mola::gtsam_compat::to_mrpt_se3_pdf(T_map2_odom_i, T_map2_odom_i_cov);
            if (live)
            {
                pdf.mean = *live;
            }
        }
    }

    if (NAVSTATE_PRINT_FG)
    {
        smoother.getFactors().print("FG:\n");
        optValues.print("Optimized values:\n");
    }

    if (NAVSTATE_PRINT_FG_ERRORS)
    {
        smoother.getFactors().printErrors(
            optValues, "Errors for optimized values:", gtsam::DefaultKeyFormatter,
            [](const gtsam::Factor* /*factor*/, double whitenedError, size_t /*index*/)
            { return whitenedError > NAVSTATE_PRINT_FG_ERRORS_THRESHOLD; });
    }

    if (isLoggingLevelVisible(mrpt::system::LVL_DEBUG) && !smoother.getFactors().empty())
    {
        const double final_rmse = std::sqrt(
            smoother.getFactors().error(optValues) /
            static_cast<double>(smoother.getFactors().size()));

        MRPT_LOG_DEBUG_STREAM("[process_pending_gtsam_updates] iSAM2 final RMSE: " << final_rmse);
    }

    // Clear pending updates:
    state_.gtsam->newFactors.resize(0);
    state_.gtsam->newValues.clear();
    state_.gtsam->newKeyStamps.clear();

    // Geo-reference as of this solve. Keyframes finalized above came from the
    // previous one, so delete_too_old_entries() paired them with the previous
    // value of this field:
    if (params_.estimate_geo_reference)
    {
        state_.lastSolvedGeoref = compute_geo_reference_locked();
    }

    // With a fixed geo-reference, the GNSS gate waits until the vehicle is
    // localized, with the same criteria as has_converged_localization():
    if (!params_.estimate_geo_reference && params_.gnss_gate_chi2 > 0 &&
        !state_.gnssGate.relocalized && state_.geo_reference.has_value() &&
        state_.gtsam->mapObservedSolved && !state_.stamp2frame_index.empty())
    {
        try
        {
            const auto ns = get_latest_state_and_covariance(
                state_.stamp2frame_index.getDirectMap().rbegin()->second);
            mrpt::math::CMatrixDouble66 poseCov = ns.pose.cov_inv.inverse_LLt();
            inflate_pinned_orientation_cov_locked(poseCov);
            const double maxPos =
                std::sqrt(std::max({poseCov(0, 0), poseCov(1, 1), poseCov(2, 2)}));
            const double maxOriDeg =
                mrpt::RAD2DEG(std::sqrt(std::max({poseCov(3, 3), poseCov(4, 4), poseCov(5, 5)})));
            state_.gnssGate.relocalized =
                maxPos <= params_.convergence_max_position_sigma &&
                maxOriDeg <= params_.convergence_max_orientation_sigma_deg;
        }
        catch (const std::exception&)
        {
            // Not solvable yet: try again after the next solve.
        }
    }

    // Publish an immutable snapshot for the lock-free read path. Only built in
    // async mode: the synchronous path reads state_ directly and building the
    // snapshot would add marginals to every solve for no benefit.
    if (params_.async_backend)
    {
        if (auto snap = build_snapshot_locked())
        {
            fastPredictor_->set_snapshot(std::move(snap));
        }
    }

    // Check convergence for initialization from GNSS / georeferencing.
    // If we just converged and should publish geo-ref, do it now:
    if (params_.estimate_geo_reference && params_.publish_estimated_georef_on_convergence &&
        !state_.estimated_georef_published)
    {
        // Publish the estimated georeferencing
        publishEstimatedGeoreferencing();
    }
}

StateEstimationSmoother::pair_nearby_frame_iterators_t StateEstimationSmoother::find_before_after(
    const mrpt::Clock::time_point& t, bool allow_exact_match)
{
    const auto& stamp2frame = state_.stamp2frame_index.getDirectMap();

    using Iterator = std::map<mrpt::Clock::time_point, frame_index_t>::const_iterator;

    if (stamp2frame.empty())
    {
        return {stamp2frame.end(), stamp2frame.end()};
    }

    // upper_bound finds the first element whose key is > t. This is the 'after' element.
    Iterator after = stamp2frame.upper_bound(t);

    if (!allow_exact_match)
    {
        // Now determine the 'before' element.
        Iterator before;
        if (after == stamp2frame.begin())
        {
            // Case A: t is smaller than ALL keys.
            // No element before t. 'after' is the first element.
            before = stamp2frame.end();
        }
        else
        {
            // Case B: t is greater than or equal to some key(s).
            // 'before' is the element immediately preceding 'after'.
            before = std::prev(after);
        }

        // Now refine the 'after' iterator based on an exact match with the 'before' iterator.
        // If the 'before' element's key is exactly 't', then 'before' is the exact match.
        // The element *after* it is what upper_bound already found.
        // If the 'before' element's key is < 't', then 'before' is the correct predecessor.
        // Check if t is an exact match:
        if (before != stamp2frame.end() && before->first == t)
        {
            // An exact match for t exists.
            // 'before' is the exact match element.
            // The element BEFORE the exact match is its predecessor, if it exists.
            Iterator element_before_match =
                (before == stamp2frame.begin()) ? stamp2frame.end() : std::prev(before);

            // The element AFTER the exact match is what upper_bound already found (the current
            // 'after').
            return {element_before_match, after};
        }

        // General case: t lies strictly between 'before' and 'after' (or is smaller/larger than
        // all). The iterators 'before' and 'after' are correct as computed above.
        return {before, after};
    }

    // case: allow_exact_match is "true"

    // If 'after' is the beginning, t is smaller than all keys.
    if (after == stamp2frame.begin())
    {
        return {stamp2frame.end(), after};  // Case A: No 'before' element
    }

    // Otherwise, 'before' is the element immediately preceding 'after'.
    Iterator before = std::prev(after);

    // This pair correctly handles:
    // 1. t between K_i and K_j: before = K_i, after = K_j
    // 2. t exactly matches K_i: before = K_i, after = K_{i+1}
    // 3. t larger than all: before = K_max, after = end()

    return {before, after};
}

// Pick the closest of the two possible frames, or none if both iterators are end()
std::optional<StateEstimationSmoother::frame_index_t> StateEstimationSmoother::pick_closest(
    const StateEstimationSmoother::pair_nearby_frame_iterators_t& closestFrames,
    const mrpt::Clock::time_point&                                stamp) const
{
    const auto& [before, after] = closestFrames;

    // Both iterators are end(), no frames available
    if (before == state_.stamp2frame_index.getDirectMap().end() &&
        after == state_.stamp2frame_index.getDirectMap().end())
    {
        return std::nullopt;
    }

    // Only 'after' is available
    if (before == state_.stamp2frame_index.getDirectMap().end())
    {
        return after->second;
    }

    // Only 'before' is available
    if (after == state_.stamp2frame_index.getDirectMap().end())
    {
        return before->second;
    }

    // Both available, pick the closest by timestamp
    const double dtBefore = std::abs(mrpt::system::timeDifference(stamp, before->first));
    const double dtAfter  = std::abs(mrpt::system::timeDifference(stamp, after->first));

    return (dtBefore < dtAfter) ? before->second : after->second;
}

void StateEstimationSmoother::initialize_new_frame(
    frame_index_t id, const pair_nearby_frame_iterators_t& closestFrames)
{
    const auto stamp   = state_.stamp2frame_index.find_value(id)->second;
    const auto stamp_s = key_stamp_seconds(stamp);

    const auto closest_idx_opt = pick_closest(closestFrames, stamp);

    // Pick the data from closest frame as initial value, or 0 if none (first ever frame)
    gtsam::Pose3  pose        = gtsam::Pose3::Identity();
    gtsam::Point3 linVelocity = gtsam::Point3::Zero();
    gtsam::Point3 angVelocity = gtsam::Point3::Zero();

    // Initialize the state struct too:
    auto& newKfState = state_.last_estimated_states[id];

    if (closest_idx_opt.has_value())
    {
        const auto& kfState = state_.last_estimated_states.at(*closest_idx_opt);

        pose        = mrpt::gtsam_wrappers::toPose3(kfState.pose);
        linVelocity = {kfState.twist.vx, kfState.twist.vy, kfState.twist.vz};
        angVelocity = {kfState.twist.wx, kfState.twist.wy, kfState.twist.wz};

        // And initialize the state struct too:
        newKfState = kfState;

        // ...but seed only the pose/twist from the neighbor: this keyframe is brand
        // new and not connected to anything yet. Inheriting the neighbor's link set
        // would make add_kinematic_factor_between() believe a link already exists
        // and silently skip it, leaving this keyframe under-constrained.
        newKfState.kinematic_links_to.clear();
    }
    else
    {
        // This is the first ever frame.
        // Add weak prior factors for the system to be determinate.
        const auto priorNoise6 =
            gtsam::noiseModel::Isotropic::Sigma(6, FIRST_POSE_WEAK_PRIOR_SIGMA);

        state_.gtsam->newFactors.addPrior(T(id), pose, priorNoise6);

        const auto& tw = params_.initial_twist;
        state_.gtsam->newFactors.addPrior(
            V(id), gtsam::Vector3(tw.vx, tw.vy, tw.vz),
            gtsam::noiseModel::Isotropic::Sigma(3, params_.initial_twist_sigma_lin));

        state_.gtsam->newFactors.addPrior(
            W(id), gtsam::Vector3(tw.wx, tw.wy, tw.wz),
            gtsam::noiseModel::Isotropic::Sigma(3, params_.initial_twist_sigma_ang));

        if (params_.link_first_pose_to_reference_origin_sigma.has_value())
        {
            state_.gtsam->newFactors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
                T(id), gtsam::Pose3::Identity(),
                gtsam::noiseModel::Isotropic::Sigma(
                    6, *params_.link_first_pose_to_reference_origin_sigma));
        }
    }

    // T: Pose
    state_.gtsam->newValues.insert(T(id), pose);
    state_.gtsam->newKeyStamps[T(id)] = stamp_s;

    // V: Lin Velocity
    state_.gtsam->newValues.insert(V(id), linVelocity);
    state_.gtsam->newKeyStamps[V(id)] = stamp_s;

    // W: Ang Velocity
    state_.gtsam->newValues.insert(W(id), angVelocity);
    state_.gtsam->newKeyStamps[W(id)] = stamp_s;

#if !GTSAM_USES_BOOST
    // Z: IMU biases, seeded with the latest estimate. The first keyframe gets
    // a prior; every later one is chained by the random walk of its links.
    if (params_.imu_preintegration.enabled)
    {
        state_.gtsam->newValues.insert(Z(id), state_.gtsam->lastBias);
        state_.gtsam->newKeyStamps[Z(id)] = stamp_s;
        if (!closest_idx_opt.has_value())
        {
            const auto&    ip = params_.imu_preintegration;
            gtsam::Vector6 sig;
            sig << gtsam::Vector3::Constant(ip.initial_accel_bias_sigma),
                gtsam::Vector3::Constant(ip.initial_gyro_bias_sigma);
            state_.gtsam->newFactors.addPrior(
                Z(id), gtsam::imuBias::ConstantBias(), gtsam::noiseModel::Diagonal::Sigmas(sig));
        }
    }
#endif

    // Add planar constraints:
    if (params_.enforce_planar_motion)
    {
        const auto planar_z_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6(
            PLANAR_Z_SIGMA, PLANAR_Z_SIGMA, PLANAR_XY_SIGMA,  // rx≈0, ry≈0, rz free
            PLANAR_XY_SIGMA, PLANAR_XY_SIGMA, PLANAR_Z_SIGMA)  // tx free, ty free, tz≈0
        );

        state_.gtsam->newFactors.addPrior(T(id), gtsam::Pose3::Identity(), planar_z_noise);
    }
}

void StateEstimationSmoother::add_kinematic_factor_between(
    const frame_index_t from, const frame_index_t to)  // NOLINT
{
    ASSERT_NOT_EQUAL_(from, to);

    // Take note of already connected frames to avoid duplications.
    // Check both sets first, then insert both, to keep them in sync.
    // --------------------------------------------------------------------
    auto& fromKf = state_.last_estimated_states.at(from);
    auto& toKf   = state_.last_estimated_states.at(to);

    if (fromKf.kinematic_links_to.count(to) != 0 || toKf.kinematic_links_to.count(from) != 0)
    {
        return;  // already added
    }

    fromKf.kinematic_links_to.insert(to);
    toKf.kinematic_links_to.insert(from);

    // Dispatch to factor generation:
    // --------------------------------------------------------------------
    // Elapsed time between the two keyframes, taken on the clock's own signed
    // tick type. Never as a difference of mrpt::Clock::toDouble() values: that
    // helper rebases onto the UNIX epoch with an UNSIGNED subtraction, so a
    // timestamp even a millisecond before it wraps to ~1.8e12 s instead of a
    // small negative number, and every difference taken through it is garbage.
    // Datasets whose clock starts at zero do produce such timestamps.
    const double dt = mrpt::system::timeDifference(
        state_.stamp2frame_index.inverse(from), state_.stamp2frame_index.inverse(to));

    switch (params_.kinematic_model)
    {
        case KinematicModel::ConstantVelocity:
        {
            AbsFactorConstVelKinematics f;
            f.from_kf   = from;
            f.to_kf     = to;
            f.deltaTime = dt;
            addFactor(f);
        }
        break;

        case KinematicModel::Tricycle:
        {
            AbsFactorTricycleKinematics f;
            f.from_kf   = from;
            f.to_kf     = to;
            f.deltaTime = dt;
            addFactor(f);
        }
        break;

        default:
            THROW_EXCEPTION("Invalid kinematic_model value");
    }

#if !GTSAM_USES_BOOST
    if (params_.imu_preintegration.enabled)
    {
        // The bias random walk goes with the link from its creation, so no bias
        // variable is ever left unconstrained; the preintegration factor itself
        // is attached once the raw readings cover the link's interval.
        const auto&    ip  = params_.imu_preintegration;
        const double   adt = std::max(std::abs(dt), 1e-3);
        gtsam::Vector6 sig;
        sig << gtsam::Vector3::Constant(ip.accel_bias_random_walk * std::sqrt(adt)),
            gtsam::Vector3::Constant(ip.gyro_bias_random_walk * std::sqrt(adt));
        auto& sink = state_.gtsam->pendingKinematic[GtsamImpl::link_key(from, to)];
        sink.emplace_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(
            Z(from), Z(to), gtsam::imuBias::ConstantBias(),
            gtsam::noiseModel::Diagonal::Sigmas(sig));
        state_.gtsam->imuLinksAwaiting.insert(GtsamImpl::link_key(from, to));
    }
#endif
}

size_t StateEstimationSmoother::count_const_vel_factors_for_testing() const
{
    auto lck = mrpt::lockHelper(stateMutex_);

    if (!state_.gtsam->smoother.has_value())
    {
        return 0;
    }

    size_t n = 0;
    for (const auto& f : state_.gtsam->smoother->getFactors())
    {
        // findUnusedFactorSlots=true leaves null slots behind for removed factors.
        if (!f)
        {
            continue;
        }
        if (dynamic_cast<const mola::factors::FactorConstLocalVelocityPose*>(f.get()) != nullptr)
        {
            n++;
        }
    }
    return n;
}

std::set<std::pair<mrpt::Clock::time_point, mrpt::Clock::time_point>>
    StateEstimationSmoother::const_vel_factor_links_for_testing() const
{
    auto lck = mrpt::lockHelper(stateMutex_);

    std::set<std::pair<mrpt::Clock::time_point, mrpt::Clock::time_point>> links;

    if (!state_.gtsam->smoother.has_value())
    {
        return links;
    }

    for (const auto& f : state_.gtsam->smoother->getFactors())
    {
        // findUnusedFactorSlots=true leaves null slots behind for removed factors.
        if (!f)
        {
            continue;
        }
        const auto* fc = dynamic_cast<const mola::factors::FactorConstLocalVelocityPose*>(f.get());
        if (fc == nullptr)
        {
            continue;
        }
        // keys() layout is {kTi, kWi, kTj, kWj}: pose keys are at indices 0 and 2,
        // shared with both the linear- and angular-velocity factor of the same link.
        const auto& keys    = fc->keys();
        const auto  idxFrom = static_cast<frame_index_t>(gtsam::Symbol(keys.at(0)).index());
        const auto  idxTo   = static_cast<frame_index_t>(gtsam::Symbol(keys.at(2)).index());

        links.emplace(
            state_.stamp2frame_index.inverse(idxFrom), state_.stamp2frame_index.inverse(idxTo));
    }
    return links;
}

void StateEstimationSmoother::buffer_raw_imu_locked(const mrpt::obs::CObservationIMU& imu)
{
#if !GTSAM_USES_BOOST
    using namespace mrpt::obs;
    if (!imu.has(IMU_X_ACC) || !imu.has(IMU_Y_ACC) || !imu.has(IMU_Z_ACC) || !imu.has(IMU_WX) ||
        !imu.has(IMU_WY) || !imu.has(IMU_WZ))
    {
        return;
    }
    auto&       g  = *state_.gtsam;
    const auto& ip = params_.imu_preintegration;
    if (!g.rawImuLabelRe)
    {
        g.rawImuLabelRe.emplace(ip.sensor_label_re);
    }
    if (!std::regex_match(imu.sensorLabel, *g.rawImuLabelRe))
    {
        return;
    }
    // One IMU: the first one seen that matches.
    if (g.rawImuLabel.empty())
    {
        g.rawImuLabel = imu.sensorLabel;
        auto p        = gtsam::PreintegrationParams::MakeSharedU(ip.gravity);
        p->setGyroscopeCovariance(mrpt::square(ip.gyro_noise_density) * gtsam::I_3x3);
        p->setAccelerometerCovariance(mrpt::square(ip.accel_noise_density) * gtsam::I_3x3);
        p->setIntegrationCovariance(mrpt::square(ip.integration_sigma) * gtsam::I_3x3);
        p->setBodyPSensor(mrpt::gtsam_wrappers::toPose3(imu.sensorPose));
        g.pimParams = p;
        MRPT_LOG_INFO_STREAM(
            "[imu_preintegration] Preintegrating IMU '"
            << imu.sensorLabel << "', pose on vehicle: " << imu.sensorPose);
    }
    if (imu.sensorLabel != g.rawImuLabel)
    {
        return;
    }
    const double t = key_stamp_seconds(imu.timestamp);
    if (!g.rawImu.empty() && t <= g.rawImu.back().t)
    {
        return;  // repeated or out of order
    }
    GtsamImpl::RawImu r;
    r.t   = t;
    r.acc = {imu.get(IMU_X_ACC), imu.get(IMU_Y_ACC), imu.get(IMU_Z_ACC)};
    r.gyr = {imu.get(IMU_WX), imu.get(IMU_WY), imu.get(IMU_WZ)};
    g.rawImu.push_back(r);

    // Readings older than the window can no longer be part of any link.
    const double keepSince = t - params_.sliding_window_length - 2.0;
    while (!g.rawImu.empty() && g.rawImu.front().t < keepSince)
    {
        g.rawImu.pop_front();
    }
#else
    (void)imu;
#endif
}

void StateEstimationSmoother::attach_imu_factors_locked()
{
#if !GTSAM_USES_BOOST
    auto& g = *state_.gtsam;
    if (!g.pimParams || g.rawImu.empty())
    {
        return;
    }
    const auto&  ip        = params_.imu_preintegration;
    const double newestImu = g.rawImu.back().t;

    for (auto it = g.imuLinksAwaiting.begin(); it != g.imuLinksAwaiting.end();)
    {
        const auto [a, b] = *it;
        const auto ia     = state_.last_estimated_states.find(a);
        const auto ib     = state_.last_estimated_states.find(b);
        if (ia == state_.last_estimated_states.end() || ib == state_.last_estimated_states.end() ||
            ia->second.kinematic_links_to.count(b) == 0)
        {
            it = g.imuLinksAwaiting.erase(it);  // the link is gone
            continue;
        }
        frame_index_t i  = a;
        frame_index_t j  = b;
        double        ti = key_stamp_seconds(state_.stamp2frame_index.inverse(a));
        double        tj = key_stamp_seconds(state_.stamp2frame_index.inverse(b));
        if (tj < ti)
        {
            std::swap(i, j);
            std::swap(ti, tj);
        }
        if (newestImu < tj)
        {
            ++it;  // its readings have not all arrived yet
            continue;
        }

        // Zero-order hold: each reading holds until the next one. Start at the
        // last reading at or before ti (or the first one, if close enough).
        auto first = std::upper_bound(
            g.rawImu.begin(), g.rawImu.end(), ti,
            [](double v, const GtsamImpl::RawImu& r) { return v < r.t; });
        bool ok = true;
        if (first == g.rawImu.begin())
        {
            ok = (first->t - ti) <= ip.max_reading_gap;
        }
        else
        {
            --first;
        }

        gtsam::PreintegratedImuMeasurements pim(g.pimParams, g.lastBias);
        double                              tc = ti;
        for (auto k = first; ok && k != g.rawImu.end() && tc < tj; ++k)
        {
            const auto   next  = std::next(k);
            const double tNext = (next != g.rawImu.end()) ? std::min(next->t, tj) : tj;
            if (next != g.rawImu.end() && next->t - k->t > ip.max_reading_gap)
            {
                ok = false;
                break;
            }
            if (tNext > tc)
            {
                pim.integrateMeasurement(k->acc, k->gyr, tNext - tc);
            }
            tc = tNext;
        }

        if (ok && tj - ti >= ip.min_link_duration && std::abs(pim.deltaTij() - (tj - ti)) < 1e-6)
        {
            gtsam::Matrix9 floor    = gtsam::Matrix9::Zero();
            floor.block<3, 3>(0, 0) = mrpt::square(ip.min_sigma_rotation) * gtsam::I_3x3;
            floor.block<3, 3>(3, 3) = mrpt::square(ip.min_sigma_position) * gtsam::I_3x3;
            floor.block<3, 3>(6, 6) = mrpt::square(ip.min_sigma_velocity) * gtsam::I_3x3;
            g.pendingKinematic[*it].emplace_shared<mola::factors::FactorImuPreintegrationBodyVel>(
                T(i), V(i), T(j), V(j), Z(i), pim, floor);
        }
        it = g.imuLinksAwaiting.erase(it);
    }
#endif
}

void StateEstimationSmoother::remove_kinematic_factor_between(
    const frame_index_t from, const frame_index_t to)  // NOLINT
{
    // Undo the link bookkeeping, so the pair can legitimately be re-linked later.
    if (auto it = state_.last_estimated_states.find(from); it != state_.last_estimated_states.end())
    {
        it->second.kinematic_links_to.erase(to);
    }
    if (auto it = state_.last_estimated_states.find(to); it != state_.last_estimated_states.end())
    {
        it->second.kinematic_links_to.erase(from);
    }

    const auto key = GtsamImpl::link_key(from, to);

#if !GTSAM_USES_BOOST
    state_.gtsam->imuLinksAwaiting.erase(key);
#endif

    // Still pending: it never reached the solver, so dropping it costs nothing.
    // A link can be partly pending and partly flushed (an IMU factor attached
    // after the rest), so both are handled.
    if (auto it = state_.gtsam->pendingKinematic.find(key);
        it != state_.gtsam->pendingKinematic.end())
    {
        state_.gtsam->pendingKinematic.erase(it);
    }

    // Already inside the smoother: queue its factors for removal at the next update.
    if (auto it = state_.gtsam->flushedKinematic.find(key);
        it != state_.gtsam->flushedKinematic.end())
    {
        auto& rm = state_.gtsam->factorsToRemove;
        rm.insert(rm.end(), it->second.begin(), it->second.end());
        state_.gtsam->flushedKinematic.erase(it);
    }
}

std::shared_ptr<const Snapshot> StateEstimationSmoother::build_snapshot_locked() const
{
    if (state_.stamp2frame_index.empty())
    {
        return nullptr;
    }

    auto snap = std::make_shared<Snapshot>();

    // Anchor: the newest keyframe's optimized state + covariance.
    const auto& latestIt      = state_.stamp2frame_index.getDirectMap().rbegin();
    const auto  latestStamp   = latestIt->first;
    const auto  latestFrameId = latestIt->second;
    try
    {
        snap->anchor = get_latest_state_and_covariance(latestFrameId);
    }
    catch (const std::exception&)
    {
        // Graph still under-constrained: no usable snapshot yet.
        return nullptr;
    }
    snap->anchorStamp = latestStamp;

    // Extrapolate a low-pass-filtered velocity, not the boundary keyframe's raw
    // (noisy) twist, so the front end's motion prior stays smooth.
    if (params_.predict_twist_filter_enabled && state_.filtered_predict_twist.has_value())
    {
        snap->anchor.twist = *state_.filtered_predict_twist;
    }

    // Odometry-frame transforms (numeric id >= 1) and the name<->id mapping.
    snap->frameNames = state_.known_odom_frames;
    for (const auto& [id, pdf] : state_.last_estimated_frames)
    {
        if (id == REFERENCE_FRAME_ID)
        {
            continue;
        }
        snap->frameTransforms[id] = pdf;
    }

    // Each source's own last raw pose in {odom_i}, for the frame-local path.
    for (const auto& [id, raw] : state_.last_raw_pose_by_source)
    {
        snap->lastRawPoseBySource[id] = Snapshot::RawSourcePose{raw.stamp, raw.pose};
    }
    for (const auto& [id, res] : state_.prediction_residuals)
    {
        if (res.count >= MIN_PREDICTION_RESIDUALS)
        {
            snap->predictionVarRate[id] = res.varRate;
        }
    }

    snap->geoReference        = state_.geo_reference;
    snap->currentGeoReference = get_geo_reference_locked();

    // Convergence flag, mode-aware, mirroring has_converged_localization().
    bool converged = false;
    if (state_.geo_reference.has_value())
    {
        if (params_.estimate_geo_reference)
        {
            converged = true;
        }
        else
        {
            mrpt::math::CMatrixDouble66 poseCov = snap->anchor.pose.cov_inv.inverse_LLt();
            inflate_pinned_orientation_cov_locked(poseCov);
            const double maxPos =
                std::sqrt(std::max({poseCov(0, 0), poseCov(1, 1), poseCov(2, 2)}));
            const double maxOriDeg =
                mrpt::RAD2DEG(std::sqrt(std::max({poseCov(3, 3), poseCov(4, 4), poseCov(5, 5)})));
            converged = state_.gtsam->mapObservedSolved &&
                        maxPos <= params_.convergence_max_position_sigma &&
                        maxOriDeg <= params_.convergence_max_orientation_sigma_deg;
        }
    }
    snap->localizationConverged = converged;
    snap->mapPositionObserved   = state_.gtsam->mapPositionObservedSolved;

    snap->valid = true;
    return snap;
}

NavState StateEstimationSmoother::get_latest_state_and_covariance(const frame_index_t idx) const
{
    const auto& frame = state_.last_estimated_states.at(idx);

    NavState ns;

    // Pose:
    ns.pose.mean       = frame.pose;
    const auto poseCov = gtsam::Matrix6(state_.gtsam->smoother->marginalCovariance(T(idx)));
    if (poseCov.determinant() <= 0)
    {
        THROW_EXCEPTION_FMT(
            "get_latest_state_and_covariance: pose marginal covariance for KF %u is "
            "numerically degenerate (det=%g). The factor graph may be under-constrained.",
            static_cast<unsigned>(idx), poseCov.determinant());
    }
    ns.pose.cov_inv =
        mola::gtsam_compat::to_mrpt_se3_pdf(mrpt::gtsam_wrappers::toPose3(frame.pose), poseCov)
            .cov.inverse_LLt();

    // Twist:
    ns.twist        = frame.twist;
    const auto vCov = gtsam::Matrix3(state_.gtsam->smoother->marginalCovariance(V(idx)));
    const auto wCov = gtsam::Matrix3(state_.gtsam->smoother->marginalCovariance(W(idx)));

    gtsam::Matrix6 twCov    = gtsam::Matrix6::Zero();
    twCov.block<3, 3>(0, 0) = vCov;
    twCov.block<3, 3>(3, 3) = wCov;

    if (twCov.determinant() <= 0)
    {
        THROW_EXCEPTION_FMT(
            "get_latest_state_and_covariance: twist marginal covariance for KF %u is "
            "numerically degenerate (det=%g). The factor graph may be under-constrained.",
            static_cast<unsigned>(idx), twCov.determinant());
    }
    ns.twist_inv_cov = twCov.inverse();

    return ns;
}

void StateEstimationSmoother::update_prediction_residuals_locked(
    const odometry_frameid_t src, const mrpt::Clock::time_point& t,
    const mrpt::poses::CPose3D& pose)
{
    if (!params_.predict_covariance_from_residuals || !state_.filtered_predict_twist.has_value())
    {
        return;
    }
    const auto itRaw = state_.last_raw_pose_by_source.find(src);
    if (itRaw == state_.last_raw_pose_by_source.end())
    {
        return;
    }
    const double dt = mrpt::system::timeDifference(itRaw->second.stamp, t);
    if (dt <= 1e-3 || dt > params_.max_time_to_use_velocity_model)
    {
        return;
    }

    // What the frame-local prediction would have said for this reading:
    const auto predicted =
        itRaw->second.pose.mean + body_twist_delta(params_, *state_.filtered_predict_twist, dt);
    const auto err = pose - predicted;
    const auto rot = mrpt::poses::Lie::SO<3>::log(err.getRotationMatrix());

    const std::array<double, 6> r = {err.x(), err.y(), err.z(), rot[0], rot[1], rot[2]};

    std::array<double, 6> q{};
    for (size_t i = 0; i < r.size(); i++)
    {
        q[i] = mrpt::square(r[i] / dt);
    }

    auto& s = state_.prediction_residuals[src];

    // Warm-up, while these errors are not used yet: seed the running mean with
    // their per-axis median, so a start-up transient (e.g. a source that is
    // already moving when the estimator assumes rest) cannot set it.
    if (s.count < MIN_PREDICTION_RESIDUALS)
    {
        s.warmup.push_back(q);
        s.count++;
        if (s.count == MIN_PREDICTION_RESIDUALS)
        {
            for (size_t i = 0; i < q.size(); i++)
            {
                std::vector<double> v;
                v.reserve(s.warmup.size());
                for (const auto& w : s.warmup)
                {
                    v.push_back(w[i]);
                }
                const auto mid = v.begin() + static_cast<std::ptrdiff_t>(v.size() / 2);
                std::nth_element(v.begin(), mid, v.end());
                s.varRate[i] = *mid;
            }
            s.warmup.clear();
            s.warmup.shrink_to_fit();
        }
        return;
    }

    const double tau   = std::max(1e-3, params_.predict_residual_time_const);
    const double alpha = 1.0 - std::exp(-dt / tau);
    for (size_t i = 0; i < q.size(); i++)
    {
        // One gross error (a failed registration) must not dominate the mean:
        const double qi = std::min(q[i], 25.0 * std::max(s.varRate[i], MIN_PREDICTION_VAR_RATE));
        s.varRate[i] += alpha * (qi - s.varRate[i]);
    }
    s.count++;
}

void StateEstimationSmoother::update_predict_twist_filter_locked(
    const mrpt::math::TTwist3D& rawTwist, const mrpt::Clock::time_point& stamp)
{
    if (!params_.predict_twist_filter_enabled)
    {
        return;
    }

    // Bootstrap (first sample, or after a reset): adopt the raw value.
    if (!state_.filtered_predict_twist.has_value() ||
        !state_.filtered_predict_twist_stamp.has_value())
    {
        state_.filtered_predict_twist       = rawTwist;
        state_.filtered_predict_twist_stamp = stamp;
        return;
    }

    const double dt = mrpt::system::timeDifference(*state_.filtered_predict_twist_stamp, stamp);

    // Out-of-order or same-stamp solve (the solver can re-run several times
    // while the newest keyframe stamp is unchanged): keep the smoothed state as
    // is. Overwriting it with rawTwist here would wipe the EMA history and let
    // the raw boundary twist through, i.e. exactly the jitter this filter damps.
    if (dt <= 0)
    {
        return;
    }

    // dt-aware exponential moving average, so a variable solve rate keeps a
    // constant effective time constant.
    const double tau   = std::max(1e-3, params_.predict_twist_filter_time_const);
    const double alpha = 1.0 - std::exp(-dt / tau);

    auto& f = *state_.filtered_predict_twist;
    f.vx += alpha * (rawTwist.vx - f.vx);
    f.vy += alpha * (rawTwist.vy - f.vy);
    f.vz += alpha * (rawTwist.vz - f.vz);
    f.wx += alpha * (rawTwist.wx - f.wx);
    f.wy += alpha * (rawTwist.wy - f.wy);
    f.wz += alpha * (rawTwist.wz - f.wz);

    state_.filtered_predict_twist_stamp = stamp;
}

std::optional<mrpt::poses::CPose3DPDFGaussian> StateEstimationSmoother::estimated_T_enu_to_map()
    const
{
    auto lck = mrpt::lockHelper(stateMutex_);
    return estimated_T_enu_to_map_locked();
}

std::optional<std::pair<double, double>> StateEstimationSmoother::estimated_imu_yaw_bias() const
{
    std::lock_guard<std::mutex> lck(lastYawBiasMutex_);
    return lastYawBias_;
}

#if defined(MOLA_KERNEL_NAVSTATE_FILTER_HAS_GEO_REFERENCE)
std::optional<mola::Georeferencing> StateEstimationSmoother::get_geo_reference() const
{
    auto lck = mrpt::lockHelper(stateMutex_);
    return get_geo_reference_locked();
}
#endif

std::optional<mola::Georeferencing> StateEstimationSmoother::get_geo_reference_locked() const
{
    // Called with stateMutex_ already held by the caller.
    if (!params_.estimate_geo_reference)
    {
        return state_.geo_reference;  // the fixed one, if any
    }
    // Estimated: as of the last solve, so the gate and T_enu_to_map always come
    // from the same one, and match the keyframe poses of that solve.
    return state_.lastSolvedGeoref;
}

std::optional<mola::Georeferencing> StateEstimationSmoother::georef_for_enu_poses_locked() const
{
    // With a fixed geo-reference, {map} poses mean nothing until something
    // observes the {map} position (it is the first odometry frame until then):
    if (!params_.estimate_geo_reference && !state_.gtsam->mapPositionObservedSolved)
    {
        return {};
    }
    return get_geo_reference_locked();
}

std::optional<mola::Georeferencing> StateEstimationSmoother::compute_geo_reference_locked() const
{
    // Estimated: the datum is the first GNSS fix; T_enu_to_map is meaningful
    // only once its azimuth is observed (not held by the gauge anchor).
    std::optional<mrpt::topography::TGeodeticCoords> datum;
    if (state_.geo_reference.has_value())
    {
        datum = state_.geo_reference->geo_coord;
    }
    else
    {
        datum = state_.tentative_geo_coord_reference;
    }
    if (!datum || state_.gtsam->enuYawAnchor)
    {
        return {};
    }
    const auto T_enu_to_map = estimated_T_enu_to_map_locked();
    if (!T_enu_to_map)
    {
        return {};
    }
    mola::Georeferencing g;
    g.geo_coord    = *datum;
    g.T_enu_to_map = *T_enu_to_map;
    return g;
}

bool StateEstimationSmoother::save_geo_reference_to_file(const std::string& yamlFile) const
{
    std::optional<mola::Georeferencing> g;
    {
        auto lck = mrpt::lockHelper(stateMutex_);
        g        = get_geo_reference_locked();
    }
    if (!g)
    {
        return false;
    }

    // Same schema as mp2p_icp::ToYAML(), so the mp2p_icp/mola tools read it:
    mrpt::containers::yaml d = mrpt::containers::yaml::Map();
    d["type"]                = "mp2p_icp::Georeferencing";
    d["defined"]             = true;

    mrpt::containers::yaml geo = mrpt::containers::yaml::Map();
    geo["lon"]                 = g->geo_coord.lon.decimal_value;
    geo["lat"]                 = g->geo_coord.lat.decimal_value;
    geo["altitude"]            = g->geo_coord.height;
    d["geo_coord"]             = geo;

    const auto&            p    = g->T_enu_to_map.mean;
    mrpt::containers::yaml mean = mrpt::containers::yaml::Map();
    mean["x"]                   = p.x();
    mean["y"]                   = p.y();
    mean["z"]                   = p.z();
    mean["yaw_deg"]             = mrpt::RAD2DEG(p.yaw());
    mean["pitch_deg"]           = mrpt::RAD2DEG(p.pitch());
    mean["roll_deg"]            = mrpt::RAD2DEG(p.roll());

    mrpt::containers::yaml pose = mrpt::containers::yaml::Map();
    pose["mean"]                = mean;
    pose["cov"]                 = mrpt::containers::yaml::FromMatrix(g->T_enu_to_map.cov);
    d["T_enu_to_map"]           = pose;

    std::ofstream f(yamlFile);
    if (!f.is_open())
    {
        return false;
    }
    f << d;
    f.close();
    return !f.fail();
}

std::optional<mrpt::poses::CPose3DPDFGaussian>
    StateEstimationSmoother::estimated_T_enu_to_map_locked() const
{
    // Called with stateMutex_ already held by the caller.
    auto it = state_.last_estimated_frames.find(REFERENCE_FRAME_ID);
    if (it == state_.last_estimated_frames.end())
    {
        return {};
    }
    return {it->second};
}

std::optional<mrpt::poses::CPose3DPDFGaussian>
    StateEstimationSmoother::get_estimated_T_map_to_odometry_frame(const frame_index_t idx) const
{
    ASSERT_GE_(idx, 1);
    // Called with stateMutex_ already held by the caller.
    auto it = state_.last_estimated_frames.find(idx);
    if (it == state_.last_estimated_frames.end())
    {
        return {};
    }
    return {it->second};
}

std::optional<mrpt::poses::CPose3DPDFGaussian>
    StateEstimationSmoother::estimated_T_map_to_odometry_frame(const std::string& frame_id) const
{
    auto lck = mrpt::lockHelper(stateMutex_);

    const auto& str2id = state_.known_odom_frames.getDirectMap();
    if (auto it = str2id.find(frame_id); it != str2id.end())
    {
        return get_estimated_T_map_to_odometry_frame(it->second);
    }

    // frame not known or not estimated yet
    return {};
}

bool StateEstimationSmoother::has_converged_localization(
    mrpt::poses::CPose3DPDFGaussian& pose) const
{
    auto lck = mrpt::lockHelper(stateMutex_);

    // We need at least some frames to have an estimate
    if (state_.last_estimated_states.empty())
    {
        return false;
    }

    // Get the latest timestamp from the state
    auto tNowOpt = state_.get_current_extrapolated_stamp();
    if (!tNowOpt)
    {
        return false;
    }

    // Find the most recent frame
    if (state_.stamp2frame_index.empty())
    {
        return false;
    }

    // A geo-reference is required before a pose in the reference ("map")
    // frame is even meaningful -- either already estimated live
    // (estimate_geo_reference=true) or fixed from a loaded geo-referenced
    // map (relocalize mode, via set_geo_reference()).
    if (!state_.geo_reference.has_value())
    {
        return false;
    }

    const auto& latestIt       = state_.stamp2frame_index.getDirectMap().rbegin();
    const auto  latestFrameIdx = latestIt->second;

    NavState ns;
    try
    {
        ns = get_latest_state_and_covariance(latestFrameIdx);
    }
    catch (const std::exception& e)
    {
        // Factor graph still under-constrained: not converged yet, not a
        // fatal error (see estimated_navstate(), which handles the same
        // exception the same way).
        MRPT_LOG_DEBUG_FMT(
            "[has_converged_localization] State for KF %u not ready yet: %s",
            static_cast<unsigned>(latestFrameIdx), e.what());
        return false;
    }

    mrpt::math::CMatrixDouble66 poseCov = ns.pose.cov_inv.inverse_LLt();
    // An orientation pinned by a gauge anchor looks certain, but is unknown:
    inflate_pinned_orientation_cov_locked(poseCov);

    bool converged;
    if (params_.estimate_geo_reference)
    {
        // Live-georeferencing mode: state_.geo_reference is only ever
        // populated (see process_pending_gtsam_updates_locked()) once
        // T_enu_to_map's OWN position+orientation sigmas already passed
        // these same thresholds, and is sticky afterwards (a later,
        // temporarily-worse per-frame vehicle-pose sigma -- e.g. right
        // after a kinematics-only extrapolation past the last GNSS fix --
        // must not un-converge an already-established geo-reference).
        converged = true;
    }
    else
    {
        // Relocalize mode: state_.geo_reference is set up-front, before any
        // GNSS/IMU fusion has actually happened (see set_geo_reference()),
        // so its mere presence says nothing about whether the vehicle is
        // actually localized yet. Use the vehicle's OWN latest pose
        // uncertainty instead (this was the previous bug here: this
        // function used to return `params_.estimate_geo_reference &&
        // state_.geo_reference.has_value()`, unconditionally false in
        // relocalize mode since estimate_geo_reference is false there by
        // design -- so relocalization could never be reported as converged
        // no matter how good the GNSS+IMU fix actually was).
        const double pos_sigma_x   = std::sqrt(poseCov(0, 0));
        const double pos_sigma_y   = std::sqrt(poseCov(1, 1));
        const double pos_sigma_z   = std::sqrt(poseCov(2, 2));
        const double max_pos_sigma = std::max({pos_sigma_x, pos_sigma_y, pos_sigma_z});

        const double ori_sigma_yaw   = std::sqrt(poseCov(3, 3));
        const double ori_sigma_pitch = std::sqrt(poseCov(4, 4));
        const double ori_sigma_roll  = std::sqrt(poseCov(5, 5));
        const double max_ori_sigma_deg =
            mrpt::RAD2DEG(std::max({ori_sigma_yaw, ori_sigma_pitch, ori_sigma_roll}));

        // Until some source observes {map} (e.g. a GNSS fix through the
        // fixed geo-reference), it is defined as the first odometry frame by
        // a gauge anchor, which makes the vehicle pose look certain at the
        // odometry origin: that is not a localization. The observation must
        // already be in the solved estimate, not just queued.
        converged = state_.gtsam->mapObservedSolved &&
                    max_pos_sigma <= params_.convergence_max_position_sigma &&
                    max_ori_sigma_deg <= params_.convergence_max_orientation_sigma_deg;

        MRPT_LOG_DEBUG_FMT(
            "[has_converged_localization] converged=%s map_observed=%s pos_sigmas=(%.3f,%.3f,%.3f) "
            "m ori_sigmas=(%.2f,%.2f,%.2f) deg thresholds=(%.3f m, %.2f deg)",
            converged ? "YES" : "NO", state_.gtsam->mapObservedSolved ? "YES" : "NO", pos_sigma_x,
            pos_sigma_y, pos_sigma_z, mrpt::RAD2DEG(ori_sigma_yaw), mrpt::RAD2DEG(ori_sigma_pitch),
            mrpt::RAD2DEG(ori_sigma_roll), params_.convergence_max_position_sigma,
            params_.convergence_max_orientation_sigma_deg);
    }

    if (converged)
    {
        pose.copyFrom(ns.pose);
        if (state_.gtsam->rollPitchPinned() || state_.gtsam->yawPinned())
        {
            pose.cov = poseCov;
        }
    }

    return converged;
}

void StateEstimationSmoother::inflate_pinned_orientation_cov_locked(
    mrpt::math::CMatrixDouble66& cov) const
{
    // MRPT order: x, y, z, yaw, pitch, roll
    const auto unknown = [&cov](int i, double sigma)
    {
        for (int j = 0; j < 6; j++)
        {
            cov(i, j) = 0;
            cov(j, i) = 0;
        }
        cov(i, i) = sigma * sigma;
    };
    if (state_.gtsam->yawPinned())
    {
        unknown(3, M_PI);
    }
    if (state_.gtsam->rollPitchPinned())
    {
        unknown(4, M_PI_2);
        unknown(5, M_PI_2);
    }
}

std::optional<mola::Georeferencing> StateEstimationSmoother::current_georeferencing() const
{
    auto lck = mrpt::lockHelper(stateMutex_);
    return state_.geo_reference;
}

void StateEstimationSmoother::publishEstimatedGeoreferencing()
{
    // Must be called with lock held. Only a converged estimate is published:
    // process_pending_gtsam_updates_locked() stores it in state_.geo_reference
    // once the T_enu_to_map sigmas pass the convergence thresholds and its
    // azimuth is no longer pinned by its gauge anchor.
    if (!state_.geo_reference.has_value())
    {
        return;
    }

    // Publish via MapSourceBase
    MapUpdate mu;
    mu.method          = "state_estimator";
    mu.reference_frame = params_.reference_frame_name;
    mu.timestamp       = mrpt::Clock::now();
    mu.map_name        = "georef";
    mu.georeferencing  = state_.geo_reference;

    advertiseUpdatedMap(mu);

    state_.estimated_georef_published = true;

    MRPT_LOG_THROTTLE_INFO_STREAM(
        5.0, "Published estimated geo-reference: "
                 << "lat=" << state_.geo_reference->geo_coord.lat.getAsString()
                 << ", lon=" << state_.geo_reference->geo_coord.lon.getAsString()
                 << ", T_enu_to_map=" << state_.geo_reference->T_enu_to_map.mean.asString());
}

// ---------------------------------------------------------------------------
// IMU bias tracking
// ---------------------------------------------------------------------------
#if defined(MOLA_SMOOTHER_HAS_IMU_BIAS_SOURCE)

std::vector<std::string> StateEstimationSmoother::imu_bias_sensor_labels() const
{
    if (!params_.imu_bias_tracking)
    {
        return {};
    }
    auto lck = mrpt::lockHelper(imuBiasMutex_);
    return imuBiasTracker_.sensor_labels();
}

mola::ImuBias StateEstimationSmoother::to_imu_bias(
    const std::string& label, const mola::imu::ImuBiasTracker::Estimate& e) const
{
    ImuBias b;
    b.sensor_label     = label;
    b.gyro             = e.bias_gyro;
    b.acc              = e.bias_acc;
    b.gyro_cov         = e.cov_gyro;
    b.acc_cov          = e.cov_acc;
    b.gyro_random_walk = imuBiasTracker_.parameters.bias_gyro_random_walk;
    b.acc_random_walk  = imuBiasTracker_.parameters.bias_acc_random_walk;
    b.timestamp        = mrpt::Clock::fromDouble(e.stamp);
    if (e.has_measurements())
    {
        b.last_update = mrpt::Clock::fromDouble(e.last_update);
    }
    return b;
}

std::optional<mola::ImuBias> StateEstimationSmoother::imu_bias_impl(
    const std::string& sensorLabel, const std::optional<mrpt::Clock::time_point>& t) const
{
    if (!params_.imu_bias_tracking)
    {
        return std::nullopt;
    }
    auto       lck = mrpt::lockHelper(imuBiasMutex_);
    const auto e   = imuBiasTracker_.estimate(sensorLabel);
    if (!e)
    {
        return std::nullopt;
    }
    if (t && mrpt::Clock::toDouble(*t) >= e->stamp)
    {
        // Propagated to exactly that time:
        auto b = to_imu_bias(
            sensorLabel, *imuBiasTracker_.estimate(sensorLabel, mrpt::Clock::toDouble(*t)));
        b.timestamp = *t;
        return b;
    }
    return to_imu_bias(sensorLabel, *e);
}

void StateEstimationSmoother::queue_imu_bias_updates_locked(const std::vector<std::string>& labels)
{
    for (const auto& l : labels)
    {
        imuBiasUpdatedLabels_.insert(l);
    }
}

void StateEstimationSmoother::advertise_imu_biases()
{
    // From spinOnce(), the module's own thread: never from the threads of the
    // modules that feed this one, which may hold their own locks.
    std::vector<ImuBias> biases;
    {
        auto lck = mrpt::lockHelper(imuBiasMutex_);
        for (const auto& label : imuBiasUpdatedLabels_)
        {
            if (const auto e = imuBiasTracker_.estimate(label); e)
            {
                biases.push_back(to_imu_bias(label, *e));
            }
        }
        imuBiasUpdatedLabels_.clear();
    }
    for (const auto& b : biases)
    {
        MRPT_LOG_THROTTLE_INFO_FMT(
            5.0, "IMU bias '%s': gyro [%.4f %.4f %.4f] deg/s, sigma %.4f deg/s",
            b.sensor_label.c_str(), mrpt::RAD2DEG(b.gyro.x), mrpt::RAD2DEG(b.gyro.y),
            mrpt::RAD2DEG(b.gyro.z), mrpt::RAD2DEG(std::sqrt(b.gyro_cov(2, 2))));
        if (any_imu_bias_subscriber())
        {
            advertise_updated_imu_bias(b);
        }
    }
}

void StateEstimationSmoother::imu_bias_add_reading(const mrpt::obs::CObservationIMU& imu)
{
    if (!params_.imu_bias_tracking ||
        !std::regex_match(imu.sensorLabel, imuBiasLabelsRe_.get_regex(params_.imu_bias_labels_re)))
    {
        return;
    }
    auto lck = mrpt::lockHelper(imuBiasMutex_);
    if (imuBiasTracker_.add(imu) > 0)
    {
        imuBiasUpdatedLabels_.insert(imu.sensorLabel);
    }
}

void StateEstimationSmoother::imu_bias_add_hint_locked(double t, bool isStatic)
{
    queue_imu_bias_updates_locked(imuBiasTracker_.add_motion_hint(t, isStatic));
    (isStatic ? imuBiasHintsStatic_ : imuBiasHintsMoving_)++;
}

void StateEstimationSmoother::imu_bias_add_pose(
    const mrpt::Clock::time_point& t, const mrpt::poses::CPose3D& pose, const std::string& source)
{
    if (!params_.imu_bias_tracking)
    {
        return;
    }
    const double tNow = mrpt::Clock::toDouble(t);
    auto         lck  = mrpt::lockHelper(imuBiasMutex_);
    auto         it   = imuBiasHintRefs_.find(source);
    if (it == imuBiasHintRefs_.end() || tNow < it->second.t)
    {
        imuBiasHintRefs_[source] = {tNow, pose};
        return;
    }
    auto&        ref = it->second;
    const double dt  = tNow - ref.t;
    if (dt < params_.imu_bias_hint_min_period)
    {
        return;
    }
    // A pose increment over [ref.t, tNow]: the vehicle moved or not over it.
    const mrpt::poses::CPose3D inc = pose - ref.pose;
    const double               v   = inc.translation().norm() / dt;
    const double w        = mrpt::poses::Lie::SO<3>::log(inc.getRotationMatrix()).norm() / dt;
    const bool   isStatic = v < params_.imu_bias_hint_max_speed &&
                          w < mrpt::DEG2RAD(params_.imu_bias_hint_max_angular_speed_deg);
    // Hints hold forward for a while only: cover the whole increment, unless this
    // source was silent for long (then only its end is known).
    constexpr double MAX_INCREMENT = 5.0;  // [s]
    if (dt <= MAX_INCREMENT)
    {
        const double step = 0.5 * imuBiasTracker_.parameters.motion_hint_hold;
        for (double th = ref.t; th < tNow; th += step)
        {
            imu_bias_add_hint_locked(th, isStatic);
        }
    }
    imu_bias_add_hint_locked(tNow, isStatic);
    ref = {tNow, pose};
}

void StateEstimationSmoother::imu_bias_add_twist(
    const mrpt::Clock::time_point& t, const mrpt::math::TTwist3D& tw)
{
    if (!params_.imu_bias_tracking)
    {
        return;
    }
    const double v = std::sqrt(mrpt::square(tw.vx) + mrpt::square(tw.vy) + mrpt::square(tw.vz));
    const double w = std::sqrt(mrpt::square(tw.wx) + mrpt::square(tw.wy) + mrpt::square(tw.wz));
    const bool   isStatic = v < params_.imu_bias_hint_max_speed &&
                          w < mrpt::DEG2RAD(params_.imu_bias_hint_max_angular_speed_deg);
    auto lck = mrpt::lockHelper(imuBiasMutex_);
    imu_bias_add_hint_locked(mrpt::Clock::toDouble(t), isStatic);
}

void StateEstimationSmoother::imu_bias_correct(mrpt::obs::CObservationIMU& imu) const
{
    using namespace mrpt::obs;
    if (!params_.imu_bias_tracking || !params_.imu_bias_correct_angular_velocity ||
        !imu.has(IMU_WX) || !imu.has(IMU_WY) || !imu.has(IMU_WZ))
    {
        return;
    }
    std::optional<mola::imu::ImuBiasTracker::Estimate> e;
    {
        auto lck = mrpt::lockHelper(imuBiasMutex_);
        e        = imuBiasTracker_.estimate(imu.sensorLabel);
    }
    if (!e || !e->has_measurements())
    {
        return;
    }
    imu.set(IMU_WX, imu.get(IMU_WX) - e->bias_gyro.x);
    imu.set(IMU_WY, imu.get(IMU_WY) - e->bias_gyro.y);
    imu.set(IMU_WZ, imu.get(IMU_WZ) - e->bias_gyro.z);
}

void StateEstimationSmoother::publish_imu_bias_diagnostics()
{
    std::vector<ImuBias> biases;
    {
        auto lck = mrpt::lockHelper(imuBiasMutex_);
        for (const auto& label : imuBiasTracker_.sensor_labels())
        {
            biases.push_back(to_imu_bias(label, *imuBiasTracker_.estimate(label)));
        }
    }
    const auto now = mrpt::Clock::now();
    module_publish_diagnostics({now, "imu_bias_hints_static", double(imuBiasHintsStatic_.load())});
    module_publish_diagnostics({now, "imu_bias_hints_moving", double(imuBiasHintsMoving_.load())});
    for (const auto& b : biases)
    {
        std::string key = b.sensor_label;
        for (auto& c : key)
        {
            if (!std::isalnum(static_cast<unsigned char>(c)))
            {
                c = '_';
            }
        }
        key = "imu_bias_" + key + "_gyro_";
        module_publish_diagnostics({now, key + "x_deg_s", mrpt::RAD2DEG(b.gyro.x)});
        module_publish_diagnostics({now, key + "y_deg_s", mrpt::RAD2DEG(b.gyro.y)});
        module_publish_diagnostics({now, key + "z_deg_s", mrpt::RAD2DEG(b.gyro.z)});
        const double var = std::max({b.gyro_cov(0, 0), b.gyro_cov(1, 1), b.gyro_cov(2, 2)});
        module_publish_diagnostics({now, key + "sigma_deg_s", mrpt::RAD2DEG(std::sqrt(var))});
    }
}

#else  // no ImuBiasSource: tracking disabled

void StateEstimationSmoother::imu_bias_add_reading(const mrpt::obs::CObservationIMU&) {}
void StateEstimationSmoother::imu_bias_add_pose(
    const mrpt::Clock::time_point&, const mrpt::poses::CPose3D&, const std::string&)
{
}
void StateEstimationSmoother::imu_bias_add_twist(
    const mrpt::Clock::time_point&, const mrpt::math::TTwist3D&)
{
}
void StateEstimationSmoother::imu_bias_correct(mrpt::obs::CObservationIMU&) const {}
void StateEstimationSmoother::publish_imu_bias_diagnostics() {}
void StateEstimationSmoother::advertise_imu_biases() {}

#endif

std::optional<mrpt::math::TPoint3D> StateEstimationSmoother::predicted_antenna_in_map_locked(
    const mrpt::Clock::time_point& t, const mrpt::math::TPoint3D& lever, double maxDt)
{
    const auto                          nearby = find_before_after(t, true);
    std::optional<double>               bestDt;
    std::optional<mrpt::math::TPoint3D> ret;
    for (const auto& it : {nearby.first, nearby.second})
    {
        if (it == state_.stamp2frame_index.getDirectMap().end())
        {
            continue;
        }
        const auto st = state_.last_estimated_states.find(it->second);
        if (st == state_.last_estimated_states.end())
        {
            continue;
        }
        const double dt = mrpt::system::timeDifference(it->first, t);
        if (std::abs(dt) > maxDt || (bestDt && std::abs(dt) >= std::abs(*bestDt)))
        {
            continue;
        }
        bestDt         = dt;
        const auto& fs = st->second;
        const auto  v  = fs.twist;
        const auto  at = fs.pose + mrpt::poses::CPose3D::FromXYZYawPitchRoll(
                                       v.vx * dt, v.vy * dt, v.vz * dt, v.wz * dt, 0, 0);
        ret = at.composePoint(lever);
    }
    return ret;
}

StateEstimationSmoother::GnssGateStats StateEstimationSmoother::gnss_gate_stats() const
{
    auto lck = mrpt::lockHelper(stateMutex_);
    return gnssGateStats_;
}

bool StateEstimationSmoother::gnss_gate_locked(
    const mrpt::obs::CObservationGPS& gps, const mrpt::topography::TGeodeticCoords& geo,
    const std::array<double, 3>& enuVarFloored)
{
    auto& gg = state_.gnssGate;
    auto& st = gnssGateStats_;

    const size_t loudAfter = params_.gnss_gate_reacquire_after > 0
                                 ? static_cast<size_t>(params_.gnss_gate_reacquire_after)
                                 : 10U;

    const auto reject = [&](bool admission, const std::string& why)
    {
        st.rejected++;
        if (admission)
        {
            st.rejected_admission++;
        }
        else
        {
            st.rejected_innovation++;
        }
        st.consecutive_rejected++;
        MRPT_LOG_THROTTLE_WARN_FMT(
            2.0, "[fuse_gnss] GNSS fix rejected (%zu in total, %zu in a row): %s", st.rejected,
            st.consecutive_rejected, why.c_str());
        if (st.consecutive_rejected >= loudAfter)
        {
            MRPT_LOG_THROTTLE_ERROR_FMT(
                5.0,
                "[fuse_gnss] %zu consecutive GNSS fixes rejected: a broken receiver, a bogus "
                "source on the same input, or a drifted estimate.",
                st.consecutive_rejected);
        }
        return false;
    };

    const auto accept = [&]()
    {
        st.accepted++;
        st.consecutive_rejected = 0;
        gg.rejectedRunLength    = 0;
        gg.lastRejected.reset();
        gg.runStartFixEnu.reset();
        gg.runStartPredEnu.reset();
        return true;
    };

    // --- Admission tests ---
    const auto& gga = gps.getMsgByClass<mrpt::obs::gnss::Message_NMEA_GGA>();
    if (!params_.gnss_accepted_fix_qualities.empty())
    {
        const auto ok = parse_fix_qualities(params_.gnss_accepted_fix_qualities);
        if (ok.count(static_cast<int>(gga.fields.fix_quality)) == 0)
        {
            return reject(
                true, mrpt::format(
                          "fix quality %d not in '%s'", static_cast<int>(gga.fields.fix_quality),
                          params_.gnss_accepted_fix_qualities.c_str()));
        }
    }

    const auto& rawCov = *gps.covariance_enu;
    for (int i = 0; i < 3; i++)
    {
        if (!std::isfinite(rawCov(i, i)) || rawCov(i, i) <= 0)
        {
            return reject(true, mrpt::format("invalid ENU variance %g", rawCov(i, i)));
        }
    }
    if (!std::isfinite(geo.lat.decimal_value) || !std::isfinite(geo.lon.decimal_value) ||
        !std::isfinite(geo.height))
    {
        return reject(true, "non-finite coordinates");
    }

    const double rawSigmaXY = std::sqrt(std::max(rawCov(0, 0), rawCov(1, 1)));
    if (params_.gnss_max_horizontal_sigma > 0 && rawSigmaXY > params_.gnss_max_horizontal_sigma)
    {
        return reject(
            true,
            mrpt::format(
                "horizontal sigma %.3f m > %.3f m", rawSigmaXY, params_.gnss_max_horizontal_sigma));
    }

    std::optional<mrpt::topography::TGeodeticCoords> datum;
    if (state_.geo_reference.has_value())
    {
        datum = state_.geo_reference->geo_coord;
    }
    else if (state_.tentative_geo_coord_reference.has_value())
    {
        datum = state_.tentative_geo_coord_reference;
    }
    if (params_.gnss_max_height_deviation > 0 && datum &&
        std::abs(geo.height - datum->height) > params_.gnss_max_height_deviation)
    {
        return reject(
            true, mrpt::format(
                      "height %.2f m is %.2f m away from the datum height (max %.2f m)", geo.height,
                      geo.height - datum->height, params_.gnss_max_height_deviation));
    }

    // --- Consistency between fixes, used while there is no prediction ---
    State::GnssGate::Fix fix;
    mrpt::topography::geodeticToGeocentric_WGS84(geo, fix.ecef);
    fix.sigma = std::sqrt(*std::max_element(enuVarFloored.begin(), enuVarFloored.end()));
    fix.t     = gps.timestamp;
    fix.geo   = geo;

    const double kSigma     = params_.gnss_gate_chi2 > 0 ? std::sqrt(params_.gnss_gate_chi2) : 5.0;
    const auto   consistent = [&](const State::GnssGate::Fix& a, const State::GnssGate::Fix& b)
    {
        const double dist = (a.ecef - b.ecef).norm();
        const double dt   = std::abs(mrpt::system::timeDifference(a.t, b.t));
        return dist <= kSigma * std::sqrt(mrpt::square(a.sigma) + mrpt::square(b.sigma)) +
                           params_.gnss_max_plausible_speed * dt;
    };

    // --- Datum: only after a few mutually consistent fixes ---
    if (params_.estimate_geo_reference && !datum)
    {
        auto& cand = gg.datumCandidates;
        if (!cand.empty() && !consistent(cand.back(), fix))
        {
            st.datum_restarts++;
            MRPT_LOG_THROTTLE_WARN_FMT(
                2.0,
                "[fuse_gnss] GNSS fix %.2f m away from the previous one: restarting the datum "
                "candidate (%zu restarts so far)",
                (cand.back().ecef - fix.ecef).norm(), st.datum_restarts);
            cand.clear();
        }
        cand.push_back(fix);
        if (cand.size() < static_cast<size_t>(params_.gnss_datum_min_consistent_fixes))
        {
            return false;  // waiting for confirmation, not a rejection
        }
        state_.tentative_geo_coord_reference = cand.front().geo;
        MRPT_LOG_DEBUG_FMT("[fuse_gnss] Datum defined after %zu consistent fixes", cand.size());
        cand.clear();
        gg.lastAccepted = fix;
        return accept();
    }

    if (params_.gnss_gate_chi2 <= 0)
    {
        gg.lastAccepted = fix;
        return accept();
    }

    // --- Innovation gate, against the predicted antenna position ---
    std::optional<mrpt::poses::CPose3D> T_enu_to_map;
    if (!params_.estimate_geo_reference)
    {
        if (state_.geo_reference.has_value() && state_.gnssGate.relocalized)
        {
            T_enu_to_map = state_.geo_reference->T_enu_to_map.mean;
        }
    }
    else if (state_.geo_reference.has_value() && state_.lastSolvedGeoref.has_value())
    {
        // Only once converged: before that, T_enu_to_map is not good enough.
        T_enu_to_map = state_.lastSolvedGeoref->T_enu_to_map.mean;
    }
    std::optional<mrpt::math::TPoint3D> predMap;
    if (T_enu_to_map)
    {
        predMap = predicted_antenna_in_map_locked(
            gps.timestamp, gps.sensorPose.translation(),
            params_.gnss_nearby_keyframe_stamp_tolerance);
    }

    const auto runOrReacquire = [&](bool sameRun, bool movedEnough, const std::string& why)
    {
        gg.rejectedRunLength = sameRun ? gg.rejectedRunLength + 1 : 1;
        gg.lastRejected      = fix;
        if (params_.gnss_gate_reacquire_after > 0 && movedEnough &&
            gg.rejectedRunLength >= static_cast<size_t>(params_.gnss_gate_reacquire_after))
        {
            st.reacquired++;
            MRPT_LOG_WARN_FMT(
                "[fuse_gnss] Accepting GNSS again after %zu consistent rejected fixes (%s): "
                "the estimate is deemed to have drifted.",
                gg.rejectedRunLength, why.c_str());
            return true;
        }
        return reject(false, why);
    };

    if (!predMap)
    {
        // No prediction (not converged, or {map} not observed yet): fixes must
        // at least be consistent with the previous accepted one.
        if (!gg.lastAccepted || consistent(*gg.lastAccepted, fix))
        {
            gg.lastAccepted = fix;
            return accept();
        }
        const bool sameRun = gg.lastRejected.has_value() && consistent(*gg.lastRejected, fix);
        const auto why     = mrpt::format(
                "%.2f m from the previous accepted fix, %.2f s apart",
                (gg.lastAccepted->ecef - fix.ecef).norm(),
                std::abs(mrpt::system::timeDifference(gg.lastAccepted->t, fix.t)));
        if (runOrReacquire(sameRun, true, why))
        {
            gg.lastAccepted = fix;
            return accept();
        }
        return false;
    }

    mrpt::math::TPoint3D fixEnu;
    mrpt::topography::geodeticToENU_WGS84(geo, fixEnu, *datum);
    const auto predEnu = T_enu_to_map->composePoint(*predMap);
    const auto r       = fixEnu - predEnu;

    // Predicted position uncertainty: base, plus what may have drifted since
    // the last accepted fix.
    if (gg.postSigma < 0)
    {
        gg.postSigma = params_.estimate_geo_reference ? params_.convergence_max_position_sigma : 0;
    }
    if (gg.lastPredictedAntenna)
    {
        gg.pathSinceAccepted += (*predMap - *gg.lastPredictedAntenna).norm();
    }
    gg.lastPredictedAntenna = *predMap;
    const double driftSigma =
        gg.postSigma + params_.gnss_gate_drift_per_meter * gg.pathSinceAccepted;
    const double predVar =
        mrpt::square(params_.gnss_gate_prediction_sigma) + mrpt::square(driftSigma);

    const double d2 = mrpt::square(r.x) / (enuVarFloored[0] + predVar) +
                      mrpt::square(r.y) / (enuVarFloored[1] + predVar) +
                      mrpt::square(r.z) / (enuVarFloored[2] + predVar);

    const auto onAccepted = [&]()
    {
        const double fixVar = mrpt::square(fix.sigma);
        gg.postSigma =
            driftSigma > 0 ? std::sqrt(1.0 / (1.0 / mrpt::square(driftSigma) + 1.0 / fixVar)) : 0.0;
        if (gg.reacquiring)
        {
            // Keep the gate open until the estimate is back within the nominal one:
            const double nominal =
                kSigma * std::sqrt(mrpt::square(params_.gnss_gate_prediction_sigma) + fixVar);
            if (r.norm() > nominal)
            {
                gg.postSigma = std::max(gg.postSigma, r.norm());
            }
            else
            {
                gg.reacquiring = false;
            }
        }
        gg.pathSinceAccepted = 0;
        gg.lastAccepted      = fix;
        return accept();
    };

    if (d2 <= params_.gnss_gate_chi2)
    {
        return onAccepted();
    }

    // Rejected. Fixes that move like the predicted antenna mean the estimate is
    // the one off (a heading error only rotates the offset slowly); a source at
    // a fixed wrong place does not move with the vehicle at all:
    bool sameRun = false;
    if (gg.runStartFixEnu && gg.runStartPredEnu)
    {
        const auto   fixMove  = fixEnu - *gg.runStartFixEnu;
        const auto   predMove = predEnu - *gg.runStartPredEnu;
        const double noise    = 3.0 * std::sqrt(2.0) * fix.sigma;
        sameRun               = (fixMove - predMove).norm() <= noise + 0.5 * predMove.norm();
    }
    if (!sameRun)
    {
        gg.runStartFixEnu  = fixEnu;
        gg.runStartPredEnu = predEnu;
    }
    const bool movedEnough =
        (predEnu - *gg.runStartPredEnu).norm() >= params_.gnss_gate_reacquire_min_path;
    const auto why = mrpt::format(
        "%.2f m from the prediction (E %.2f, N %.2f, U %.2f), chi2 %.1f > %.1f", r.norm(), r.x, r.y,
        r.z, d2, params_.gnss_gate_chi2);
    if (runOrReacquire(sameRun, movedEnough, why))
    {
        // Let the following fixes in, until the estimate has moved there:
        gg.reacquiring = true;
        return onAccepted();
    }
    return false;
}

}  // namespace mola::state_estimation_smoother
