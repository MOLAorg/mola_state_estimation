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
#include <mola_gtsam_factors/FactorAngularVelocityIntegration.h>
#include <mola_gtsam_factors/FactorConstLocalVelocity.h>
#include <mola_gtsam_factors/FactorGnssMapEnu.h>
#include <mola_gtsam_factors/FactorTrapezoidalIntegrator.h>
#include <mola_gtsam_factors/FactorTricycleKinematic.h>
#include <mola_gtsam_factors/MeasuredGravityFactor.h>
#include <mola_gtsam_factors/Pose3RotationFactor.h>
#include <mola_gtsam_factors/imu_helpers.h>

#include "FastPredictor.h"
#include "Snapshot.h"
#include "extrapolation.h"

// std:
#include <algorithm>
#include <chrono>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

// arguments: class_name, parent_class, class namespace
IMPLEMENTS_MRPT_OBJECT(
    StateEstimationSmoother, mola::ExecutableBase, mola::state_estimation_smoother)

namespace
{
constexpr double ENU2MAP_WEAK_SIGMA          = 1e4;
constexpr double INIT_ODOM_FRAME_POSE_SIGMA  = 1e3;
constexpr double FIRST_POSE_WEAK_PRIOR_SIGMA = 1e6;
constexpr double PLANAR_XY_SIGMA             = 1e10;
constexpr double PLANAR_Z_SIGMA              = 1e-4;
constexpr double TRICYCLE_LARGE_SIGMAS       = 1e6;

/// Huber threshold [whitened units] for raw gyro observations; the standard
/// value giving ~95% efficiency under Gaussian noise.
constexpr double GYRO_HUBER_K = 1.345;

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

using gtsam::symbol_shorthand::T;  // Poses                          (Pose3)
using gtsam::symbol_shorthand::V;  // Lin velocity (body frame)      (Point3)
using gtsam::symbol_shorthand::W;  // Ang velocity (body frame)      (Point3)
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
    isam2Params.relinearizeThreshold  = 0.1;
    isam2Params.relinearizeSkip       = 1;
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

        mrpt::gtsam_wrappers::to_gtsam_se3_cov6(
            state_.geo_reference->T_enu_to_map, enu2map, enu2map_cov);

        // Update into last_estimated_frames too, so estimated_T_enu_to_map() returns it:
        state_.last_estimated_frames[REFERENCE_FRAME_ID] = state_.geo_reference->T_enu_to_map;
    }

    // Initial value:
    state_.gtsam->newValues.insert(symbol_T_enu_to_map, enu2map);
    // Weak prior factor:
    state_.gtsam->newFactors.addPrior(symbol_T_enu_to_map, enu2map, enu2map_cov);
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

void StateEstimationSmoother::spinOnce()
{
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

    LocalizationUpdate lu;
    lu.child_frame     = childFrame;
    lu.reference_frame = params_.reference_frame_name;
    lu.method          = methodLabel + "/map_odom";
    lu.quality         = 1;
    lu.timestamp       = stamp;
    lu.pose            = T_map_to_odom->mean.asTPose();
    lu.cov             = T_map_to_odom->cov;

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
        uint64_t                                                              batchEpoch = 0;
        {
            std::unique_lock<std::mutex> lk(queueMutex_);
            queueCv_.wait(lk, [this] { return backendStop_.load() || !ingestQueue_.empty(); });
            if (backendStop_.load())
            {
                break;
            }
            batch.swap(ingestQueue_);
            batchEpoch = resetEpoch_.load(std::memory_order_relaxed);
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
    mrpt::obs::CActionRobotMovement2D odoAct;
    odoAct.motionModelConfiguration.modelSelection = mrpt::obs::CActionRobotMovement2D::mmGaussian;
    odoAct.motionModelConfiguration.gaussianModel.a1       = params_.odom_motion_model_a1;
    odoAct.motionModelConfiguration.gaussianModel.a2       = params_.odom_motion_model_a2;
    odoAct.motionModelConfiguration.gaussianModel.a3       = params_.odom_motion_model_a3;
    odoAct.motionModelConfiguration.gaussianModel.a4       = params_.odom_motion_model_a4;
    odoAct.motionModelConfiguration.gaussianModel.minStdXY = params_.odom_motion_model_min_std_xy;
    odoAct.motionModelConfiguration.gaussianModel.minStdPHI =
        mrpt::DEG2RAD(params_.odom_motion_model_min_std_phi_deg);

    const auto odometryIncrement = odom.odometry - lastOdom;

    odoAct.computeFromOdometry(odometryIncrement, odoAct.motionModelConfiguration);

    mrpt::poses::CPose3DPDFGaussian incrementPdf;
    incrementPdf.copyFrom(*odoAct.poseChange);
    // Ensure as minimal uncertainty in all 3D DOFs to prevent numerical issues:
    incrementPdf.cov.asEigen().diagonal().array() += 1e-4;

    // Compose this increment onto the accumulated dead-reckoning pose, so the
    // covariance handed to fuse_pose_locked() is the uncertainty of the
    // ABSOLUTE pose in {odom_i} -- which is what that factor asserts -- rather
    // than the uncertainty of this one increment.
    //
    // This matters because the resulting factor is
    // BetweenFactor(T_map_to_odom_i, T(kf), pose_in_odom, cov). T_map_to_odom_i
    // is a single rigid variable, so with a per-increment covariance every
    // keyframe in the window is told it lies within ~1 cm of one common frame,
    // no matter how far the wheels have dead-reckoned since. On a platform with
    // any slip those constraints are mutually inconsistent and the only way the
    // optimizer can satisfy them is to distort the poses.
    //
    // Growing with distance travelled is the point: the factor is informative
    // while the dead reckoning is fresh and fades as it stales, which is what it
    // actually knows. The composition is conservative for nearby keyframe pairs
    // (it ignores the correlation between two accumulations sharing a history),
    // and conservative is the safe direction here.
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
        odoAct.poseChange->getMeanVal().asString().c_str(),
        std::sqrt(newOdomPosePdf.cov(0, 0) + newOdomPosePdf.cov(1, 1)),
        mrpt::RAD2DEG(std::sqrt(newOdomPosePdf.cov(5, 5))));

    // Save for next iteration (advance the anchor: this reading was kept):
    state_.last_wheels_odometry_name  = odomName;
    state_.last_wheels_odometry       = odom.odometry;
    state_.last_wheels_odometry_stamp = odom.timestamp;

    if (!params_.odometry_relative_factors)
    {
        // Fuse this new probabilistic pose observation:
        fuse_pose_locked(odom.timestamp, newOdomPosePdf, odomName);
        return;
    }

    fuse_odometry_relative_locked(odom, odomName, incrementPdf, newOdomPosePdf);
}

// Relative formulation: the motion goes into BetweenFactors between consecutive
// odometry keyframes, and exactly ONE absolute pose-in-{odom_i} factor is ever
// added, on the first kept reading of each source, to resolve T_map_to_odom_i.
//
// One is all it takes, and it stays that way for the whole run. T_map_to_odom_i
// is a single rigid variable: given that anchor plus the relative chain, every
// later "keyframe k is at odom pose p_k" is already implied, up to the odometry
// drift in between. A second absolute factor observes the frame no better -- it
// re-injects the accumulated dead-reckoning error into the map poses, which is
// the defect the covariance accumulation mitigates rather than removes.
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
    const mrpt::poses::CPose3DPDFGaussian& increment,
    const mrpt::poses::CPose3DPDFGaussian& absolutePoseInOdom)
{
    const auto frame_id_idx = add_or_get_odom_frame_id(odomName);
    const auto this_kf_id   = create_or_get_keyframe_by_timestamp_locked(odom.timestamp);

    // Unconditional, and independent of which factor is added below: this is the
    // anchor estimated_navstate() extrapolates from when a front end asks for a
    // pose in the source's OWN frame, so it has to track every kept reading.
    state_.last_raw_pose_by_source[frame_id_idx] =
        State::RawSourcePose{odom.timestamp, absolutePoseInOdom};

    if (!state_.wheels_odometry_anchor_kf.has_value())
    {
        gtsam::Pose3   pose_out;
        gtsam::Matrix6 cov_out;
        mrpt::gtsam_wrappers::to_gtsam_se3_cov6(absolutePoseInOdom, pose_out, cov_out);

        state_.gtsam->newFactors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            symbol_T_map_to_odom_i_base + frame_id_idx, T(this_kf_id), pose_out,
            gtsam::noiseModel::Gaussian::Covariance(cov_out));

        state_.wheels_odometry_anchor_kf = this_kf_id;

        MRPT_LOG_DEBUG_FMT(
            "[fuse_odometry] odom frame '%s' anchored at KF %u", odomName.c_str(),
            static_cast<unsigned>(this_kf_id));
    }

    const auto prev                = state_.last_wheels_odometry_kf;
    state_.last_wheels_odometry_kf = this_kf_id;

    if (!prev.has_value() || *prev == this_kf_id)
    {
        return;  // first reading of the chain, or both readings landed on one keyframe
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
    // T_prev^-1 * T_now. So the motion model's increment is directly the
    // measurement this factor needs, with directly the covariance it computed.
    gtsam::Pose3   incr_out;
    gtsam::Matrix6 incrCov_out;
    mrpt::gtsam_wrappers::to_gtsam_se3_cov6(increment, incr_out, incrCov_out);

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

void StateEstimationSmoother::fuse_imu_locked(const mrpt::obs::CObservationIMU& imu)
{
    // Ignore an IMU reading with no usable content up front, before touching the
    // decimation stamp or creating a keyframe: otherwise an empty sample would
    // consume the decimation interval (skipping the next, useful one) and add a
    // factor-less keyframe.
    const bool hasAttitude = imu.has(mrpt::obs::IMU_ORI_QUAT_W);
    const bool hasGravity =
        imu.has(mrpt::obs::IMU_X_ACC) && params_.imu_normalized_gravity_alignment_sigma > 0;
    const bool hasAngularVelocity = imu.has(mrpt::obs::IMU_WX) && imu.has(mrpt::obs::IMU_WY) &&
                                    imu.has(mrpt::obs::IMU_WZ) &&
                                    params_.imu_angular_velocity_sigma > 0;
    if (!hasAttitude && !hasGravity && !hasAngularVelocity)
    {
        return;
    }

    // High-rate decimation: skip IMU readings arriving too soon after the last
    // processed one. IMU attitude/gravity are absolute observations, so dropping
    // intermediate readings just lowers the redundant-factor rate (unlike wheel
    // odometry, there is no increment to accumulate).
    if (params_.imu_min_sample_period > 0 && state_.last_processed_imu_stamp.has_value())
    {
        const double dt =
            mrpt::system::timeDifference(*state_.last_processed_imu_stamp, imu.timestamp);
        if (dt < params_.imu_min_sample_period)
        {
            return;
        }
    }
    state_.last_processed_imu_stamp = imu.timestamp;

    // Create a new KF id (or reuse a very close match):
    const auto this_kf_id = create_or_get_keyframe_by_timestamp_locked(
        imu.timestamp, params_.imu_nearby_keyframe_stamp_tolerance);

    MRPT_LOG_DEBUG_FMT(
        "[fuse_imu]: t=%f  this_kf_id=%zu ", mrpt::Clock::toDouble(imu.timestamp),
        static_cast<size_t>(this_kf_id));

    // Shared by every branch below (at least one runs, given the early return above):
    const auto sensorOnVehicle = mrpt::gtsam_wrappers::toPose3(imu.sensorPose);

    // Direct azimuth observation?
    // -------------------------------------------------
    if (imu.has(mrpt::obs::IMU_ORI_QUAT_W))
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
            const auto measuredRotation = mola::factors::imu_apply_enu_azimuth_correction(
                gtsam::Rot3::Quaternion(qw, qx, qy, qz), params_.imu_attitude_azimuth_offset_deg);

            // Create noise model for rotation (3 DOF: roll, pitch, yaw)
            auto rotationNoise = gtsam::noiseModel::Isotropic::Sigma(
                3, mrpt::DEG2RAD(params_.imu_attitude_sigma_deg));

            state_.gtsam->newFactors.emplace_shared<mola::factors::Pose3RotationFactor>(
                symbol_T_enu_to_map, T(this_kf_id), sensorOnVehicle, measuredRotation,
                rotationNoise);
        }
    }

    // Gravity-aligned acceleration observation?
    // -------------------------------------------------
    if (imu.has(mrpt::obs::IMU_X_ACC) && params_.imu_normalized_gravity_alignment_sigma > 0)
    {
        // TODO: Use ImuTransformer, etc.

        const gtsam::Vector3 measuredGravity = {
            imu.get(mrpt::obs::IMU_X_ACC), imu.get(mrpt::obs::IMU_Y_ACC),
            imu.get(mrpt::obs::IMU_Z_ACC)};

        // Some IMU drivers publishes normalized acc:
        if (mola::factors::imu_accel_looks_like_gravity(measuredGravity))
        {
            const gtsam::Vector3 measuredGravityNormalized = measuredGravity.normalized();

            // Create noise model for gravity alignment:
            auto accNoise = gtsam::noiseModel::Isotropic::Sigma(
                3, mrpt::DEG2RAD(params_.imu_normalized_gravity_alignment_sigma));

            state_.gtsam->newFactors.emplace_shared<mola::factors::MeasuredGravityFactor>(
                symbol_T_enu_to_map, T(this_kf_id), sensorOnVehicle, measuredGravityNormalized,
                accNoise);
        }
    }

    // Angular velocity (gyroscope) observation: a direct prior on this keyframe's
    // body-frame angular-velocity variable, so a genuine, fast rotation is represented
    // in the graph immediately instead of only through the constant-velocity kinematic
    // factor between keyframes (see imu_angular_velocity_sigma's docstring).
    if (hasAngularVelocity)
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

    if (!gps.covariance_enu.has_value())
    {
        MRPT_LOG_THROTTLE_WARN(
            5.0, "Discarding GNSS (GPS) reading since it does not have ENU covariance.");
        return;
    }

    mrpt::math::TPoint3D ENU_point;
    mrpt::topography::geodeticToENU_WGS84(geoCoords, ENU_point, *refGeoCoords);

    // Create a new KF id (or reuse a very close match):
    const auto this_kf_id = create_or_get_keyframe_by_timestamp_locked(
        gps.timestamp, params_.gnss_nearby_keyframe_stamp_tolerance);

    MRPT_LOG_DEBUG_FMT(
        "[fuse_gnss]: t=%f this_kf_id=%zu ENU=%s", mrpt::Clock::toDouble(gps.timestamp),
        static_cast<size_t>(this_kf_id), ENU_point.asString().c_str());

    // Add geo-ref factor:
    const auto sensorOnVehicle = mrpt::gtsam_wrappers::toPoint3(gps.sensorPose.translation());
    const auto observedEnu     = mrpt::gtsam_wrappers::toPoint3(ENU_point);
    const auto enuNoise = gtsam::noiseModel::Gaussian::Covariance(gps.covariance_enu->asEigen());

    gtsam::SharedNoiseModel enuNoiseModel;
    if (params_.gnss_huber_threshold > 0)
    {
        enuNoiseModel = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(params_.gnss_huber_threshold), enuNoise);
    }
    else
    {
        enuNoiseModel = enuNoise;
    }

    state_.gtsam->newFactors.emplace_shared<mola::factors::FactorGnssMapEnu>(
        symbol_T_enu_to_map, T(this_kf_id), sensorOnVehicle, observedEnu, enuNoiseModel);
}

void StateEstimationSmoother::fuse_pose(
    const mrpt::Clock::time_point& timestamp, const mrpt::poses::CPose3DPDFGaussian& pose,
    const std::string& frame_id)
{
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

void StateEstimationSmoother::fuse_pose_locked(
    const mrpt::Clock::time_point& timestamp, const mrpt::poses::CPose3DPDFGaussian& pose,
    const std::string& frame_id)
{
    // get this numerical frame_id :
    const auto frame_id_idx = add_or_get_odom_frame_id(frame_id);

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

    // Add factor:
    gtsam::Pose3   pose_out;
    gtsam::Matrix6 cov_out;
    mrpt::gtsam_wrappers::to_gtsam_se3_cov6(poseSanitized, pose_out, cov_out);

    // TODO: robust factors here?

    // reference frame ("map") or "odom_i"?
    if (frame_id_idx == REFERENCE_FRAME_ID)
    {
        // ref is "map":
        state_.gtsam->newFactors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
            T(this_kf_id), pose_out, gtsam::noiseModel::Gaussian::Covariance(cov_out));
    }
    else
    {
        // ref is an odometry frame:
        state_.gtsam->newFactors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            symbol_T_map_to_odom_i_base + frame_id_idx, T(this_kf_id), pose_out,
            gtsam::noiseModel::Gaussian::Covariance(cov_out));

        // Remember this source's own last raw pose (in {odom_i}), the anchor
        // estimated_navstate() extrapolates from to keep the short-term
        // prediction continuous in the front end's own frame.
        state_.last_raw_pose_by_source[frame_id_idx] =
            State::RawSourcePose{timestamp, poseSanitized};
    }
}

void StateEstimationSmoother::fuse_twist(
    const mrpt::Clock::time_point& timestamp, const mrpt::math::TTwist3D& twist,
    const mrpt::math::CMatrixDouble66& twistCov)
{
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
        auto rawPdf =
            extrapolate_pose_pdf(params_, rawAnchor.pose, ret.twist, anchorTwistCov, dtPred);
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
    const double std_lin_vel   = params_.sigma_random_walk_acceleration_linear;
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
        gtsam::noiseModel::Isotropic::Sigma(3, params_.sigma_integrator_position);

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

    std::set<mrpt::Clock::time_point> stamps_to_erase;
    std::set<frame_index_t>           ids_to_erase;
    for (const auto& [existing_t, frame_idx] : state_.stamp2frame_index)
    {
        if (mrpt::system::timeDifference(existing_t, newestStamp) > params_.sliding_window_length)
        {
            stamps_to_erase.insert(existing_t);
            ids_to_erase.insert(frame_idx);
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
        const mrpt::Clock::time_point& t, const std::optional<double>& overrideCloseEnough)
{
    const auto tle = mola::ProfilerEntry(profiler_, "create_or_get_keyframe_by_timestamp");

    const double threshold = overrideCloseEnough ? *overrideCloseEnough
                                                 : params_.min_time_difference_to_create_new_frame;

    // See if we have an existing frame index close enough to t:
    const auto closestPrior = find_before_after(t, true);
    for (const auto& it : {closestPrior.first, closestPrior.second})
    {
        if (it == state_.stamp2frame_index.getDirectMap().end())
        {
            continue;
        }
        const auto& [existing_t, frame_idx] = *it;

        const double dt = std::abs(mrpt::system::timeDifference(existing_t, t));

        if (dt < threshold)
        {
            return frame_idx;
        }
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

    // Remove really old entries in our bimap. GTSAM fixed lag handles removing actual factors.
    delete_too_old_entries();

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

    return newId;
}

void StateEstimationSmoother::process_pending_gtsam_updates()
{
    auto lck = mrpt::lockHelper(stateMutex_);
    process_pending_gtsam_updates_locked();
}

void StateEstimationSmoother::process_pending_gtsam_updates_locked()
{
    const auto tle = mola::ProfilerEntry(profiler_, "process_pending_gtsam_updates");

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
                gtsam::FactorIndices indices;
                for (size_t i = range.first; i < range.second && i < newIdx.size(); i++)
                {
                    indices.push_back(newIdx[i]);
                }
                state_.gtsam->flushedKinematic[linkKey] = indices;
            }
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
            else if (dynamic_cast<const mola::factors::MeasuredGravityFactor*>(p))
                ++nImuGravity;
            else if (dynamic_cast<const mola::factors::FactorGnssMapEnu*>(p))
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

        pdf.mean = mrpt::poses::CPose3D(mrpt::gtsam_wrappers::toTPose3D(T_enu_to_map));
        pdf.cov  = mrpt::gtsam_wrappers::to_mrpt_se3_cov6(T_enu_to_map_cov);

        // Convert info matrix to covariance
        if (!state_.stamp2frame_index.empty())
        {
            const auto& latestIt       = state_.stamp2frame_index.getDirectMap().rbegin();
            const auto  latestFrameIdx = latestIt->second;

            const auto poseCov =
                gtsam::Matrix6(state_.gtsam->smoother->marginalCovariance(T(latestFrameIdx)));
            mrpt::math::CMatrixDouble66 cov = mrpt::gtsam_wrappers::to_mrpt_se3_cov6(poseCov);

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
            const double em_max_ori_sigma_rad =
                std::max({em_ori_sigma_yaw, em_ori_sigma_pitch, em_ori_sigma_roll});
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

            if (converged && state_.tentative_geo_coord_reference.has_value())
            {
                state_.geo_reference.emplace();
                state_.geo_reference->geo_coord    = state_.tentative_geo_coord_reference.value();
                state_.geo_reference->T_enu_to_map = pdf;
                state_.estimated_georef_published  = false;  // so it's re-published
            }
        }
    }

    // retrieve odometry frames:
    {
        const auto tleOdomMarg =
            mola::ProfilerEntry(profiler_, "process_pending.marginals.odom_frames");
        for (const auto& [_, odomFrameIdx] : state_.known_odom_frames)
        {
            const auto symbolOdom        = symbol_T_map_to_odom_i_base + odomFrameIdx;
            const auto T_map2_odom_i     = optValues.at<gtsam::Pose3>(symbolOdom);
            const auto T_map2_odom_i_cov = smoother.marginalCovariance(symbolOdom);

            auto& pdf = state_.last_estimated_frames[odomFrameIdx];

            pdf.mean = mrpt::poses::CPose3D(mrpt::gtsam_wrappers::toTPose3D(T_map2_odom_i));
            pdf.cov  = mrpt::gtsam_wrappers::to_mrpt_se3_cov6(T_map2_odom_i_cov);
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

    // Still pending: it never reached the solver, so dropping it costs nothing.
    if (auto it = state_.gtsam->pendingKinematic.find(key);
        it != state_.gtsam->pendingKinematic.end())
    {
        state_.gtsam->pendingKinematic.erase(it);
        return;
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

    snap->geoReference = state_.geo_reference;

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
            const mrpt::math::CMatrixDouble66 poseCov = snap->anchor.pose.cov_inv.inverse_LLt();
            const double                      maxPos =
                std::sqrt(std::max({poseCov(0, 0), poseCov(1, 1), poseCov(2, 2)}));
            const double maxOriDeg =
                mrpt::RAD2DEG(std::sqrt(std::max({poseCov(3, 3), poseCov(4, 4), poseCov(5, 5)})));
            converged = maxPos <= params_.convergence_max_position_sigma &&
                        maxOriDeg <= params_.convergence_max_orientation_sigma_deg;
        }
    }
    snap->localizationConverged = converged;

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
    ns.pose.cov_inv = mrpt::gtsam_wrappers::to_mrpt_se3_cov6(poseCov).inverse_LLt();

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
        const mrpt::math::CMatrixDouble66 poseCov = ns.pose.cov_inv.inverse_LLt();

        const double pos_sigma_x   = std::sqrt(poseCov(0, 0));
        const double pos_sigma_y   = std::sqrt(poseCov(1, 1));
        const double pos_sigma_z   = std::sqrt(poseCov(2, 2));
        const double max_pos_sigma = std::max({pos_sigma_x, pos_sigma_y, pos_sigma_z});

        const double ori_sigma_yaw   = std::sqrt(poseCov(3, 3));
        const double ori_sigma_pitch = std::sqrt(poseCov(4, 4));
        const double ori_sigma_roll  = std::sqrt(poseCov(5, 5));
        const double max_ori_sigma_deg =
            mrpt::RAD2DEG(std::max({ori_sigma_yaw, ori_sigma_pitch, ori_sigma_roll}));

        converged = max_pos_sigma <= params_.convergence_max_position_sigma &&
                    max_ori_sigma_deg <= params_.convergence_max_orientation_sigma_deg;

        MRPT_LOG_DEBUG_FMT(
            "[has_converged_localization] converged=%s pos_sigmas=(%.3f,%.3f,%.3f) m "
            "ori_sigmas=(%.2f,%.2f,%.2f) deg thresholds=(%.3f m, %.2f deg)",
            converged ? "YES" : "NO", pos_sigma_x, pos_sigma_y, pos_sigma_z,
            mrpt::RAD2DEG(ori_sigma_yaw), mrpt::RAD2DEG(ori_sigma_pitch),
            mrpt::RAD2DEG(ori_sigma_roll), params_.convergence_max_position_sigma,
            params_.convergence_max_orientation_sigma_deg);
    }

    if (converged)
    {
        pose.copyFrom(ns.pose);
    }

    return converged;
}

std::optional<mola::Georeferencing> StateEstimationSmoother::current_georeferencing() const
{
    auto lck = mrpt::lockHelper(stateMutex_);
    return state_.geo_reference;
}

void StateEstimationSmoother::publishEstimatedGeoreferencing()
{
    // Must be called with lock held
    if (!state_.tentative_geo_coord_reference.has_value())
    {
        return;
    }

    auto T_enu_map_opt = estimated_T_enu_to_map_locked();
    if (!T_enu_map_opt.has_value())
    {
        return;
    }

    // Store as our now-fixed geo-reference
    state_.geo_reference.emplace();
    state_.geo_reference->geo_coord    = *state_.tentative_geo_coord_reference;
    state_.geo_reference->T_enu_to_map = *T_enu_map_opt;

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

}  // namespace mola::state_estimation_smoother
