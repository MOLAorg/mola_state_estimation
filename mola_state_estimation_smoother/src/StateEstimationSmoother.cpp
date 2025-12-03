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
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

// Custom factors:
#include <mola_gtsam_factors/FactorAngularVelocityIntegration.h>
#include <mola_gtsam_factors/FactorConstLocalVelocity.h>
#include <mola_gtsam_factors/FactorGnssMapEnu.h>
#include <mola_gtsam_factors/FactorTrapezoidalIntegrator.h>

// arguments: class_name, parent_class, class namespace
IMPLEMENTS_MRPT_OBJECT(
    StateEstimationSmoother, mola::ExecutableBase, mola::state_estimation_smoother)

namespace
{
constexpr double ENU2MAP_WEAK_SIGMA           = 1e4;
constexpr double INIT_ODOM_FRAME_POSE_SIGMA   = 1e3;
constexpr double FIRST_POSE_WEAK_PRIOR_SIGMA  = 1e6;
constexpr double FIRST_TWIST_WEAK_PRIOR_SIGMA = 1e3;
constexpr double PLANAR_XY_SIGMA              = 1e10;
constexpr double PLANAR_Z_SIGMA               = 1e-4;

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

    // This is initialized in initialize(), once we have the parameters
    std::optional<gtsam::IncrementalFixedLagSmoother> smoother;

    // Queue of pending updates for incremental iSAM2:
    gtsam::NonlinearFactorGraph              newFactors;
    gtsam::Values                            newValues;
    gtsam::FixedLagSmoother::KeyTimestampMap newKeyStamps;

#if 0
    // Batch version:
    gtsam::NonlinearFactorGraph     fg;
    gtsam::Values                   values;
    gtsam::LevenbergMarquardtParams lmParams = gtsam::LevenbergMarquardtParams::CeresDefaults();
#endif
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
    reset();

    // Load params:
    params.loadFrom(cfg["params"]);

    // Forward parameters to GTSAM smoother & iSAM2:
    gtsam::ISAM2Params isam2Params;
    isam2Params.findUnusedFactorSlots = true;  // Important, must be set for fixed-lag smoother
    isam2Params.relinearizeThreshold  = 0.1;
    isam2Params.relinearizeSkip       = 1;

    state_.gtsam->smoother.emplace(params.sliding_window_length, isam2Params);

    // Initialize georeference-related gtsam variables:
    // Even if not using a geo-referenced map, even if not using GPS sensors,
    // do define the T_enu_to_map transform variable, so it can be used for gravity-alignment
    // via IMU accelerometer, at least:
    auto           enu2map     = gtsam::Pose3::Identity();
    gtsam::Matrix6 enu2map_cov = gtsam::Matrix6::Identity() * mrpt::square(ENU2MAP_WEAK_SIGMA);

    if (params.fixed_geo_reference.has_value())
    {
        state_.geo_reference = *params.fixed_geo_reference;

        mrpt::gtsam_wrappers::to_gtsam_se3_cov6(
            state_.geo_reference->T_enu_to_map, enu2map, enu2map_cov);
    }

    // Initial value:
    state_.gtsam->newValues.insert(symbol_T_enu_to_map, enu2map);
    // Weak prior factor:
    state_.gtsam->newFactors.addPrior(symbol_T_enu_to_map, enu2map, enu2map_cov);
}

void StateEstimationSmoother::spinOnce()
{
    // At the predefined module rate, publish the current estimation, if we have any subscriber:
    if (!anyUpdateLocalizationSubscriber())
    {
        return;
    }

    auto lck = mrpt::lockHelper(stateMutex_);

    const auto tNowOpt = state_.get_current_extrapolated_stamp();
    if (!tNowOpt)
    {
        MRPT_LOG_THROTTLE_WARN(5.0, "Cannot publish vehicle pose (no input data yet?)");
        return;
    }

    const auto nv = estimated_navstate(*tNowOpt, params.reference_frame_name);
    if (!nv)
    {
        MRPT_LOG_THROTTLE_WARN(5.0, "Cannot publish vehicle pose (stalled input data?)");
        return;
    }

    LocalizationUpdate lu;
    lu.child_frame     = params.vehicle_frame_name;
    lu.reference_frame = params.reference_frame_name;

    lu.method    = "state_estimator";
    lu.quality   = 1;
    lu.timestamp = *tNowOpt;
    lu.pose      = nv->pose.getPoseMean().asTPose();
    lu.cov       = nv->pose.cov_inv.inverse();

    MRPT_LOG_DEBUG_FMT(
        "[spinOnce] Publishing timely pose estimate: t=%f pose=%s", mrpt::Clock::toDouble(*tNowOpt),
        lu.pose.asString().c_str());

    advertiseUpdatedLocalization(lu);
}

void StateEstimationSmoother::reset()
{
    auto lck = mrpt::lockHelper(stateMutex_);

    // reset:
    state_ = State();
}

void StateEstimationSmoother::fuse_odometry(
    const mrpt::obs::CObservationOdometry& odom, const std::string& odomName)
{
    auto lck = mrpt::lockHelper(stateMutex_);

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
    }
    else
    {
        // This is the first time we have wheels odometry.
        lastOdom = odom.odometry;
    }
    // Use a probabilistic motion model:
    mrpt::obs::CActionRobotMovement2D odoAct;
    odoAct.motionModelConfiguration.modelSelection = mrpt::obs::CActionRobotMovement2D::mmGaussian;
    odoAct.motionModelConfiguration.gaussianModel.minStdPHI = 1e-3;
    odoAct.motionModelConfiguration.gaussianModel.minStdPHI = mrpt::DEG2RAD(0.1);

    const auto odometryIncrement = odom.odometry - lastOdom;

    odoAct.computeFromOdometry(odometryIncrement, odoAct.motionModelConfiguration);

    mrpt::poses::CPose3DPDFGaussian newOdomPosePdf;
    newOdomPosePdf.copyFrom(*odoAct.poseChange);
    // Ensure as minimal uncertainty in all 3D DOFs to prevent numerical issues:
    newOdomPosePdf.cov.asEigen().diagonal().array() += 1e-4;

    // Convert probabilistic pose back to global "odom" frame for data fusion the in "odom" frame:
    newOdomPosePdf.changeCoordinatesReference(mrpt::poses::CPose3D(lastOdom));

    MRPT_LOG_DEBUG_FMT(
        "[fuse_odometry]: t=%f name=%s pose=%s poseChange=%s",
        mrpt::Clock::toDouble(odom.timestamp), odomName.c_str(), odom.odometry.asString().c_str(),
        odoAct.poseChange->getMeanVal().asString().c_str());

    // Save for next iteration:
    state_.last_wheels_odometry_name = odomName;
    state_.last_wheels_odometry      = odom.odometry;

    // Fuse this new probabilistic pose observation:
    this->fuse_pose(odom.timestamp, newOdomPosePdf, odomName);
}

void StateEstimationSmoother::fuse_imu(const mrpt::obs::CObservationIMU& imu)
{
    auto lck = mrpt::lockHelper(stateMutex_);
#if 0
    state_.update_last_input_stamp(imu.timestamp);

    THROW_EXCEPTION("TODO");
    (void)imu;

#endif
    MRPT_LOG_DEBUG_FMT("[fuse_imu]: t=%f", mrpt::Clock::toDouble(imu.timestamp));
}

void StateEstimationSmoother::fuse_gnss(const mrpt::obs::CObservationGPS& gps)
{
    auto lck = mrpt::lockHelper(stateMutex_);

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

    // First, are we using geo-referencing at all?
    if (params.estimate_geo_reference && !state_.geo_reference.has_value())
    {
        // We are still starting to estimating the geo-reference T_enu2map.
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
    // Use fixed reference coming from an external source:
    if (!params.estimate_geo_reference && state_.geo_reference.has_value())
    {
        refGeoCoords = state_.geo_reference->geo_coord;
    }

    // Can we use geo-referencing now:
    if (!refGeoCoords.has_value())
    {
        MRPT_LOG_DEBUG("[fuse_gnss]: Ignoring reading since there is no geo-reference data (yet?)");
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
    const auto this_kf_id = create_or_get_keyframe_by_timestamp(
        gps.timestamp, params.gnss_nearby_keyframe_stamp_tolerance);

    MRPT_LOG_DEBUG_FMT(
        "[fuse_gnss]: t=%f this_kf_id=%zu ENU=%s", mrpt::Clock::toDouble(gps.timestamp),
        static_cast<size_t>(this_kf_id), ENU_point.asString().c_str());

    // Add geo-ref factor:
    const auto sensorOnVehicle = mrpt::gtsam_wrappers::toPoint3(gps.sensorPose.translation());
    const auto observedEnu     = mrpt::gtsam_wrappers::toPoint3(ENU_point);
    const auto enuCov = gtsam::noiseModel::Gaussian::Covariance(gps.covariance_enu->asEigen());

    state_.gtsam->newFactors.emplace_shared<mola::FactorGnssMapEnu>(
        symbol_T_enu_to_map, T(this_kf_id), sensorOnVehicle, observedEnu, enuCov);
}

void StateEstimationSmoother::fuse_pose(
    const mrpt::Clock::time_point& timestamp, const mrpt::poses::CPose3DPDFGaussian& pose,
    const std::string& frame_id)
{
    auto lck = mrpt::lockHelper(stateMutex_);

    // get this numerical frame_id :
    const auto frame_id_idx = add_or_get_odom_frame_id(frame_id);

    // Create a new KF id (or reuse a very close match):
    const auto this_kf_id = create_or_get_keyframe_by_timestamp(timestamp);

    MRPT_LOG_DEBUG_FMT(
        "[fuse_pose]: kf_idx=%zu t=%f frame='%s' (idx=%zu) p=%s sigmas=%.02e %.02e %.02e (m) %.02e "
        "%.02e %.02e (deg)",
        static_cast<std::size_t>(this_kf_id), mrpt::Clock::toDouble(timestamp), frame_id.c_str(),
        static_cast<std::size_t>(frame_id_idx), pose.mean.asString().c_str(),
        std::sqrt(pose.cov(0, 0)), std::sqrt(pose.cov(1, 1)), std::sqrt(pose.cov(2, 2)),
        mrpt::RAD2DEG(std::sqrt(pose.cov(3, 3))), mrpt::RAD2DEG(std::sqrt(pose.cov(4, 4))),
        mrpt::RAD2DEG(std::sqrt(pose.cov(5, 5))));

    // numerical sanity:
    for (int i = 0; i < 6; i++)
    {
        ASSERT_GT_(pose.cov(i, i), .0);
    }

    // Add factor:
    gtsam::Pose3   pose_out;
    gtsam::Matrix6 cov_out;
    mrpt::gtsam_wrappers::to_gtsam_se3_cov6(pose, pose_out, cov_out);

    // TODO: robust factors here?

    if (frame_id_idx == REFERENCE_FRAME_ID)
    {
        // reference frame:
        state_.gtsam->newFactors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
            T(this_kf_id), pose_out, gtsam::noiseModel::Gaussian::Covariance(cov_out));
    }
    else
    {
        // odometry frame:
        state_.gtsam->newFactors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            symbol_T_map_to_odom_i_base + frame_id_idx, T(this_kf_id), pose_out,
            gtsam::noiseModel::Gaussian::Covariance(cov_out));
    }

#if 0
    // Estimate twist:
    // If we add an additional direct observation of twist, the result is
    // more accurate for coarser time steps:
    if (!lastKF)
    {
        return;
    }

    const double dt = mrpt::system::timeDifference(lastKF->first, timestamp);

    if (dt > params.max_time_to_use_velocity_model)
    {
        return;
    }

    ASSERT_GT_(dt, .0);

    mrpt::math::TTwist3D tw;

    const auto incrPosePdf = pose - lastKF->second.pose->pose;
    const auto incrPose    = incrPosePdf.mean;

    tw.vx = incrPose.x() / dt;
    tw.vy = incrPose.y() / dt;
    tw.vz = incrPose.z() / dt;

    const auto logRot = mrpt::poses::Lie::SO<3>::log(incrPose.getRotationMatrix());

    tw.wx = logRot[0] / dt;
    tw.wy = logRot[1] / dt;
    tw.wz = logRot[2] / dt;

    // Ensure minimum covariance for avoiding overconfidence and numerical
    // illness:
    auto twCov = mrpt::math::CMatrixDouble66::Zero();
    for (int i = 0; i < 3; i++)
    {
        twCov(i, i) += mrpt::square(params.sigma_twist_from_consecutive_poses_linear);
        twCov(3 + i, 3 + i) += mrpt::square(params.sigma_twist_from_consecutive_poses_angular);
    }

    this->fuse_twist(timestamp, tw, twCov);
#endif
}

void StateEstimationSmoother::fuse_twist(
    const mrpt::Clock::time_point& timestamp, const mrpt::math::TTwist3D& twist,
    const mrpt::math::CMatrixDouble66& twistCov)
{
    auto lck = mrpt::lockHelper(stateMutex_);
#if 0
    state_.update_last_input_stamp(timestamp);

    TwistData d;
    d.twist    = twist;
    d.twistCov = twistCov;

    state_.data.insert({timestamp, d});


    MRPT_LOG_DEBUG_FMT(
        "[fuse_twist]: t=%f twist=%s sigmas=%.02e %.02e %.02e (m) %.02e %.02e "
        "%.02e (deg)",
        mrpt::Clock::toDouble(timestamp), twist.asString().c_str(), std::sqrt(twistCov(0, 0)),
        std::sqrt(twistCov(1, 1)), std::sqrt(twistCov(2, 2)),
        mrpt::RAD2DEG(std::sqrt(twistCov(3, 3))), mrpt::RAD2DEG(std::sqrt(twistCov(4, 4))),
        mrpt::RAD2DEG(std::sqrt(twistCov(5, 5))));
#endif
}

std::optional<NavState> StateEstimationSmoother::estimated_navstate(
    const mrpt::Clock::time_point& timestamp, const std::string& frame_id)
{
    // 1) Make sure we processed all pending sensor data, and have updated the cached values from
    //    GTSAM values
    process_pending_gtsam_updates();

    // 2) Get the vehicle state from cached optimized values:
    // Look for the closest frame and extrapolate.
    auto lck = mrpt::lockHelper(stateMutex_);

    std::optional<double>        closestFrameDt;
    std::optional<frame_index_t> closesFrameIdx;

    const auto closestPrior = find_before_after(timestamp, true);
    for (const auto& it : {closestPrior.first, closestPrior.second})
    {
        if (it == state_.stamp2frame_index.getDirectMap().end())
        {
            continue;
        }
        const auto& [existing_t, frame_idx] = *it;

        const double dt = std::abs(mrpt::system::timeDifference(existing_t, timestamp));
        if (!closestFrameDt.has_value() || dt < *closestFrameDt)
        {
            closestFrameDt = dt;
            closesFrameIdx = frame_idx;
        }
    }

    if (!closesFrameIdx.has_value())
    {
        MRPT_LOG_DEBUG_FMT(
            "[estimated_navstate] Could not find any nearby frame near requested t=%.03f",
            mrpt::Clock::toDouble(timestamp));
        return {};
    }

    // Recover
    const NavState ret = get_latest_state_and_covariance(*closesFrameIdx);

    MRPT_TODO("Implement probabilistic extrapolation");

    // Check maximum extrapolation time:
    // state_.last_estimated_states;

    // 3) Convert pose to the requested frame_id:
    MRPT_TODO("Implement frame transform");

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

#if MOLA_VERSION_CHECK(2, 1, 0)
void StateEstimationSmoother::onNewObservation(const CObservation::ConstPtr& o)
#else
void StateEstimationSmoother::onNewObservation(const CObservation::Ptr& o)
#endif
{
    const ProfilerEntry tle(profiler_, "onNewObservation");

    ASSERT_(o);

    // IMU:
    if (auto obsIMU = std::dynamic_pointer_cast<const mrpt::obs::CObservationIMU>(o);
        obsIMU && std::regex_match(o->sensorLabel, params.do_process_imu_labels_re))
    {
        this->fuse_imu(*obsIMU);
    }
    // Odometry source:
    else if (auto obsOdom = std::dynamic_pointer_cast<const mrpt::obs::CObservationOdometry>(o);
             obsOdom && std::regex_match(o->sensorLabel, params.do_process_odometry_labels_re))
    {
        this->fuse_odometry(*obsOdom, o->sensorLabel);
    }
    // Robot pose wrt "map":
    else if (auto obsPose = std::dynamic_pointer_cast<const mrpt::obs::CObservationRobotPose>(o);
             obsPose)
    {
        auto sensedSensorPose = obsPose->pose;
        if (obsPose->sensorPose != mrpt::poses::CPose3D())
        {
            sensedSensorPose =
                sensedSensorPose + mrpt::poses::CPose3DPDFGaussian(-obsPose->sensorPose);
        }

        this->fuse_pose(obsPose->timestamp, sensedSensorPose, params.reference_frame_name);
    }
    // GNSS source:
    else if (auto obsGPS = std::dynamic_pointer_cast<const mrpt::obs::CObservationGPS>(o);
             obsGPS && std::regex_match(o->sensorLabel, params.do_process_odometry_labels_re))
    {
        this->fuse_gnss(*obsGPS);
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

#if 0
std::optional<NavState> StateEstimationSmoother::build_and_optimize_fg(
    const mrpt::Clock::time_point queryTimestamp, const std::string& frame_id)
{
    using namespace std::string_literals;

    mrpt::system::CTimeLoggerEntry tle(profiler_, "build_and_optimize_fg");

    auto lck = mrpt::lockHelper(stateMutex_);

    // Return an empty answer if we don't have data, or we would need to
    // extrapolate too much:
    if (state_.data.empty() || state_.known_frames.empty())
    {
        return {};
    }
    {
        const double tq_2_tfirst =
            mrpt::system::timeDifference(queryTimestamp, state_.data.begin()->first);
        const double tlast_2_tq =
            mrpt::system::timeDifference(state_.data.rbegin()->first, queryTimestamp);
        if (tq_2_tfirst > params.max_time_to_use_velocity_model ||
            tlast_2_tq > params.max_time_to_use_velocity_model)
        {
            MRPT_LOG_DEBUG_STREAM(
                "[build_and_optimize_fg] Skipping due to need to extrapolate "
                "too much: tq_2_tfirst="
                << tq_2_tfirst << " tlast_2_tq=" << tlast_2_tq
                << " max_time_to_use_velocity_model=" << params.max_time_to_use_velocity_model);

            return {};
        }
    }

    // shortcuts:
    auto& fg     = state_.gtsam->fg;
    auto& values = state_.gtsam->values;

    values.clear();
    fg.resize(0);

    // Build the sequence of time points:
    // FG variable indices will use the indices in this vector:
    auto& dQuery = state_.data[queryTimestamp];
    dQuery.query = QueryPointData();

    using map_it_t = std::map<mrpt::Clock::time_point, PointData>::value_type;

    std::vector<map_it_t*> entries;
    std::optional<size_t>  query_KF_id;
    for (auto& it : state_.data)
    {
        if (it.first == queryTimestamp)
        {
            query_KF_id = entries.size();
        }

        entries.push_back(&it);
    }
    ASSERT_(query_KF_id.has_value());

    // add const vel kinematic factors between consecutive KFs:
    ASSERT_(entries.size() >= 1);

    for (size_t i = 1; i < entries.size(); i++)
    {
        mola::FactorConstVelKinematics f;
        f.from_kf   = i - 1;
        f.to_kf     = i;
        f.deltaTime = mrpt::system::timeDifference(entries[i - 1]->first, entries[i]->first);

        addFactor(f);
    }

    // Init values:
    for (size_t i = 0; i < entries.size(); i++)
    {
        const auto& e = entries[i]->second;

        const auto lastPose = mrpt::gtsam_wrappers::toPose3(e.last_known_state.pose);
        state_.gtsam->values.insert<gtsam::Point3>(P(i), lastPose.translation());
        state_.gtsam->values.insert<gtsam::Rot3>(R(i), lastPose.rotation());

        const auto& tw = e.last_known_state.twist;
        state_.gtsam->values.insert<gtsam::Point3>(V(i), gtsam::Vector3(tw.vx, tw.vy, tw.vz));
        state_.gtsam->values.insert<gtsam::Point3>(W(i), gtsam::Vector3(tw.wx, tw.wy, tw.wz));
    }
    for (const auto& [frameName, frameId] : state_.known_frames.getDirectMap())
    {
        // F(0): this variable is not used.
        // We only need to estimate F(i), the SE(3) pose of the frame_id "i" wrt
        // "0" (see paper diagrams!)
        if (frameId == 0)
        {
            continue;
        }

        // TODO: Save and reuse last optimized value!
        state_.gtsam->values.insert<gtsam::Pose3>(F(frameId), gtsam::Pose3::Identity());
    }


    // Unary prior for initial twist:
    const auto& tw = params.initial_twist;
    fg.addPrior(
        V(0), gtsam::Vector3(tw.vx, tw.vy, tw.vz),
        gtsam::noiseModel::Isotropic::Sigma(3, params.initial_twist_sigma_lin));

    fg.addPrior(
        W(0), gtsam::Vector3(tw.wx, tw.wy, tw.wz),
        gtsam::noiseModel::Isotropic::Sigma(3, params.initial_twist_sigma_ang));

    // Process pose observations:
    // ------------------------------------------
    for (size_t kfId = 0; kfId < entries.size(); kfId++)
    {
        // const auto  tim = entries.at(kfId)->first;
        const auto& d = entries.at(kfId)->second;

        // ---------------------------------
        // Data point of type: Pose
        // ---------------------------------
        if (d.pose.has_value())
        {
            if (d.pose->frameId == 0)
            {
                // Pose observations in the first frame are just priors:
                // (see paper!)

                gtsam::Pose3   p;
                gtsam::Matrix6 pCov;
                mrpt::gtsam_wrappers::to_gtsam_se3_cov6(d.pose->pose, p, pCov);

                {
                    auto noisePos = gtsam::noiseModel::Gaussian::Covariance(pCov.block<3, 3>(3, 3));

                    gtsam::noiseModel::Base::shared_ptr robNoisePos;
                    if (params.robust_param > 0)
                    {
                        robNoisePos = gtsam::noiseModel::Robust::Create(
                            gtsam::noiseModel::mEstimator::GemanMcClure::Create(
                                params.robust_param),
                            noisePos);
                    }
                    else
                    {
                        robNoisePos = noisePos;
                    }

                    fg.addPrior(P(kfId), p.translation(), robNoisePos);
                }

                {
                    auto noiseRot = gtsam::noiseModel::Gaussian::Covariance(pCov.block<3, 3>(0, 0));

                    gtsam::noiseModel::Base::shared_ptr robNoiseRot;
                    if (params.robust_param > 0)
                    {
                        robNoiseRot = gtsam::noiseModel::Robust::Create(
                            gtsam::noiseModel::mEstimator::GemanMcClure::Create(
                                params.robust_param),
                            noiseRot);
                    }
                    else
                    {
                        robNoiseRot = noiseRot;
                    }

                    fg.addPrior(R(kfId), p.rotation(), robNoiseRot);
                }
            }
            else
            {
                // Pose observations in subsequent frames are more complex:
                // (see paper!)
                THROW_EXCEPTION("todo");
            }
        }

        // ---------------------------------
        // Data point of type: Twist
        // ---------------------------------
        if (d.twist.has_value())
        {
            const auto&          pd   = d.twist.value();
            const gtsam::Vector3 v    = {pd.twist.vx, pd.twist.vy, pd.twist.vz};
            const gtsam::Vector3 w    = {pd.twist.wx, pd.twist.wy, pd.twist.wz};
            gtsam::Matrix3       vCov = pd.twistCov.asEigen().block<3, 3>(0, 0);
            gtsam::Matrix3       wCov = pd.twistCov.asEigen().block<3, 3>(3, 3);

            {
                auto noiseV = gtsam::noiseModel::Gaussian::Covariance(vCov);
                gtsam::noiseModel::Base::shared_ptr robNoiseV;
                if (params.robust_param > 0)
                {
                    robNoiseV = gtsam::noiseModel::Robust::Create(
                        gtsam::noiseModel::mEstimator::GemanMcClure::Create(params.robust_param),
                        noiseV);
                }
                else
                {
                    robNoiseV = noiseV;
                }

                fg.addPrior(V(kfId), v, robNoiseV);
            }
            {
                auto noiseW = gtsam::noiseModel::Gaussian::Covariance(wCov);
                gtsam::noiseModel::Base::shared_ptr robNoiseW;
                if (params.robust_param > 0)
                {
                    robNoiseW = gtsam::noiseModel::Robust::Create(
                        gtsam::noiseModel::mEstimator::GemanMcClure::Create(params.robust_param),
                        noiseW);
                }
                else
                {
                    robNoiseW = noiseW;
                }

                fg.addPrior(W(kfId), w, robNoiseW);
            }
        }

    }  // end for each kfId

    // FG is built: optimize it
    // -------------------------------------
    if (NAVSTATE_PRINT_FG)
    {
        fg.print();
        state_.gtsam->values.print();

#if 0
        fg.saveGraph("fg.dot");
#endif
    }

    gtsam::LevenbergMarquardtOptimizer lm(fg, state_.gtsam->values);

    const auto& optimal = lm.optimize();

    const double final_rmse = std::sqrt(fg.error(optimal) / fg.size());

    MRPT_LOG_DEBUG_STREAM(
        "[build_and_optimize_fg] LM ran for "
        << lm.iterations() << " iterations, " << fg.size() << " factors, " << state_.data.size()
        << " KFs, RMSE: " << std::sqrt(fg.error(state_.gtsam->values) / fg.size()) << " => "
        << final_rmse);

    if (NAVSTATE_PRINT_FG)
    {
        optimal.print("Optimized:");
        std::cout << "\n query_KF_id: " << *query_KF_id << std::endl;

        MRPT_LOG_INFO_STREAM("state_.data: ");
        for (const auto& [k, v] : state_.data)
        {
            MRPT_LOG_INFO_FMT("t=%f: %s", mrpt::Clock::toDouble(k), v.asString().c_str());
        }
    }

    if (NAVSTATE_PRINT_FG_ERRORS)
    {
        fg.printErrors(optimal, "Errors for optimized values:");
    }

    // final sanity check:
    if (final_rmse > params.max_rmse)
    {
        MRPT_LOG_WARN_STREAM(
            "[build_and_optimize_fg] Discarding solution due to high "
            "RMSE="
            << final_rmse);

        return {};
    }

    // Extract results from the factor graph:
    // ----------------------------------------------
    NavState out;

    // SE(3) pose:
    auto poseResult = gtsam::Pose3(
        optimal.at<gtsam::Rot3>(R(*query_KF_id)), optimal.at<gtsam::Point3>(P(*query_KF_id)));
    const auto outPose = mrpt::gtsam_wrappers::toTPose3D(poseResult);
    out.pose.mean      = mrpt::poses::CPose3D(outPose);

    gtsam::Marginals marginals(fg, optimal);

    // Pose SE(3) cov: (in mrpt order is xyz, then yaw/pitch/roll):
    gtsam::Matrix6 cov_inv    = gtsam::Matrix6::Zero();
    cov_inv.block<3, 3>(0, 0) = marginals.marginalInformation(P(*query_KF_id));
    cov_inv.block<3, 3>(3, 3) = marginals.marginalInformation(R(*query_KF_id));
    out.pose.cov_inv          = cov_inv;

    // Twist (already in body frame in the factor graph):
    const auto linVel = optimal.at<gtsam::Vector3>(V(*query_KF_id));
    const auto angVel = optimal.at<gtsam::Vector3>(W(*query_KF_id));

    out.twist.vx = linVel.x();
    out.twist.vy = linVel.y();
    out.twist.vz = linVel.z();
    out.twist.wx = angVel.x();
    out.twist.wy = angVel.y();
    out.twist.wz = angVel.z();

    // Twist cov:
    gtsam::Matrix6 tw_cov_inv    = gtsam::Matrix6::Zero();
    tw_cov_inv.block<3, 3>(0, 0) = marginals.marginalInformation(V(*query_KF_id));
    tw_cov_inv.block<3, 3>(3, 3) = marginals.marginalInformation(W(*query_KF_id));
    out.twist_inv_cov            = tw_cov_inv;

    // delete temporary entry:
    dQuery.query.reset();
    if (dQuery.empty()) state_.data.erase(queryTimestamp);

    // Save optimized values into data entries to bootstrap optimization
    // in next iterations:
    // -------------------------------------------------------------------
    for (size_t i = 0; i < entries.size(); i++)
    {
        auto& e = entries[i]->second;

        const auto pose =
            gtsam::Pose3(optimal.at<gtsam::Rot3>(R(i)), optimal.at<gtsam::Point3>(P(i)));

        auto& ks = e.last_known_state;

        ks.pose = mrpt::poses::CPose3D::FromRotationAndTranslation(
            pose.rotation().matrix(), pose.translation());

        const auto linV = optimal.at<gtsam::Vector3>(V(i));
        const auto angV = optimal.at<gtsam::Vector3>(W(i));
        ks.twist.vx     = linV.x();
        ks.twist.vy     = linV.y();
        ks.twist.vz     = linV.z();
        ks.twist.wx     = angV.x();
        ks.twist.wy     = angV.y();
        ks.twist.wz     = angV.z();

        if (params.enforce_planar_motion)
        {
            enforce_planar_pose(ks.pose);
            enforce_planar_twist(ks.twist);
        }
    }

    // Honor requested frame_id:
    // ----------------------------------
    ASSERTMSG_(
        state_.known_frames.hasKey(frame_id),
        "Requested results in unknown frame_id: '"s + frame_id + "'"s);

    // if this is the first frame_id, we are already done, otherwise,
    // recover and apply the transformation:
    if (const frameid_t frameId = state_.known_frames.direct(frame_id); frameId != 0)
    {
        // Apply F(frameId) transformation on the left:
        const auto T = optimal.at<gtsam::Pose3>(F(frameId));

        THROW_EXCEPTION("TODO");
    }
    NavState out;

    return out;
}
#endif

/// Implementation of Eqs (1),(4) in the MOLA RSS2019 paper.
void StateEstimationSmoother::addFactor(const mola::FactorConstVelKinematics& f)
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
    const double std_lin_vel = params.sigma_random_walk_acceleration_linear;
    const double std_ang_vel = params.sigma_random_walk_acceleration_angular;

    if (dt > params.time_between_frames_to_warning)
    {
        MRPT_LOG_WARN_FMT("Constant-velocity kinematics factor added for large dT=%.03f s.", dt);
    }

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
    state_.gtsam->newFactors.emplace_shared<mola::factors::FactorConstLocalVelocityPose>(
        kTi, kbVi, kTj, kbVj, gtsam::noiseModel::Isotropic::Sigma(3, std_lin_vel * dt));

    // \omega is in the body frame, we need a special factor to rotate it:
    // See line 4 of eq (4) in the MOLA RSS2019 paper.
    state_.gtsam->newFactors.emplace_shared<mola::factors::FactorConstLocalVelocityPose>(
        kTi, kbWi, kTj, kbWj, gtsam::noiseModel::Isotropic::Sigma(3, std_ang_vel * dt));

    // 2) Add kinematics / numerical integration factor
    // ---------------------------------------------------
    auto noise_kinematicsPosition =
        gtsam::noiseModel::Isotropic::Sigma(3, params.sigma_integrator_position);

    auto noise_kinematicsOrientation =
        gtsam::noiseModel::Isotropic::Sigma(3, params.sigma_integrator_orientation);

    // Impl. line 2 of eq (1) in the MOLA RSS2019 paper
    state_.gtsam->newFactors.emplace_shared<mola::factors::FactorTrapezoidalIntegratorPose>(
        kTi, kbVi, kTj, kbVj, dt, noise_kinematicsPosition);

    // Impl. line 1 of eq (4) in the MOLA RSS2019 paper.
    state_.gtsam->newFactors.emplace_shared<mola::factors::FactorAngularVelocityIntegrationPose>(
        kTi, kbWi, kTj, dt, noise_kinematicsOrientation);
}

void StateEstimationSmoother::addFactor(const mola::FactorTricycleKinematics& f)
{
    THROW_EXCEPTION("Write me!");
    (void)f;
}

void StateEstimationSmoother::delete_too_old_entries()
{
    // auto lck = mrpt::lockHelper(stateMutex_); // this is assumed to be acquired by caller

    // Remove really old entries in our bimap. GTSAM fixed lag handles removing actual factors.
    const double newestTime =
        mrpt::Clock::toDouble(state_.stamp2frame_index.getDirectMap().rbegin()->first);
    const double minTime = newestTime - params.sliding_window_length;

    std::set<mrpt::Clock::time_point> stamps_to_erase;
    std::set<frame_index_t>           ids_to_erase;
    for (const auto& [existing_t, frame_idx] : state_.stamp2frame_index)
    {
        const double t_existing = mrpt::Clock::toDouble(existing_t);
        if (t_existing < minTime)
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
}

// Creates a new frame index for timestamp t, or returns the existing one if close enough.
// This also is in charge of the complex task of finding nearby existing frames and adding the
// kinematic factors to ensure smooth motion estimation.
StateEstimationSmoother::frame_index_t StateEstimationSmoother::create_or_get_keyframe_by_timestamp(
    const mrpt::Clock::time_point& t, const std::optional<double>& overrideCloseEnough)
{
    const auto tle = mola::ProfilerEntry(profiler_, "create_or_get_keyframe_by_timestamp");

    auto lck = mrpt::lockHelper(stateMutex_);

    const double threshold =
        overrideCloseEnough ? *overrideCloseEnough : params.min_time_difference_to_create_new_frame;

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

    // As we create a new timely keyframe, update what's the last time we created such new frame:
    state_.last_observation_stamp           = t;
    state_.last_observation_wallclock_stamp = mrpt::Clock::now();

    // Look for the closest existing frames, and create kinematic pairs if they don't exist yet:
    const auto closestPost = find_before_after(t, false);

    // Create new GTSAM symbols for this keyframe:
    initialize_new_frame(newFrameIdx, closestPost);

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
    if (frame_id_name == params.reference_frame_name)
    {
        return REFERENCE_FRAME_ID;
    }

    ASSERT_NOT_EQUAL_(frame_id_name, params.vehicle_frame_name);
    ASSERT_NOT_EQUAL_(frame_id_name, params.enu_frame_name);

    // auto lck = mrpt::lockHelper(stateMutex_); // acquired by caller

    // Existing frame?
    if (auto it = state_.known_odom_frames.find_key(frame_id_name);
        it != state_.known_odom_frames.getDirectMap().end())
    {
        return it->second;
    }

    // New one: starting at "1" (0=reserved for "map")
    const auto newId = static_cast<odometry_frameid_t>(state_.known_odom_frames.size()) + 1;
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
    const auto tle = mola::ProfilerEntry(profiler_, "process_pending_gtsam_updates");

    auto lck = mrpt::lockHelper(stateMutex_);

    // Even if we have no new factors/values, do update the stamps of "persistent" variables:
    if (state_.last_observation_stamp.has_value())
    {
        const auto lastObservationStamp_sec = mrpt::Clock::toDouble(*state_.last_observation_stamp);

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

    // Update the smoother with pending factors/values:
    if (!state_.gtsam->newFactors.empty() || !state_.gtsam->newValues.empty() ||
        !state_.gtsam->newKeyStamps.empty())
    {
        smoother.update(
            state_.gtsam->newFactors, state_.gtsam->newValues, state_.gtsam->newKeyStamps);
    }

    // Optional: Perform extra internal iterations for better accuracy
    for (unsigned int i = 1; i < params.additional_isam2_update_steps; ++i)
    {
        smoother.update();
    }

    // Print debug info:
    MRPT_LOG_DEBUG_STREAM(
        "[process_pending_gtsam_updates] After update: "
        << smoother.getFactors().size() << " factors, " << smoother.getFactors().nrFactors()
        << " nr factors. New factors=" << state_.gtsam->newFactors.size()
        << ", new values=" << state_.gtsam->newValues.size());

    const auto optValues = smoother.calculateEstimate();

    // Retrieve the latest estimate and save it into "state_.last_estimated_state":
    for (auto& [kfIdx, kf] : state_.last_estimated_states)
    {
        const auto pose = optValues.at<gtsam::Pose3>(T(kfIdx));
        const auto linV = optValues.at<gtsam::Vector3>(V(kfIdx));
        const auto angV = optValues.at<gtsam::Vector3>(W(kfIdx));

        kf.pose  = mrpt::poses::CPose3D(mrpt::gtsam_wrappers::toTPose3D(pose));
        kf.twist = {linV.x(), linV.y(), linV.z(), angV.x(), angV.y(), angV.z()};

        if (params.enforce_planar_motion)
        {
            enforce_planar_pose(kf.pose);
            enforce_planar_twist(kf.twist);
        }
    }

    // Retrieve latest enu_to_map for geo-referencing:
    if (params.estimate_geo_reference)
    {
        const auto T_enu_to_map     = optValues.at<gtsam::Pose3>(symbol_T_enu_to_map);
        const auto T_enu_to_map_cov = smoother.marginalCovariance(symbol_T_enu_to_map);

        std::cout << "T_enu_to_map: " << mrpt::gtsam_wrappers::toTPose3D(T_enu_to_map)
                  << "\ncov: " << T_enu_to_map_cov.diagonal().array().sqrt().eval() << "\n\n";
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
    const auto stamp_s = mrpt::Clock::toDouble(stamp);

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
    }
    else
    {
        // This is the first ever frame.
        // Add weak prior factors for the system to be determinate.
        const auto priorNoise6 =
            gtsam::noiseModel::Isotropic::Sigma(6, FIRST_POSE_WEAK_PRIOR_SIGMA);
        const auto priorNoise3 =
            gtsam::noiseModel::Isotropic::Sigma(3, FIRST_TWIST_WEAK_PRIOR_SIGMA);

        state_.gtsam->newFactors.addPrior(T(id), pose, priorNoise6);
        state_.gtsam->newFactors.addPrior(V(id), linVelocity, priorNoise3);
        state_.gtsam->newFactors.addPrior(W(id), angVelocity, priorNoise3);
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
    if (params.enforce_planar_motion)
    {
        const auto planar_z_noise = gtsam::noiseModel::Diagonal::Sigmas(
            gtsam::Vector6(
                PLANAR_XY_SIGMA, PLANAR_XY_SIGMA, PLANAR_XY_SIGMA, PLANAR_XY_SIGMA, PLANAR_XY_SIGMA,
                PLANAR_Z_SIGMA));

        state_.gtsam->newFactors.addPrior(T(id), gtsam::Pose3::Identity(), planar_z_noise);
    }
}

void StateEstimationSmoother::add_kinematic_factor_between(
    const frame_index_t from, const frame_index_t to)  // NOLINT
{
    ASSERT_NOT_EQUAL_(from, to);

    // Take note of already connected frames to avoid duplications
    // --------------------------------------------------------------------
    // From => to
    {
        auto& fromKf = state_.last_estimated_states.at(from);
        if (fromKf.kinematic_links_to.count(to) != 0)
        {
            return;  // already added
        }
        fromKf.kinematic_links_to.insert(to);
    }

    // To => From
    {
        auto& toKf = state_.last_estimated_states.at(to);
        if (toKf.kinematic_links_to.count(from) != 0)
        {
            return;  // already added
        }
        toKf.kinematic_links_to.insert(from);
    }

    // Dispatch to factor generation:
    // --------------------------------------------------------------------
    const double dt = mrpt::Clock::toDouble(state_.stamp2frame_index.inverse(to)) -
                      mrpt::Clock::toDouble(state_.stamp2frame_index.inverse(from));

    switch (params.kinematic_model)
    {
        case KinematicModel::ConstantVelocity:
        {
            mola::FactorConstVelKinematics f;
            f.from_kf   = from;
            f.to_kf     = to;
            f.deltaTime = dt;
            addFactor(f);
        }
        break;

        case KinematicModel::Tricycle:
        {
            mola::FactorTricycleKinematics f;
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

NavState StateEstimationSmoother::get_latest_state_and_covariance(const frame_index_t idx) const
{
    const auto& frame = state_.last_estimated_states.at(idx);

    NavState ns;

    // Pose:
    ns.pose.mean       = frame.pose;
    const auto poseCov = gtsam::Matrix6(state_.gtsam->smoother->marginalCovariance(T(idx)));
    ASSERT_(poseCov.determinant() > 0);
    ns.pose.cov_inv = mrpt::gtsam_wrappers::to_mrpt_se3_cov6(poseCov).inverse_LLt();

    // Twist:
    ns.twist        = frame.twist;
    const auto vCov = gtsam::Matrix3(state_.gtsam->smoother->marginalCovariance(V(idx)));
    const auto wCov = gtsam::Matrix3(state_.gtsam->smoother->marginalCovariance(W(idx)));

    gtsam::Matrix6 twCov    = gtsam::Matrix6::Zero();
    twCov.block<3, 3>(0, 0) = vCov;
    twCov.block<3, 3>(3, 3) = wCov;

    ASSERT_(twCov.determinant() > 0);
    ns.twist_inv_cov = twCov;

    return ns;
}

}  // namespace mola::state_estimation_smoother
