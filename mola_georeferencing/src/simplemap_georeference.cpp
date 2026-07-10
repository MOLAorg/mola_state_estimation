/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
 Closed-source licenses available upon request, for this package
 alone or in combination with the complete SLAM system.
*/

// MOLA+MRPT
#include <mola_georeferencing/simplemap_georeference.h>
#include <mrpt/core/get_env.h>
#include <mrpt/poses/gtsam_wrappers.h>
#include <mrpt/topography/conversions.h>

#if __has_include(<mp2p_icp/update_velocity_buffer_from_obs.h>)
#include <mola_imu_preintegration/LocalVelocityBuffer.h>
#include <mp2p_icp/update_velocity_buffer_from_obs.h>
#define HAS_VELOCITY_BUFFER
#endif

// gtsam factors:
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <mola_gtsam_factors/FactorGnssEnu.h>
#include <mola_gtsam_factors/MeasuredGravityFactor.h>
#include <mola_gtsam_factors/Pose3RotationFactor.h>
#include <mola_gtsam_factors/imu_helpers.h>

#include <algorithm>
#include <limits>

mola::SMGeoReferencingOutput mola::simplemap_georeference(
    const mrpt::maps::CSimpleMap& sm, const SMGeoReferencingParams& params)
{
    mola::SMGeoReferencingOutput ret;

    ASSERT_(!sm.empty());

    const GNSSFrames smFrames =
        extract_gnss_frames_from_sm(sm, params.geodeticReference, params.minimumGNSSFixQuality);

    if (params.logger)
    {
        std::stringstream ss;
        ss << "[simplemap_georeference] Found: " << smFrames.frames.size() << " GNSS frames";
        params.logger->logStr(mrpt::system::LVL_INFO, ss.str());
    }

    // we check GNSS frames later on, to check if we have at least IMU data.

    // Build and optimize GTSAM graph:
    using gtsam::symbol_shorthand::P;  // P(i): each vehicle pose
    using gtsam::symbol_shorthand::T;  // T(0): the single sought transformation

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values               v;

    add_gnss_factors(graph, v, smFrames, params.fgParams);

    // Collect which P(kf_index) keys are already in the graph from GNSS:
    std::set<size_t> existingPoseKeys;
    for (const auto& f : smFrames.frames)
    {
        existingPoseKeys.insert(f.kf_index);
    }

    bool hasIMUGravityFactors  = false;
    bool hasIMUAttitudeFactors = false;
    if (params.useIMUGravityAlignment || params.useIMUAttitudeAlignment)
    {
        const IMUFrames imuFrames = extract_imu_frames_from_sm(sm);

        const auto nGravity = std::count_if(
            imuFrames.frames.begin(), imuFrames.frames.end(),
            [](const FrameIMU& f) { return f.normalizedAcc.has_value(); });
        const auto nAttitude = std::count_if(
            imuFrames.frames.begin(), imuFrames.frames.end(),
            [](const FrameIMU& f) { return f.rawAttitude.has_value(); });

        if (params.logger)
        {
            std::stringstream ss;
            ss << "[simplemap_georeference] Found: " << nGravity
               << " IMU gravity (accelerometer) frames, " << nAttitude
               << " IMU absolute-attitude frames";
            params.logger->logStr(mrpt::system::LVL_INFO, ss.str());
        }

        if (params.useIMUGravityAlignment && nGravity > 0)
        {
            hasIMUGravityFactors = true;
            add_imu_gravity_factors(graph, v, imuFrames, existingPoseKeys, params.imuGravityParams);
        }
        if (params.useIMUAttitudeAlignment && nAttitude > 0)
        {
            hasIMUAttitudeFactors = true;
            add_imu_attitude_factors(
                graph, v, imuFrames, existingPoseKeys, params.imuAttitudeParams);
        }
    }

    if (smFrames.frames.empty() && !hasIMUGravityFactors && !hasIMUAttitudeFactors)
    {
        if (params.logger)
        {
            params.logger->logStr(
                mrpt::system::LVL_ERROR,
                "The input simplemap seems not to have neither GNSS nor IMU acceleration/attitude "
                "data, so no georeferencing/gravity alignment can be performed.");
        }
        return ret;
    }

    thread_local bool DEBUG_PRINT_GRAPH =
        mrpt::get_env<bool>("MOLA_SM_GEOREF_PRINT_FACTOR_GRAPH", false);
    if (DEBUG_PRINT_GRAPH)
    {
        graph.print("\n====\nGTSAM graph:\n");
        v.print("\n====\nGTSAM initial values:\n");
    }

    gtsam::LevenbergMarquardtParams lmParams = gtsam::LevenbergMarquardtParams::CeresDefaults();

    gtsam::LevenbergMarquardtOptimizer lm(graph, v, lmParams);

    auto optimal = lm.optimize();

    thread_local bool DEBUG_PRINT_FG_ERRORS =
        mrpt::get_env<bool>("MOLA_SM_GEOREF_PRINT_FG_ERRORS", false);
    if (DEBUG_PRINT_FG_ERRORS)
    {
        graph.printErrors(optimal, "\n===\nFG errors:\n");
    }

    const double errInit = graph.error(v);
    const double errEnd  = graph.error(optimal);

    const double rmseInit = std::sqrt(errInit / static_cast<double>(graph.size()));
    const double rmseEnd  = std::sqrt(errEnd / static_cast<double>(graph.size()));

    gtsam::Marginals marginals(graph, optimal);

    const auto T0     = optimal.at<gtsam::Pose3>(T(0));
    const auto T0_cov = marginals.marginalCovariance(T(0));
    const auto stds   = T0_cov.diagonal().array().sqrt().eval();

    if (params.logger)
    {
        std::stringstream ss;
        ss << "[simplemap_georeference] LM iterations: " << lm.iterations()
           << ", init error: " << errInit << " (rmse=" << rmseInit << "), final error: " << errEnd
           << "(rmse=" << rmseEnd << ") , for " << smFrames.frames.size()
           << " frames, GTSAM sigmas: " << mrpt::RAD2DEG(stds[0]) << " [deg], "
           << mrpt::RAD2DEG(stds[1]) << " [deg], " << mrpt::RAD2DEG(stds[2]) << " [deg], "
           << stds[3] << " [m], " << stds[4] << " [m], " << stds[5] << " [m]";
        params.logger->logStr(mrpt::system::LVL_INFO, ss.str());
    }

    // store results:
    ret.geo_ref.emplace();

    // We will always have this T, using IMU or GNSS:
    ret.geo_ref->T_enu_to_map = {
        mrpt::poses::CPose3D(mrpt::gtsam_wrappers::toTPose3D(T0)),
        mrpt::gtsam_wrappers::to_mrpt_se3_cov6(T0_cov)};

    // We may not have geodetics reference if using IMU only. Leave lat=lon=h=0
    if (smFrames.refCoord.has_value())
    {
        ret.geo_ref->geo_coord = *smFrames.refCoord;
        ret.has_geodetic_datum = true;
    }

    ret.final_rmse = rmseEnd;

    return ret;
}

mp2p_icp::metric_map_t::Georeferencing mola::recenter_georeference(
    const mp2p_icp::metric_map_t::Georeferencing& in,
    const mrpt::math::TPoint3D&                   desiredEnuToMapTranslation)
{
    using mrpt::poses::CPose3D;
    using mrpt::poses::CPose3DPDFGaussian;

    mp2p_icp::metric_map_t::Georeferencing out = in;

    const mrpt::math::TPoint3D t1 = desiredEnuToMapTranslation;

    // We work in geocentric (ECEF) coordinates so the result is EXACT on the
    // curved Earth: the local ENU axes rotate as the datum moves, so keeping the
    // same T_enu_to_map rotation while shifting the datum would only be a
    // flat-Earth approximation. Instead we preserve the physical map<->ECEF
    // placement and recompute T_enu_to_map for the new datum's ENU frame. Note
    // that this means the rotation of T_enu_to_map is, in general, NOT
    // preserved: only the map<->geodetic mapping is left exact.

    // ENU frame of the current datum, expressed in ECEF (rotation = ENU->ECEF,
    // translation = datum ECEF position). This is deterministic (no uncertainty).
    mrpt::math::TPose3D e0;
    mrpt::topography::ENU_axes_from_WGS84(in.geo_coord, e0, /*only_angles=*/false);
    const CPose3DPDFGaussian E0{CPose3D(e0)};

    // Physical placement of the map in ECEF (invariant): map -> ECEF.
    //   map -> ENU is T_enu_to_map^{-1}; ENU -> ECEF is E0.
    // Using the Gaussian PDF composition operators propagates the input
    // covariance through this transformation (instead of just its mean).
    const CPose3DPDFGaussian A = E0 + (-in.T_enu_to_map);

    // The new ENU datum is placed at the map point `t1` (T_enu_to_map maps the
    // ENU origin to its own translation). Its ECEF position (mean only, `t1` is
    // a fixed user input with no uncertainty):
    const mrpt::math::TPoint3D X1 = A.mean.composePoint(t1);

    // New datum in geodetic coordinates (ECEF -> geodetic):
    mrpt::topography::TGeodeticCoords newDatum;
    mrpt::topography::geocentricToGeodetic(
        X1, newDatum, mrpt::topography::TEllipsoid::Ellipsoid_WGS84());

    // ENU frame of the new datum in ECEF (deterministic):
    mrpt::math::TPose3D e1;
    mrpt::topography::ENU_axes_from_WGS84(newDatum, e1, /*only_angles=*/false);
    const CPose3DPDFGaussian E1{CPose3D(e1)};

    // New T_enu_to_map preserving the exact map<->ECEF placement:
    //   map -> ECEF = E1 o T1^{-1} = A   =>   T1 = A^{-1} o E1.
    // By construction its mean translation equals `t1`; its covariance is the
    // propagation of the input T_enu_to_map covariance through this chain.
    const CPose3DPDFGaussian T1 = (-A) + E1;

    out.geo_coord    = newDatum;
    out.T_enu_to_map = T1;

    return out;
}

mola::GNSSFrames mola::extract_gnss_frames_from_sm(
    const mrpt::maps::CSimpleMap&                           sm,
    const std::optional<mrpt::topography::TGeodeticCoords>& refCoordIn,
    unsigned int                                            minimumFixQuality)
{
    thread_local bool DEBUG_PRINT_GNSS_FRAMES =
        mrpt::get_env<bool>("MOLA_SM_GEOREF_PRINT_GNSS_FRAMES", false);

    GNSSFrames ret;

    ret.refCoord = refCoordIn;

    ret.frames.reserve(sm.size());

    // Build list of KF poses with GNSS observations:
    for (size_t kfIdx = 0; kfIdx < sm.size(); kfIdx++)
    {
        const auto& [pose, sf, twist] = sm.get(kfIdx);

        ASSERT_(pose);
        ASSERT_(sf);

        const auto p = pose->getMeanVal();

        mrpt::obs::CObservationGPS::Ptr obs;
        for (size_t i = 0; !!(obs = sf->getObservationByClass<mrpt::obs::CObservationGPS>(i)); i++)
        {
            if (!obs->hasMsgType(mrpt::obs::gnss::NMEA_GGA))
            {
                continue;
            }

            // Optional fix-quality filter (disabled when minimumFixQuality==0):
            if (minimumFixQuality > 0)
            {
                const auto& ggaMsg = obs->getMsgByClass<mrpt::obs::gnss::Message_NMEA_GGA>();
                if (ggaMsg.fields.fix_quality < minimumFixQuality)
                {
                    continue;  // skip low-quality fix
                }
            }

            auto& f = ret.frames.emplace_back();

            f.pose     = p;
            f.kf_index = kfIdx;
            f.obs      = obs;
            f.gga      = obs->getMsgByClass<mrpt::obs::gnss::Message_NMEA_GGA>();

            if (obs->covariance_enu)
            {
                f.sigma_E = std::sqrt((*obs->covariance_enu)(0, 0));
                f.sigma_N = std::sqrt((*obs->covariance_enu)(1, 1));
                f.sigma_U = std::sqrt((*obs->covariance_enu)(2, 2));
            }
            else
            {
                f.sigma_E = f.gga.fields.HDOP * 4.5 /*HDOP_REFERENCE_METERS*/;
                f.sigma_N = f.sigma_E;
                f.sigma_U = f.sigma_E;
            }

            const bool invalid_sigmas =
                (f.sigma_E <= 0 || f.sigma_N <= 0 || f.sigma_U <= 0 || std::isnan(f.sigma_E) ||
                 std::isnan(f.sigma_N) || std::isnan(f.sigma_U));

            const bool invalid_geodetic =
                (f.gga.fields.latitude_degrees == 0 && f.gga.fields.longitude_degrees == 0 &&
                 f.gga.fields.altitude_meters == 0);

            if (invalid_sigmas || invalid_geodetic)
            {
                ret.frames.pop_back();  // Remove invalid entry
                continue;  // skip invalid entry
            }

            f.coords.lat    = f.gga.fields.latitude_degrees;
            f.coords.lon    = f.gga.fields.longitude_degrees;
            f.coords.height = f.gga.fields.altitude_meters;

            // keep first one:
            if (!ret.refCoord.has_value())
            {
                ret.refCoord = f.coords;
            }

            // Convert GNSS obs to ENU:
            mrpt::topography::geodeticToENU_WGS84(f.coords, f.enu, *ret.refCoord);

            if (DEBUG_PRINT_GNSS_FRAMES)
            {
                std::cout << "gnss frame for sm[" << i << "]: " << std::fixed
                          << std::setprecision(6) << " lat: " << f.coords.lat
                          << " deg, lon: " << f.coords.lon << " deg, alt: " << f.coords.height
                          << " m, ENU: " << f.enu << " m, sigmas (E/N/U): " << f.sigma_E << "/"
                          << f.sigma_N << "/" << f.sigma_U << " m"
                          << "\n";
            }
        }
    }

    // Degeneracy check: the spatial spread of the GNSS observations must be
    // large compared to their uncertainty, otherwise the global-attitude
    // (roll/pitch/yaw) problem is ill-conditioned. We compare the ENU
    // bounding-box diagonal against 3x the smallest per-axis sigma.
    if (ret.frames.size() >= 2)
    {
        mrpt::math::TPoint3D bbMin    = ret.frames.front().enu;
        mrpt::math::TPoint3D bbMax    = ret.frames.front().enu;
        double               minSigma = std::numeric_limits<double>::max();

        for (const auto& f : ret.frames)
        {
            bbMin.x = std::min(bbMin.x, f.enu.x);
            bbMin.y = std::min(bbMin.y, f.enu.y);
            bbMin.z = std::min(bbMin.z, f.enu.z);
            bbMax.x = std::max(bbMax.x, f.enu.x);
            bbMax.y = std::max(bbMax.y, f.enu.y);
            bbMax.z = std::max(bbMax.z, f.enu.z);

            minSigma = std::min({minSigma, f.sigma_E, f.sigma_N, f.sigma_U});
        }

        const double bboxDiagonal = (bbMax - bbMin).norm();

        if (bboxDiagonal <= 3.0 * minSigma)
        {
            ret.possibly_degenerate = true;

            std::cerr << "\n"
                      << "############################################################\n"
                      << "# [extract_gnss_frames_from_sm] WARNING: POSSIBLY DEGENERATE\n"
                      << "# GNSS configuration detected.\n"
                      << "#   ENU bounding-box diagonal: " << bboxDiagonal << " m\n"
                      << "#   minimum per-axis sigma:    " << minSigma << " m\n"
                      << "#   (diagonal must be > 3x the minimum sigma = " << 3.0 * minSigma
                      << " m)\n"
                      << "# The spatial spread of the " << ret.frames.size()
                      << " GNSS observations is too small with respect to\n"
                      << "# their uncertainty. The estimated global attitude (map roll/pitch/yaw)\n"
                      << "# is likely unobservable and may take absurd values.\n"
                      << "############################################################\n"
                      << std::endl;
        }
    }

    return ret;
}

void mola::add_gnss_factors(
    gtsam::NonlinearFactorGraph& fg, gtsam::Values& v, const GNSSFrames& frames,
    const AddGNSSFactorParams& params)
{
    using gtsam::symbol_shorthand::P;  // P(i): each vehicle pose, in the {map} frame
    using gtsam::symbol_shorthand::T;  // T(0): the single sought transformation: {enu} -> {map}

    if (!std::isfinite(params.robustParamHuberK) || params.robustParamHuberK <= 0)
    {
        THROW_EXCEPTION_FMT(
            "Invalid AddGNSSFactorParams::robustParamHuberK=%f: must be finite and positive.",
            params.robustParamHuberK);
    }

    v.insert(T(0), gtsam::Pose3::Identity());

    // Expression to optimize (i=0...N):
    // P (+) kf_pose{i} = gps_enu{i}

    auto noisePoses         = gtsam::noiseModel::Isotropic::Sigma(6, 1e-2);
    auto noiseHorizontality = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector6(1e3, 1e3, 1e3, 1e6, 1e6, params.horizontalitySigmaZ));

    for (size_t i = 0; i < frames.frames.size(); i++)
    {
        const auto& frame = frames.frames.at(i);

        auto noiseOrg = gtsam::noiseModel::Diagonal::Sigmas(
            gtsam::Vector3(frame.sigma_E, frame.sigma_N, frame.sigma_U)
                .array()
                .max(params.minimumUncertaintyXYZ));

        auto robustNoise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(params.robustParamHuberK), noiseOrg);

        const auto observedENU = mrpt::gtsam_wrappers::toPoint3(frame.enu);
        const auto sensorPointOnVeh =
            mrpt::gtsam_wrappers::toPoint3(frame.obs->sensorPose.translation());

        fg.emplace_shared<mola::factors::FactorGnssEnu>(
            P(frame.kf_index), sensorPointOnVeh, observedENU, robustNoise);

        const auto vehiclePose = mrpt::gtsam_wrappers::toPose3(frame.pose);

        if (!v.exists(P(frame.kf_index)))
        {
            v.insert(P(frame.kf_index), vehiclePose);
        }

        fg.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            T(0), P(frame.kf_index), vehiclePose, noisePoses);

        if (params.addHorizontalityConstraints)
        {
            fg.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
                P(frame.kf_index), gtsam::Pose3::Identity(), noiseHorizontality);
        }
    }
}

mola::IMUFrames mola::extract_imu_frames_from_sm(const mrpt::maps::CSimpleMap& sm)
{
    IMUFrames ret;
    ret.frames.reserve(sm.size());

    for (size_t kfIdx = 0; kfIdx < sm.size(); kfIdx++)
    {
        const auto& [pose, sf, twist] = sm.get(kfIdx);

        ASSERT_(pose);
        ASSERT_(sf);

        const auto p = pose->getMeanVal();

        // 1) Process direct CObservationIMU observations, if available. A single
        // observation may carry a gravity (accelerometer) reading, an absolute
        // attitude (orientation) reading, or both:
        mrpt::obs::CObservationIMU::Ptr obs;
        for (size_t i = 0; !!(obs = sf->getObservationByClass<mrpt::obs::CObservationIMU>(i)); i++)
        {
            std::optional<gtsam::Vector3> acc;
            if (obs->has(mrpt::obs::IMU_X_ACC) && obs->has(mrpt::obs::IMU_Y_ACC) &&
                obs->has(mrpt::obs::IMU_Z_ACC))
            {
                const gtsam::Vector3 measuredGravity = {
                    obs->get(mrpt::obs::IMU_X_ACC), obs->get(mrpt::obs::IMU_Y_ACC),
                    obs->get(mrpt::obs::IMU_Z_ACC)};

                if (mola::factors::imu_accel_looks_like_gravity(measuredGravity))
                {
                    acc = measuredGravity.normalized();
                }
            }

            std::optional<gtsam::Rot3> attitude;
            if (obs->has(mrpt::obs::IMU_ORI_QUAT_W))
            {
                const double qw = obs->get(mrpt::obs::IMU_ORI_QUAT_W);
                const double qx = obs->get(mrpt::obs::IMU_ORI_QUAT_X);
                const double qy = obs->get(mrpt::obs::IMU_ORI_QUAT_Y);
                const double qz = obs->get(mrpt::obs::IMU_ORI_QUAT_Z);

                if (mola::factors::imu_quaternion_looks_valid(qw, qx, qy, qz))
                {
                    attitude = gtsam::Rot3::Quaternion(qw, qx, qy, qz);
                }
            }

            if (!acc.has_value() && !attitude.has_value())
            {
                continue;  // nothing usable in this observation
            }

            auto& f               = ret.frames.emplace_back();
            f.kf_index            = kfIdx;
            f.vehiclePose         = p;
            f.sensorPoseOnVehicle = obs->sensorPose;
            f.normalizedAcc       = acc;
            f.rawAttitude         = attitude;
        }

        // 2) Process embedded IMU acceleration info embedded into the metadata.
        // (No absolute-attitude equivalent here: the buffered orientation is only
        // gravity-leveled, not azimuth-referenced, so it is not usable for this.)
#if defined(HAS_VELOCITY_BUFFER)
        mola::imu::LocalVelocityBuffer lvb;
        for (const auto& o : *sf)
        {
            mp2p_icp::update_velocity_buffer_from_obs(lvb, o);
        }

        // Get the current linear accelerations map (in the vehicle frame of reference)
        // Average them all so there are not too many factors:
        gtsam::Vector3 avr_acc   = gtsam::Vector3::Zero();
        size_t         avr_count = 0;

        for (const auto& [t, measuredGravity] : lvb.get_linear_accelerations())
        {
            const auto acc = mrpt::gtsam_wrappers::toPoint3(measuredGravity);
            if (mola::factors::imu_accel_looks_like_gravity(acc))
            {
                avr_count++;
                avr_acc += acc;
            }
        }

        if (avr_count > 0)
        {
            auto& f               = ret.frames.emplace_back();
            f.kf_index            = kfIdx;
            f.vehiclePose         = p;
            f.sensorPoseOnVehicle = mrpt::poses::CPose3D::Identity();
            f.normalizedAcc       = (avr_acc / static_cast<double>(avr_count)).normalized();
        }

#endif
    }  // end for each SM keyframe

    return ret;
}

void mola::add_imu_gravity_factors(
    gtsam::NonlinearFactorGraph& fg, gtsam::Values& v, const IMUFrames& imuFrames,
    const std::set<size_t>& existingPoseKeys, const AddIMUGravityFactorParams& params)
{
    using gtsam::symbol_shorthand::P;
    using gtsam::symbol_shorthand::T;

    auto noisePoses = gtsam::noiseModel::Isotropic::Sigma(6, 1e-2);
    auto accNoise =
        gtsam::noiseModel::Isotropic::Sigma(3, mrpt::DEG2RAD(params.imuGravitySigmaDeg));

    // If T(0) was not added because there were no GNSS frames, add it now:
    if (existingPoseKeys.empty())
    {
        if (!v.exists(T(0)))
        {
            v.insert(T(0), gtsam::Pose3::Identity());
        }

        // Also, add a weak prior to anchor the undefined azimuth angle, which is unobservable
        // without GNSS:

        //  Used when there are IMU factors but no GNSS data, to "anchor" the solution in the
        //  azimuth angle, which is unobservable with gravity-only factors [rad] */
        double azimuthUnobservableSigma = 1.0;

        auto noisePriorT0 = gtsam::noiseModel::Diagonal::Sigmas(
            gtsam::Vector6(1.0, 1.0, azimuthUnobservableSigma, 1.0, 1.0, 1.0));
        fg.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
            T(0), gtsam::Pose3::Identity(), noisePriorT0);
    }

    for (const auto& frame : imuFrames.frames)
    {
        if (!frame.normalizedAcc.has_value())
        {
            continue;
        }

        const auto key = P(frame.kf_index);

        // If this KF doesn't already have a P variable (from GNSS), create one:
        if (existingPoseKeys.count(frame.kf_index) == 0 && !v.exists(key))
        {
            const auto vehiclePose = mrpt::gtsam_wrappers::toPose3(frame.vehiclePose);
            v.insert(key, vehiclePose);

            fg.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(key, vehiclePose, noisePoses);
        }

        const auto sensorOnVehicle = mrpt::gtsam_wrappers::toPose3(frame.sensorPoseOnVehicle);

        fg.emplace_shared<mola::factors::MeasuredGravityFactor>(
            T(0), key, sensorOnVehicle, *frame.normalizedAcc, accNoise);
    }
}

void mola::add_imu_attitude_factors(
    gtsam::NonlinearFactorGraph& fg, gtsam::Values& v, const IMUFrames& imuFrames,
    const std::set<size_t>& existingPoseKeys, const AddIMUAttitudeFactorParams& params)
{
    using gtsam::symbol_shorthand::P;
    using gtsam::symbol_shorthand::T;

    auto noisePoses = gtsam::noiseModel::Isotropic::Sigma(6, 1e-2);

    // Roll, pitch, yaw sigmas (approximate correspondence to the Rot3 tangent-space
    // residual near the optimum). Yaw is independently weighted since IMU-fused yaw
    // (often magnetometer-based) is typically much noisier than roll/pitch:
    auto attitudeNoise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(
        mrpt::DEG2RAD(params.imuAttitudeSigmaDeg), mrpt::DEG2RAD(params.imuAttitudeSigmaDeg),
        mrpt::DEG2RAD(params.imuAttitudeYawSigmaDeg)));

    // If there were no GNSS frames, T(0) still exists in `v` (add_gnss_factors()
    // always inserts it unconditionally), but with no prior tying down its
    // *translation*: rotation-only factors (gravity and attitude alike) never
    // constrain it, since composing poses and then extracting the rotation
    // discards all translation components along the chain. Without GNSS, add a
    // weak prior now to keep the linear system well-posed, even though attitude
    // factors alone already observe T(0)'s full rotation (including azimuth):
    if (existingPoseKeys.empty())
    {
        if (!v.exists(T(0)))
        {
            v.insert(T(0), gtsam::Pose3::Identity());
        }

        double weakSigma = 1.0;  // [rad] or [m], both loose enough not to bias real estimates

        auto noisePriorT0 = gtsam::noiseModel::Isotropic::Sigma(6, weakSigma);
        fg.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
            T(0), gtsam::Pose3::Identity(), noisePriorT0);
    }

    for (const auto& frame : imuFrames.frames)
    {
        if (!frame.rawAttitude.has_value())
        {
            continue;
        }

        const auto key = P(frame.kf_index);

        // If this KF doesn't already have a P variable (from GNSS or gravity), create one:
        if (existingPoseKeys.count(frame.kf_index) == 0 && !v.exists(key))
        {
            const auto vehiclePose = mrpt::gtsam_wrappers::toPose3(frame.vehiclePose);
            v.insert(key, vehiclePose);

            fg.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(key, vehiclePose, noisePoses);
        }

        const auto sensorOnVehicle = mrpt::gtsam_wrappers::toPose3(frame.sensorPoseOnVehicle);

        const auto correctedAttitude = mola::factors::imu_apply_enu_azimuth_correction(
            *frame.rawAttitude, params.azimuthOffsetDeg);

        fg.emplace_shared<mola::factors::Pose3RotationFactor>(
            T(0), key, sensorOnVehicle, correctedAttitude, attitudeNoise);
    }
}

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

mola::IMUAccFrames mola::extract_imu_acc_frames_from_sm(const mrpt::maps::CSimpleMap& sm)
{
    const IMUFrames full = extract_imu_frames_from_sm(sm);

    IMUAccFrames ret;
    ret.frames.reserve(full.frames.size());

    for (const auto& f : full.frames)
    {
        if (!f.normalizedAcc.has_value())
        {
            continue;
        }

        auto& out               = ret.frames.emplace_back();
        out.kf_index            = f.kf_index;
        out.vehiclePose         = f.vehiclePose;
        out.sensorPoseOnVehicle = f.sensorPoseOnVehicle;
        out.normalizedAcc       = *f.normalizedAcc;
    }

    return ret;
}

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif
