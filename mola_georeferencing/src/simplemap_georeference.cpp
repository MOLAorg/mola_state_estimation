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
#include <mrpt/poses/gtsam_wrappers.h>
#include <mrpt/topography/conversions.h>

// gtsam factors:
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <mola_gtsam_factors/FactorGnssEnu.h>

mola::SMGeoReferencingOutput mola::simplemap_georeference(
    const mrpt::maps::CSimpleMap& sm, const SMGeoReferencingParams& params)
{
    mola::SMGeoReferencingOutput ret;

    ASSERT_(!sm.empty());

    const GNSSFrames smFrames = extract_gnss_frames_from_sm(sm, params.geodeticReference);

    if (params.logger)
    {
        std::stringstream ss;
        ss << "[simplemap_georeference] Found: " << smFrames.frames.size() << " GNSS frames";
        params.logger->logStr(mrpt::system::LVL_INFO, ss.str());
    }

    if (smFrames.frames.empty())
    {
        params.logger->logStr(
            mrpt::system::LVL_ERROR,
            "The input simplemap seems not to have any GNSS observations!");
        return ret;
    }

    // Build and optimize GTSAM graph:
    using gtsam::symbol_shorthand::P;  // P(i): each vehicle pose
    using gtsam::symbol_shorthand::T;  // T(0): the single sought transformation

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values               v;

    add_gnss_factors(graph, v, smFrames, params.fgParams);

    gtsam::LevenbergMarquardtParams lmParams = gtsam::LevenbergMarquardtParams::CeresDefaults();

    gtsam::LevenbergMarquardtOptimizer lm(graph, v, lmParams);

    auto optimal = lm.optimize();

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

    ret.geo_ref->T_enu_to_map = {
        mrpt::poses::CPose3D(mrpt::gtsam_wrappers::toTPose3D(T0)),
        mrpt::gtsam_wrappers::to_mrpt_se3_cov6(T0_cov)};

    ret.geo_ref->geo_coord = *smFrames.refCoord;

    ret.final_rmse = rmseEnd;

    return ret;
}

mola::GNSSFrames mola::extract_gnss_frames_from_sm(
    const mrpt::maps::CSimpleMap&                           sm,
    const std::optional<mrpt::topography::TGeodeticCoords>& refCoordIn)
{
    GNSSFrames ret;

    ret.refCoord = refCoordIn;

    ret.frames.reserve(sm.size());

    // Build list of KF poses with GNSS observations:
    for (const auto& [pose, sf, twist] : sm)
    {
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

            auto& f = ret.frames.emplace_back();

            f.pose = p;
            f.obs  = obs;
            f.gga  = obs->getMsgByClass<mrpt::obs::gnss::Message_NMEA_GGA>();

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

            if (f.sigma_E <= 0 || f.sigma_N <= 0 || f.sigma_U <= 0 || std::isnan(f.sigma_E) ||
                std::isnan(f.sigma_N) || std::isnan(f.sigma_U))
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
            gtsam::noiseModel::mEstimator::Huber::Create(1.5), noiseOrg);

        const auto observedENU = mrpt::gtsam_wrappers::toPoint3(frame.enu);
        const auto sensorPointOnVeh =
            mrpt::gtsam_wrappers::toPoint3(frame.obs->sensorPose.translation());

        fg.emplace_shared<mola::factors::FactorGnssEnu>(
            P(i), sensorPointOnVeh, observedENU, robustNoise);

        const auto vehiclePose = mrpt::gtsam_wrappers::toPose3(frame.pose);

        v.insert(P(i), vehiclePose);

        fg.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(T(0), P(i), vehiclePose, noisePoses);

        if (params.addHorizontalityConstraints)
        {
            fg.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
                P(i), gtsam::Pose3::Identity(), noiseHorizontality);
        }
    }
}
