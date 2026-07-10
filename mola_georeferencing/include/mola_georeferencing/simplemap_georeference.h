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

#pragma once

#include <mp2p_icp/metricmap.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/system/COutputLogger.h>

// GTSAM:
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

namespace mola
{
struct SMGeoReferencingOutput
{
    SMGeoReferencingOutput() = default;

    /// Will be nullopt if georeferencing failed, e.g. due to insufficient GNSS
    /// data.
    std::optional<mp2p_icp::metric_map_t::Georeferencing> geo_ref;

    /// True if `geo_ref->geo_coord` was actually derived from GNSS data.
    /// False (default) means it was left at its zero-initialized value, e.g.
    /// because only IMU data was available for gravity alignment; in that
    /// case, `geo_ref->geo_coord` must NOT be treated as a real geodetic
    /// reference (e.g. for datum re-centering).
    bool has_geodetic_datum = false;

    double final_rmse = .0;
};

struct AddGNSSFactorParams
{
    /// May be required for small maps, i.e. when the length of the trajectory
    /// is not >10 times the GNSS uncertainty.
    bool   addHorizontalityConstraints = false;
    double horizontalitySigmaZ         = 1.0;  // [m]

    double minimumUncertaintyXYZ = 0.20;  // [m]

    /// Parameter "k" (in whitened/normalized units) of the Huber robust kernel
    /// applied to each GNSS position factor. Residuals below this threshold are
    /// treated quadratically; larger ones are down-weighted linearly.
    double robustParamHuberK = 1.5;
};

struct AddIMUGravityFactorParams
{
    double imuGravitySigmaDeg = 3.0;  // [deg]
};

struct AddIMUAttitudeFactorParams
{
    /// Sigma for the roll/pitch components of the absolute-attitude factor [deg].
    double imuAttitudeSigmaDeg = 3.0;

    /// Sigma for the yaw (azimuth) component [deg]. IMU-fused yaw (often
    /// magnetometer-based) is typically much noisier than roll/pitch, hence
    /// this defaults higher. Set equal to imuAttitudeSigmaDeg for isotropic
    /// noise.
    double imuAttitudeYawSigmaDeg = 15.0;

    /// Fixed calibration offset (degrees) between the IMU's zero-yaw reading
    /// and true azimuth, e.g. due to sensor mounting or magnetic declination.
    double azimuthOffsetDeg = 0.0;
};

struct SMGeoReferencingParams
{
    SMGeoReferencingParams() = default;

    /// If provided, this will be the coordinates of the ENU frame origin.
    /// Otherwise (default), the first GNSS entry will become the reference.
    std::optional<mrpt::topography::TGeodeticCoords> geodeticReference;

    /// If non-zero, GNSS frames whose NMEA GGA fix quality is below this
    /// threshold will be discarded (e.g. 1=GPS, 2=DGPS, 4=RTK fixed, 5=RTK
    /// float). Default (0) disables the filter, keeping all valid frames.
    unsigned int minimumGNSSFixQuality = 0;

    AddGNSSFactorParams fgParams;

    /// If true, use IMU acceleration data found in the simplemap to add
    /// gravity-alignment constraints (MeasuredGravityFactor) to the
    /// optimization. This helps in cases with poor Z data in GPS.
    bool                      useIMUGravityAlignment = true;
    AddIMUGravityFactorParams imuGravityParams;

    /// If true, use IMU absolute-orientation data (e.g. quaternion fields
    /// filled in by an onboard AHRS) found in the simplemap to add full
    /// attitude constraints (Pose3RotationFactor) to the optimization. Unlike
    /// gravity-only alignment, this also observes azimuth (yaw), so it can
    /// remove the need for GNSS to solve for it.
    bool                       useIMUAttitudeAlignment = true;
    AddIMUAttitudeFactorParams imuAttitudeParams;

    mrpt::system::COutputLogger* logger = nullptr;
};

/** Function to georeferencing a given SimpleMap with GNSS observations.
 *  A minimum of 3 (non-colinear) KFs with GNSS data are required to solve
 *  for the optimal transformation.
 */
SMGeoReferencingOutput simplemap_georeference(
    const mrpt::maps::CSimpleMap& sm, const SMGeoReferencingParams& params = {});

/** Re-datums a geo-referencing so that its `T_enu_to_map` has exactly the given
 *  translation, shifting the geodetic datum (`geo_coord`) accordingly so that
 *  the map <-> geodetic mapping is left completely unchanged. Since the local
 *  ENU axes rotate as the datum moves on the curved Earth, the rotation of
 *  `T_enu_to_map` is only approximately preserved (exactly so for small datum
 *  shifts); the 6D covariance is propagated accordingly, not just copied.
 *
 *  The ENU frame origin (datum) ends up placed at the map point located at
 *  \a desiredEnuToMapTranslation (since `T_enu_to_map` maps ENU (0,0,0) to its
 *  own translation).
 *
 *  This is useful to force the ENU origin to a chosen map location (e.g. the map
 *  origin, passing {0,0,0}) instead of the arbitrary first-GNSS position, which
 *  may be several meters from the map origin and make it awkward to pick an
 *  initial pose for localization.
 *
 *  \param in The input geo-referencing (datum + T_enu_to_map).
 *  \param desiredEnuToMapTranslation The desired translation of T_enu_to_map.
 *  \return The re-datumed, geometrically-equivalent geo-referencing.
 */
mp2p_icp::metric_map_t::Georeferencing recenter_georeference(
    const mp2p_icp::metric_map_t::Georeferencing& in,
    const mrpt::math::TPoint3D&                   desiredEnuToMapTranslation);

struct FrameGNSS
{
    mrpt::poses::CPose3D              pose;
    size_t                            kf_index = 0;  //!< Index in the source CSimpleMap
    mrpt::obs::CObservationGPS::Ptr   obs;
    mrpt::obs::gnss::Message_NMEA_GGA gga;
    mrpt::topography::TGeodeticCoords coords;
    mrpt::math::TPoint3D              enu;
    double                            sigma_E = 5.0;
    double                            sigma_N = 5.0;
    double                            sigma_U = 5.0;
};

struct GNSSFrames
{
    std::vector<FrameGNSS>                           frames;
    std::optional<mrpt::topography::TGeodeticCoords> refCoord;

    /// Set to true by extract_gnss_frames_from_sm() when the spatial spread of
    /// the GNSS observations is too small with respect to their uncertainty
    /// (specifically, when the ENU bounding-box diagonal is not larger than 3x
    /// the minimum per-axis sigma). In such a degenerate configuration, the
    /// georeferencing/global-attitude problem is ill-conditioned (e.g. the map
    /// roll/pitch becomes unobservable and can take absurd values).
    bool possibly_degenerate = false;
};

GNSSFrames extract_gnss_frames_from_sm(
    const mrpt::maps::CSimpleMap&                           sm,
    const std::optional<mrpt::topography::TGeodeticCoords>& refCoord          = std::nullopt,
    unsigned int                                            minimumFixQuality = 0);

struct FrameIMU
{
    size_t               kf_index = 0;  //!< Index in the source CSimpleMap
    mrpt::poses::CPose3D vehiclePose;
    mrpt::poses::CPose3D sensorPoseOnVehicle;

    /// Set if this keyframe has a valid gravity (accelerometer) observation.
    std::optional<gtsam::Vector3> normalizedAcc;

    /// Set if this keyframe has a valid absolute-orientation observation.
    /// Stored raw (not yet corrected for the ENU/azimuth convention), since
    /// that correction depends on AddIMUAttitudeFactorParams::azimuthOffsetDeg,
    /// only known when factors are built, not at extraction time.
    std::optional<gtsam::Rot3> rawAttitude;
};

struct IMUFrames
{
    std::vector<FrameIMU> frames;
};

/// Single pass over the simplemap collecting both gravity (accelerometer)
/// and absolute-attitude (orientation) observations per keyframe.
IMUFrames extract_imu_frames_from_sm(const mrpt::maps::CSimpleMap& sm);

/// Adds MeasuredGravityFactor factors to the graph. Creates new P(kf_index)
/// variables for keyframes not already present in \a existingPoseKeys.
void add_imu_gravity_factors(
    gtsam::NonlinearFactorGraph& fg, gtsam::Values& v, const IMUFrames& imuFrames,
    const std::set<size_t>& existingPoseKeys, const AddIMUGravityFactorParams& params);

/// Adds Pose3RotationFactor factors to the graph. Creates new P(kf_index)
/// variables for keyframes not already present in \a existingPoseKeys.
void add_imu_attitude_factors(
    gtsam::NonlinearFactorGraph& fg, gtsam::Values& v, const IMUFrames& imuFrames,
    const std::set<size_t>& existingPoseKeys, const AddIMUAttitudeFactorParams& params);

void add_gnss_factors(
    gtsam::NonlinearFactorGraph& fg, gtsam::Values& v, const GNSSFrames& frames,
    const AddGNSSFactorParams& params);

/// \deprecated Use FrameIMU instead. (Not itself tagged [[deprecated]] since
/// it is only ever referenced through IMUAccFrames, whose own tag already
/// flags it to callers; doing so here would additionally warn on every
/// translation unit that merely includes this header.)
struct FrameIMUAcc
{
    size_t               kf_index = 0;  //!< Index in the source CSimpleMap
    mrpt::poses::CPose3D vehiclePose;
    mrpt::poses::CPose3D sensorPoseOnVehicle;
    gtsam::Vector3       normalizedAcc;
};

/// \deprecated Use IMUFrames instead.
struct [[deprecated("Use IMUFrames instead.")]] IMUAccFrames { std::vector<FrameIMUAcc> frames; };

/// \deprecated Use extract_imu_frames_from_sm() instead, which also extracts
/// absolute-attitude readings alongside gravity ones. This wrapper is kept
/// for backward compatibility and simply forwards to it, returning only the
/// gravity (accelerometer) subset.
[[deprecated("Use extract_imu_frames_from_sm() instead.")]] IMUAccFrames
    extract_imu_acc_frames_from_sm(const mrpt::maps::CSimpleMap& sm);

}  // namespace mola
