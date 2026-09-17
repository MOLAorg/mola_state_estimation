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

/**
 * @file   test_gnss_and_imu_attitude.cpp
 * @brief  Checks that GNSS and IMU absolute-attitude factors agree when both
 *         are present in the same simplemap. The IMU-only and GNSS-only paths
 *         may each be self-consistent while still disagreeing on the frame
 *         their shared pose variables live in, which shows up only here.
 */

#include <gtsam/geometry/Rot3.h>
#include <mola_georeferencing/simplemap_georeference.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/gtsam_wrappers.h>
#include <mrpt/topography/conversions.h>

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{
void expect(bool cond, const std::string& msg)
{
    if (!cond)
    {
        throw std::runtime_error("Test assertion failed: " + msg);
    }
}

// Signed angle difference (a-b), wrapped to (-180, 180] degrees.
double angle_diff_deg(double a_deg, double b_deg)
{
    return std::fmod(a_deg - b_deg + 540.0, 360.0) - 180.0;
}

mrpt::topography::TGeodeticCoords geodetic_from_enu(
    const mrpt::math::TPoint3D& enuPoint, const mrpt::topography::TGeodeticCoords& origin)
{
    mrpt::topography::TGeocentricCoords geocentric;
    mrpt::topography::ENUToGeocentric(
        enuPoint, origin, geocentric, mrpt::topography::TEllipsoid::Ellipsoid_WGS84());

    mrpt::topography::TGeodeticCoords geodetic;
    mrpt::topography::geocentricToGeodetic(
        geocentric, geodetic, mrpt::topography::TEllipsoid::Ellipsoid_WGS84());

    return geodetic;
}

/// Builds a simplemap where every keyframe carries a GNSS fix AND an IMU
/// absolute-attitude reading, both generated from the same ground truth.
/// The path turns as it advances, so azimuth is observable from GNSS alone too.
mrpt::maps::CSimpleMap build_gnss_plus_attitude_map(
    const mrpt::poses::CPose3D& T_enu_to_map, const mrpt::topography::TGeodeticCoords& origin,
    size_t nKeyframes, bool addGravity)
{
    mrpt::maps::CSimpleMap sm;

    const mrpt::poses::CPose3D  T_veh_to_antenna(0.35, -0.12, 1.40, 0, 0, 0);
    const mrpt::poses::CPose3D  T_veh_to_imu(0.05, 0.0, 0.50, 0, 0, 0);
    const mrpt::math::TVector3D g_enu(0, 0, 9.81);

    for (size_t i = 0; i < nKeyframes; i++)
    {
        const double s = static_cast<double>(i);

        // A gently turning, mildly climbing path in the {map} frame:
        const mrpt::poses::CPose3D T_map_to_veh(
            1.5 * s, 0.03 * s * s, 0.02 * s, mrpt::DEG2RAD(2.0 * s), 0, 0);

        auto pose_pdf  = mrpt::poses::CPose3DPDFGaussian::Create();
        pose_pdf->mean = T_map_to_veh;

        auto sf = mrpt::obs::CSensoryFrame::Create();

        // --- GNSS ---
        {
            const mrpt::poses::CPose3D T_enu_to_antenna =
                T_enu_to_map + T_map_to_veh + T_veh_to_antenna;
            const auto coords = geodetic_from_enu(T_enu_to_antenna.translation(), origin);

            auto obs = mrpt::obs::CObservationGPS::Create();

            mrpt::obs::gnss::Message_NMEA_GGA gga;
            gga.fields.latitude_degrees  = coords.lat.getDecimalValue();
            gga.fields.longitude_degrees = coords.lon.getDecimalValue();
            gga.fields.altitude_meters   = coords.height;
            gga.fields.fix_quality       = 4;  // RTK fixed
            gga.fields.thereis_HDOP      = true;
            gga.fields.HDOP              = 1.0f;
            obs->setMsg(gga);

            obs->sensorPose = T_veh_to_antenna;

            mrpt::math::CMatrixDouble33 cov;
            cov.setDiagonal(0.05 * 0.05);
            obs->covariance_enu = cov;

            sf->insert(obs);
        }

        // --- IMU ---
        {
            const mrpt::poses::CPose3D T_enu_to_imu = T_enu_to_map + T_map_to_veh + T_veh_to_imu;

            auto obs        = mrpt::obs::CObservationIMU::Create();
            obs->sensorPose = T_veh_to_imu;

            if (addGravity)
            {
                const mrpt::math::TVector3D a_sensor = T_enu_to_imu.inverseRotateVector(g_enu);
                obs->set(mrpt::obs::IMU_X_ACC, a_sensor.x);
                obs->set(mrpt::obs::IMU_Y_ACC, a_sensor.y);
                obs->set(mrpt::obs::IMU_Z_ACC, a_sensor.z);
            }

            // Undo the fixed ENU convention (yaw=0 => East) applied by
            // imu_apply_enu_azimuth_correction(), to synthesize the raw,
            // north-referenced reading a real IMU driver would report:
            const gtsam::Rot3 enuAttitude = mrpt::gtsam_wrappers::toPose3(T_enu_to_imu).rotation();
            const gtsam::Rot3 rawAttitude = gtsam::Rot3::Rz(mrpt::DEG2RAD(-90.0)) * enuAttitude;

            const auto q = rawAttitude.toQuaternion();
            obs->set(mrpt::obs::IMU_ORI_QUAT_W, q.w());
            obs->set(mrpt::obs::IMU_ORI_QUAT_X, q.x());
            obs->set(mrpt::obs::IMU_ORI_QUAT_Y, q.y());
            obs->set(mrpt::obs::IMU_ORI_QUAT_Z, q.z());

            sf->insert(obs);
        }

        sm.insert(pose_pdf, sf);
    }

    return sm;
}

mrpt::poses::CPose3D solve(
    const mrpt::maps::CSimpleMap& sm, const mrpt::topography::TGeodeticCoords& origin,
    bool useAttitude, bool useGravity)
{
    mola::SMGeoReferencingParams params;
    params.geodeticReference                        = origin;
    params.useIMUAttitudeAlignment                  = useAttitude;
    params.useIMUGravityAlignment                   = useGravity;
    params.imuAttitudeParams.imuAttitudeSigmaDeg    = 1.0;
    params.imuAttitudeParams.imuAttitudeYawSigmaDeg = 5.0;

    const auto out = mola::simplemap_georeference(sm, params);
    expect(out.geo_ref.has_value(), "Georeferencing should succeed");

    return out.geo_ref->T_enu_to_map.mean;
}

void expect_close(
    const mrpt::poses::CPose3D& est, const mrpt::poses::CPose3D& gt, double tol_m, double tol_deg,
    const std::string& what)
{
    std::cout << "  [" << what << "] estimated: " << est.asString() << "\n";

    expect((est.translation() - gt.translation()).norm() < tol_m, what + ": translation");
    expect(
        std::abs(angle_diff_deg(mrpt::RAD2DEG(est.yaw()), mrpt::RAD2DEG(gt.yaw()))) < tol_deg,
        what + ": yaw");
    expect(
        std::abs(angle_diff_deg(mrpt::RAD2DEG(est.pitch()), mrpt::RAD2DEG(gt.pitch()))) < tol_deg,
        what + ": pitch");
    expect(
        std::abs(angle_diff_deg(mrpt::RAD2DEG(est.roll()), mrpt::RAD2DEG(gt.roll()))) < tol_deg,
        what + ": roll");
}

// The core regression: GNSS and IMU attitude must both be expressed against the
// same pose variables, so adding IMU attitude to a map that GNSS already solves
// must refine the answer, never move it.
void test_gnss_and_attitude_agree()
{
    const mrpt::poses::CPose3D T_enu_to_map(
        12.0, -7.0, 3.0, mrpt::DEG2RAD(-150.0), mrpt::DEG2RAD(1.5), mrpt::DEG2RAD(-2.0));

    const mrpt::topography::TGeodeticCoords origin(36.878, -2.338, 100.0);

    const auto sm = build_gnss_plus_attitude_map(T_enu_to_map, origin, 40, /*addGravity=*/false);

    const auto gnssOnly = solve(sm, origin, /*useAttitude=*/false, /*useGravity=*/false);
    const auto withImu  = solve(sm, origin, /*useAttitude=*/true, /*useGravity=*/false);

    expect_close(gnssOnly, T_enu_to_map, 0.10, 0.5, "GNSS only");
    expect_close(withImu, T_enu_to_map, 0.10, 0.5, "GNSS + IMU attitude");

    // And, explicitly, the two must not disagree with each other:
    expect(
        std::abs(angle_diff_deg(mrpt::RAD2DEG(withImu.yaw()), mrpt::RAD2DEG(gnssOnly.yaw()))) < 0.5,
        "enabling IMU attitude must not shift the GNSS-derived azimuth");
    expect(
        (withImu.translation() - gnssOnly.translation()).norm() < 0.10,
        "enabling IMU attitude must not shift the GNSS-derived translation");
}

void test_gnss_attitude_and_gravity_agree()
{
    const mrpt::poses::CPose3D T_enu_to_map(
        -5.0, 4.0, -2.0, mrpt::DEG2RAD(95.0), mrpt::DEG2RAD(-1.0), mrpt::DEG2RAD(0.8));

    const mrpt::topography::TGeodeticCoords origin(36.878, -2.338, 100.0);

    const auto sm = build_gnss_plus_attitude_map(T_enu_to_map, origin, 40, /*addGravity=*/true);

    const auto all = solve(sm, origin, /*useAttitude=*/true, /*useGravity=*/true);

    expect_close(all, T_enu_to_map, 0.10, 0.5, "GNSS + attitude + gravity");
}

}  // namespace

int main()
{
    try
    {
        test_gnss_and_attitude_agree();
        test_gnss_attitude_and_gravity_agree();

        std::cout << "\n[Success] GNSS + IMU attitude fusion tests passed!" << std::endl;
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "\n[Test Failed] " << e.what() << std::endl;
        return 1;
    }
}
