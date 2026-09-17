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
#include <mrpt/obs/CObservationComment.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/gtsam_wrappers.h>
#include <mrpt/topography/conversions.h>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

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

// Per-sensor ages baked into the synthetic keyframes, in seconds:
constexpr double kGnssAge = 0.04;  // GNSS reading precedes the keyframe instant
constexpr double kImuAge  = 0.02;  // IMU reading follows it

/// Reads a whitespace-separated diagnostic dump into rows of columns, skipping
/// the leading comment line.
std::vector<std::vector<std::string>> read_dump(const std::string& path)
{
    std::vector<std::vector<std::string>> rows;

    std::ifstream f(path);
    expect(f.is_open(), "diagnostic dump should have been written to " + path);

    std::string line;
    while (std::getline(f, line))
    {
        if (line.empty() || line[0] == '#')
        {
            continue;
        }
        std::istringstream       ss(line);
        std::vector<std::string> cols;
        std::string              tok;
        while (ss >> tok)
        {
            cols.push_back(tok);
        }
        rows.push_back(cols);
    }

    return rows;
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
    size_t nKeyframes, bool addGravity, bool addReferenceObs = false)
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

        // Distinct, deterministic per-sensor timestamps, so the diagnostic dumps
        // have something meaningful to report as each reading's age:
        const auto tKf   = mrpt::Clock::fromDouble(1.0e9 + 0.1 * s);
        const auto tGnss = mrpt::Clock::fromDouble(1.0e9 + 0.1 * s - kGnssAge);
        const auto tImu  = mrpt::Clock::fromDouble(1.0e9 + 0.1 * s + kImuAge);

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
            obs->timestamp  = tGnss;

            mrpt::math::CMatrixDouble33 cov;
            cov.setDiagonal(0.05 * 0.05);
            obs->covariance_enu = cov;

            sf->insert(obs);
        }

        // The observation that defines the keyframe (a LiDAR scan on a real
        // robot). Deliberately inserted after the GNSS one, so that picking the
        // keyframe's time reference by observation order alone would pick the
        // wrong one. Optional, to also exercise the GNSS+IMU-only keyframe:
        if (addReferenceObs)
        {
            auto obs         = mrpt::obs::CObservationComment::Create();
            obs->timestamp   = tKf;
            obs->sensorLabel = "metadata";
            sf->insert(obs);
        }

        // --- IMU ---
        {
            const mrpt::poses::CPose3D T_enu_to_imu = T_enu_to_map + T_map_to_veh + T_veh_to_imu;

            auto obs        = mrpt::obs::CObservationIMU::Create();
            obs->sensorPose = T_veh_to_imu;
            obs->timestamp  = tImu;

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

// The two diagnostic dumps must describe the same keyframes against the same
// time reference, otherwise their offset columns cannot be compared to each
// other, which is the whole point of having them.
void check_dumps_share_one_time_reference(bool withReferenceObs)
{
    const mrpt::poses::CPose3D T_enu_to_map(3.0, 1.0, -1.0, mrpt::DEG2RAD(-150.0), 0, 0);

    const mrpt::topography::TGeodeticCoords origin(36.878, -2.338, 100.0);

    const size_t nKeyframes = 12;
    const auto   sm         = build_gnss_plus_attitude_map(
                  T_enu_to_map, origin, nKeyframes, /*addGravity=*/false, withReferenceObs);

    const auto tmpDir   = std::filesystem::temp_directory_path();
    const auto gnssFile = (tmpDir / "test_georef_dump_gnss.txt").string();
    const auto attFile  = (tmpDir / "test_georef_dump_att.txt").string();

    std::filesystem::remove(gnssFile);
    std::filesystem::remove(attFile);

    ::setenv("MOLA_SM_GEOREF_DUMP_GNSS", gnssFile.c_str(), 1);
    ::setenv("MOLA_SM_GEOREF_DUMP_IMU_ATTITUDE", attFile.c_str(), 1);

    solve(sm, origin, /*useAttitude=*/true, /*useGravity=*/false);

    ::unsetenv("MOLA_SM_GEOREF_DUMP_GNSS");
    ::unsetenv("MOLA_SM_GEOREF_DUMP_IMU_ATTITUDE");

    const auto gnssRows = read_dump(gnssFile);
    const auto attRows  = read_dump(attFile);

    expect(gnssRows.size() == nKeyframes, "GNSS dump should hold one row per keyframe");
    expect(attRows.size() == nKeyframes, "IMU attitude dump should hold one row per keyframe");

    // Column layout, as written by the dump headers:
    constexpr size_t kGnssColKf = 0, kGnssColTkf = 1, kGnssColDt = 14;
    constexpr size_t kAttColKf = 0, kAttColDt = 23, kAttColTkf = 24;

    std::map<std::string, std::string> tkfByKeyframe;
    std::map<std::string, double>      dtGnssByKeyframe;
    for (const auto& r : gnssRows)
    {
        expect(r.size() > kGnssColDt, "GNSS dump row should have every documented column");
        tkfByKeyframe[r[kGnssColKf]]    = r[kGnssColTkf];
        dtGnssByKeyframe[r[kGnssColKf]] = std::stod(r[kGnssColDt]);

        if (withReferenceObs)
        {
            expect(
                std::abs(std::stod(r[kGnssColDt]) + kGnssAge) < 1e-3,
                "GNSS dump should report the reading's true age wrt the keyframe");
        }
    }

    for (const auto& r : attRows)
    {
        expect(r.size() > kAttColTkf, "IMU dump row should have every documented column");

        const auto it = tkfByKeyframe.find(r[kAttColKf]);
        expect(it != tkfByKeyframe.end(), "both dumps should cover the same keyframes");

        // The actual regression: one shared per-keyframe time reference. Whichever
        // observation it comes from, the two dumps must agree on it, so that the
        // gap between their age columns is the true GNSS-to-IMU gap:
        expect(
            it->second == r[kAttColTkf],
            "both dumps must report the same keyframe time reference, got '" + it->second +
                "' vs '" + r[kAttColTkf] + "'");
        expect(std::stod(r[kAttColTkf]) > 0, "the keyframe time reference should be defined");

        expect(
            std::abs(
                (std::stod(r[kAttColDt]) - dtGnssByKeyframe.at(r[kAttColKf])) -
                (kImuAge + kGnssAge)) < 1e-3,
            "the two age columns must be measured against the same instant");

        if (withReferenceObs)
        {
            expect(
                std::abs(std::stod(r[kAttColDt]) - kImuAge) < 1e-3,
                "IMU dump should report the reading's true age wrt the keyframe");
        }
    }

    std::filesystem::remove(gnssFile);
    std::filesystem::remove(attFile);
}

void test_diagnostic_dumps()
{
    // With a LiDAR-like observation defining the keyframe, and without one
    // (only GNSS + IMU), which takes the fallback path:
    check_dumps_share_one_time_reference(/*withReferenceObs=*/true);
    check_dumps_share_one_time_reference(/*withReferenceObs=*/false);

    std::cout << "  [diagnostic dumps] both dumps agree on the keyframe time reference\n";
}

}  // namespace

int main()
{
    try
    {
        test_gnss_and_attitude_agree();
        test_gnss_attitude_and_gravity_agree();
        test_diagnostic_dumps();

        std::cout << "\n[Success] GNSS + IMU attitude fusion tests passed!" << std::endl;
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "\n[Test Failed] " << e.what() << std::endl;
        return 1;
    }
}
