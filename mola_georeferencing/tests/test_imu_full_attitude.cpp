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
 * @file   test_imu_full_attitude.cpp
 * @brief  Validates full-attitude (roll+pitch+yaw) IMU fusion in
 *         simplemap_georeference() with synthetic, noisy data: recovering
 *         azimuth from IMU alone (no GNSS), the azimuthOffsetDeg calibration
 *         parameter, and combined gravity+attitude fusion.
 */

#include <gtsam/geometry/Rot3.h>
#include <mola_georeferencing/simplemap_georeference.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/gtsam_wrappers.h>

#include <cmath>
#include <iostream>
#include <random>
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

// Builds a simplemap with `nKeyframes` synthetic, independently-noisy IMU
// absolute-orientation (quaternion) readings for a vehicle purely translating
// (constant orientation) under a known ground-truth T_enu_to_map and sensor
// mounting. `trueAzimuthOffsetDeg` simulates a real sensor-to-vehicle azimuth
// miscalibration baked into the raw readings (as a real IMU driver would
// report, before any user calibration is applied).
mrpt::maps::CSimpleMap build_synthetic_attitude_map(
    double roll_deg, double pitch_deg, double yaw_deg, double sensor_yaw_deg,
    double trueAzimuthOffsetDeg, double noiseSigmaDeg, size_t nKeyframes, unsigned seed)
{
    mrpt::maps::CSimpleMap sm;

    const mrpt::poses::CPose3D T_enu_to_map(
        0, 0, 0, mrpt::DEG2RAD(yaw_deg), mrpt::DEG2RAD(pitch_deg), mrpt::DEG2RAD(roll_deg));
    const mrpt::poses::CPose3D T_veh_to_sensor(0, 0, 0, mrpt::DEG2RAD(sensor_yaw_deg), 0, 0);

    std::mt19937                     rng(seed);
    std::normal_distribution<double> noise(0.0, mrpt::DEG2RAD(noiseSigmaDeg));

    for (size_t i = 0; i < nKeyframes; i++)
    {
        const mrpt::poses::CPose3D T_map_to_veh(static_cast<double>(i) * 1.5, 0, 0, 0, 0, 0);

        auto pose_pdf  = mrpt::poses::CPose3DPDFGaussian::Create();
        pose_pdf->mean = T_map_to_veh;

        auto sf = mrpt::obs::CSensoryFrame::Create();

        auto obs_imu        = mrpt::obs::CObservationIMU::Create();
        obs_imu->sensorPose = T_veh_to_sensor;

        const mrpt::poses::CPose3D T_enu_to_sensor = T_enu_to_map + T_map_to_veh + T_veh_to_sensor;
        const gtsam::Rot3 predicted = mrpt::gtsam_wrappers::toPose3(T_enu_to_sensor).rotation();

        // Undo the fixed ENU convention (yaw=0 => East) plus the true azimuth
        // offset, mirroring exactly the inverse of what
        // mola::factors::imu_apply_enu_azimuth_correction() applies, to
        // synthesize the raw reading a real IMU driver would report:
        const gtsam::Rot3 rawAttitude =
            gtsam::Rot3::Rz(mrpt::DEG2RAD(-(90.0 + trueAzimuthOffsetDeg))) * predicted;

        // Inject independent per-sample orientation noise:
        const gtsam::Rot3 noisyRaw =
            gtsam::Rot3::Ypr(noise(rng), noise(rng), noise(rng)) * rawAttitude;

        const auto q = noisyRaw.toQuaternion();
        obs_imu->set(mrpt::obs::IMU_ORI_QUAT_W, q.w());
        obs_imu->set(mrpt::obs::IMU_ORI_QUAT_X, q.x());
        obs_imu->set(mrpt::obs::IMU_ORI_QUAT_Y, q.y());
        obs_imu->set(mrpt::obs::IMU_ORI_QUAT_Z, q.z());

        sf->insert(obs_imu);
        sm.insert(pose_pdf, sf);
    }

    return sm;
}

void test_noisy_full_attitude_recovers_yaw_without_gnss()
{
    const double roll_deg = 4.0, pitch_deg = -6.0, yaw_deg = 35.0, sensor_yaw_deg = 20.0;
    const double azimuthOffsetDeg = 0.0;
    const double noiseSigmaDeg    = 3.0;
    const size_t nKeyframes       = 40;

    const auto sm = build_synthetic_attitude_map(
        roll_deg, pitch_deg, yaw_deg, sensor_yaw_deg, azimuthOffsetDeg, noiseSigmaDeg, nKeyframes,
        /*seed=*/42);

    mola::SMGeoReferencingParams params;
    params.useIMUGravityAlignment                   = false;
    params.useIMUAttitudeAlignment                  = true;
    params.imuAttitudeParams.imuAttitudeSigmaDeg    = noiseSigmaDeg;
    params.imuAttitudeParams.imuAttitudeYawSigmaDeg = noiseSigmaDeg;
    params.imuAttitudeParams.azimuthOffsetDeg       = azimuthOffsetDeg;

    const auto out = mola::simplemap_georeference(sm, params);
    expect(out.geo_ref.has_value(), "Georeferencing from IMU attitude alone should succeed");

    const auto& T_est = out.geo_ref->T_enu_to_map.mean;

    std::cout << "  [noisy attitude] Estimated RPY: (" << mrpt::RAD2DEG(T_est.roll()) << ", "
              << mrpt::RAD2DEG(T_est.pitch()) << ", " << mrpt::RAD2DEG(T_est.yaw()) << ") deg\n";

    const double tol_deg = 2.0;  // ~ noiseSigmaDeg / sqrt(nKeyframes), with margin
    expect(
        std::abs(angle_diff_deg(mrpt::RAD2DEG(T_est.roll()), roll_deg)) < tol_deg,
        "roll should be recovered despite per-sample noise");
    expect(
        std::abs(angle_diff_deg(mrpt::RAD2DEG(T_est.pitch()), pitch_deg)) < tol_deg,
        "pitch should be recovered despite per-sample noise");
    expect(
        std::abs(angle_diff_deg(mrpt::RAD2DEG(T_est.yaw()), yaw_deg)) < tol_deg,
        "yaw (azimuth) should be recovered from IMU attitude alone, without any GNSS data");
}

void test_azimuth_offset_param_compensates_known_bias()
{
    const double roll_deg = 0.0, pitch_deg = 0.0, yaw_deg = 50.0, sensor_yaw_deg = 0.0;
    const double trueAzimuthOffsetDeg = 12.0;  // e.g. sensor mounting misalignment
    const double noiseSigmaDeg        = 1.0;
    const size_t nKeyframes           = 30;

    const auto sm = build_synthetic_attitude_map(
        roll_deg, pitch_deg, yaw_deg, sensor_yaw_deg, trueAzimuthOffsetDeg, noiseSigmaDeg,
        nKeyframes, /*seed=*/7);

    // Case A: uncompensated (azimuthOffsetDeg left at its default, 0) -> yaw
    // should end up biased by approximately the true (unmodeled) offset.
    {
        mola::SMGeoReferencingParams params;
        params.useIMUGravityAlignment                   = false;
        params.useIMUAttitudeAlignment                  = true;
        params.imuAttitudeParams.imuAttitudeSigmaDeg    = 5.0;
        params.imuAttitudeParams.imuAttitudeYawSigmaDeg = 5.0;
        // params.imuAttitudeParams.azimuthOffsetDeg left at its default (0.0)

        const auto out = mola::simplemap_georeference(sm, params);
        expect(
            out.geo_ref.has_value(), "Georeferencing should still succeed even if miscalibrated");

        const double yaw_err_deg =
            std::abs(angle_diff_deg(mrpt::RAD2DEG(out.geo_ref->T_enu_to_map.mean.yaw()), yaw_deg));

        expect(
            yaw_err_deg > trueAzimuthOffsetDeg - 2.0,
            "leaving azimuthOffsetDeg uncompensated should bias yaw by ~ the true offset");
    }

    // Case B: compensated with the correct azimuthOffsetDeg -> yaw matches truth.
    {
        mola::SMGeoReferencingParams params;
        params.useIMUGravityAlignment                   = false;
        params.useIMUAttitudeAlignment                  = true;
        params.imuAttitudeParams.imuAttitudeSigmaDeg    = noiseSigmaDeg;
        params.imuAttitudeParams.imuAttitudeYawSigmaDeg = noiseSigmaDeg;
        params.imuAttitudeParams.azimuthOffsetDeg       = trueAzimuthOffsetDeg;

        const auto out = mola::simplemap_georeference(sm, params);
        expect(out.geo_ref.has_value(), "Georeferencing should succeed once compensated");

        const double yaw_err_deg =
            std::abs(angle_diff_deg(mrpt::RAD2DEG(out.geo_ref->T_enu_to_map.mean.yaw()), yaw_deg));

        expect(
            yaw_err_deg < 1.5, "azimuthOffsetDeg correctly configured should recover the true yaw");
    }
}

void test_gravity_and_attitude_factors_combine_without_conflict()
{
    // Build a map with BOTH noisy accelerometer and noisy full-attitude
    // readings on the same observations, and check the combined optimization
    // still recovers the ground truth (i.e. the two factor types agree).
    const double roll_deg = 10.0, pitch_deg = 15.0, yaw_deg = -40.0, sensor_yaw_deg = 0.0;
    const double noiseSigmaDeg = 2.0;
    const size_t nKeyframes    = 30;

    mrpt::maps::CSimpleMap sm;

    const mrpt::poses::CPose3D T_enu_to_map(
        0, 0, 0, mrpt::DEG2RAD(yaw_deg), mrpt::DEG2RAD(pitch_deg), mrpt::DEG2RAD(roll_deg));
    const mrpt::poses::CPose3D  T_veh_to_sensor(0, 0, 0, mrpt::DEG2RAD(sensor_yaw_deg), 0, 0);
    const mrpt::math::TVector3D g_enu(0, 0, 9.81);

    std::mt19937                     rng(123);
    std::normal_distribution<double> ang_noise(0.0, mrpt::DEG2RAD(noiseSigmaDeg));
    std::normal_distribution<double> acc_noise(0.0, 0.15);  // [m/s^2]

    for (size_t i = 0; i < nKeyframes; i++)
    {
        const mrpt::poses::CPose3D T_map_to_veh(static_cast<double>(i) * 1.5, 0, 0, 0, 0, 0);

        auto pose_pdf  = mrpt::poses::CPose3DPDFGaussian::Create();
        pose_pdf->mean = T_map_to_veh;

        auto sf = mrpt::obs::CSensoryFrame::Create();

        auto obs_imu        = mrpt::obs::CObservationIMU::Create();
        obs_imu->sensorPose = T_veh_to_sensor;

        const mrpt::poses::CPose3D T_enu_to_sensor = T_enu_to_map + T_map_to_veh + T_veh_to_sensor;

        // Noisy gravity (accelerometer) reading:
        mrpt::math::TVector3D a_sensor = T_enu_to_sensor.inverseRotateVector(g_enu);
        a_sensor.x += acc_noise(rng);
        a_sensor.y += acc_noise(rng);
        a_sensor.z += acc_noise(rng);
        obs_imu->set(mrpt::obs::IMU_X_ACC, a_sensor.x);
        obs_imu->set(mrpt::obs::IMU_Y_ACC, a_sensor.y);
        obs_imu->set(mrpt::obs::IMU_Z_ACC, a_sensor.z);

        // Noisy full-attitude (quaternion) reading, zero calibration offset:
        const gtsam::Rot3 predicted   = mrpt::gtsam_wrappers::toPose3(T_enu_to_sensor).rotation();
        const gtsam::Rot3 rawAttitude = gtsam::Rot3::Rz(mrpt::DEG2RAD(-90.0)) * predicted;
        const gtsam::Rot3 noisyRaw =
            gtsam::Rot3::Ypr(ang_noise(rng), ang_noise(rng), ang_noise(rng)) * rawAttitude;
        const auto q = noisyRaw.toQuaternion();
        obs_imu->set(mrpt::obs::IMU_ORI_QUAT_W, q.w());
        obs_imu->set(mrpt::obs::IMU_ORI_QUAT_X, q.x());
        obs_imu->set(mrpt::obs::IMU_ORI_QUAT_Y, q.y());
        obs_imu->set(mrpt::obs::IMU_ORI_QUAT_Z, q.z());

        sf->insert(obs_imu);
        sm.insert(pose_pdf, sf);
    }

    mola::SMGeoReferencingParams params;
    params.useIMUGravityAlignment                   = true;
    params.useIMUAttitudeAlignment                  = true;
    params.imuGravityParams.imuGravitySigmaDeg      = 3.0;
    params.imuAttitudeParams.imuAttitudeSigmaDeg    = noiseSigmaDeg;
    params.imuAttitudeParams.imuAttitudeYawSigmaDeg = noiseSigmaDeg;

    const auto out = mola::simplemap_georeference(sm, params);
    expect(out.geo_ref.has_value(), "Combined gravity+attitude georeferencing should succeed");

    const auto&  T_est   = out.geo_ref->T_enu_to_map.mean;
    const double tol_deg = 2.0;
    expect(
        std::abs(angle_diff_deg(mrpt::RAD2DEG(T_est.roll()), roll_deg)) < tol_deg,
        "combined gravity+attitude: roll recovered");
    expect(
        std::abs(angle_diff_deg(mrpt::RAD2DEG(T_est.pitch()), pitch_deg)) < tol_deg,
        "combined gravity+attitude: pitch recovered");
    expect(
        std::abs(angle_diff_deg(mrpt::RAD2DEG(T_est.yaw()), yaw_deg)) < tol_deg,
        "combined gravity+attitude: yaw recovered");
}

}  // namespace

int main()
{
    try
    {
        test_noisy_full_attitude_recovers_yaw_without_gnss();
        test_azimuth_offset_param_compensates_known_bias();
        test_gravity_and_attitude_factors_combine_without_conflict();

        std::cout << "\n[Success] All IMU full-attitude fusion tests passed!" << std::endl;
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "\n[Test Failed] " << e.what() << std::endl;
        return 1;
    }
}
