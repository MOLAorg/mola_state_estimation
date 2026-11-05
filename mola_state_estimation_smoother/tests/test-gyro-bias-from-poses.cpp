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
 * @file   test-gyro-bias-from-poses.cpp
 * @brief  imu_preintegration gyro mode: a constant gyroscope bias is recovered from a pose source
 *         while the vehicle keeps turning (no standstill), and the fused
 *         trajectory follows the pose source.
 * @author Jose Luis Blanco Claraco
 * @date   Nov 5, 2026
 */

#include <mola_kernel/interfaces/ImuBiasSource.h>
#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/Lie/SE.h>
#include <mrpt/random/RandomGenerators.h>

#include <cmath>
#include <iostream>

namespace
{
const char* params = R"###(
params:
    vehicle_frame_name: "base_link"
    reference_frame_name: "map"
    kinematic_model: KinematicModel::ConstantVelocity
    sliding_window_length: 5.0
    max_time_to_use_velocity_model: 0.75
    relative_pose_increment_sigma_lin: 0.02
    relative_pose_increment_sigma_ang: 0.005
    async_backend: false
    imu_normalized_gravity_alignment_sigma: 0
    imu_preintegration:
      mode: gyro
)###";

auto& rng = mrpt::random::getRandomGenerator();

void run_test()
{
    rng.randomize(3);

    mola::state_estimation_smoother::StateEstimationSmoother est;
    est.initialize(mrpt::containers::yaml::FromText(params));

    const double T       = 0.1;  // [s] poses
    const double TIMU    = 0.005;  // [s]
    const double V       = 1.0;  // [m/s]
    const double W       = 0.3;  // [rad/s], a constant turn
    const double BIAS    = mrpt::DEG2RAD(0.6);  // [rad/s], on every axis
    const double DUR     = 30.0;  // [s]
    const auto   imuPose = mrpt::poses::CPose3D::FromXYZYawPitchRoll(0.05, 0, 0.1, M_PI / 2, 0, 0);

    auto   odom      = mrpt::poses::CPose3D::Identity();
    double maxErrLin = 0;
    double maxErrAng = 0;
    for (size_t i = 0; T * static_cast<double>(i) < DUR; i++)
    {
        const double t = T * static_cast<double>(i);
        // IMU readings over the next pose period, in the sensor frame:
        const auto wS = imuPose.inverseRotateVector({0, 0, W});
        const auto gS = imuPose.inverseRotateVector({0, 0, 9.81});
        for (double ti = t; ti < t + T - 1e-9; ti += TIMU)
        {
            auto imu         = mrpt::obs::CObservationIMU::Create();
            imu->timestamp   = mrpt::Clock::fromDouble(ti);
            imu->sensorLabel = "imu";
            imu->sensorPose  = imuPose;
            imu->set(mrpt::obs::IMU_WX, wS.x + BIAS + rng.drawGaussian1D(0, 0.002));
            imu->set(mrpt::obs::IMU_WY, wS.y + BIAS + rng.drawGaussian1D(0, 0.002));
            imu->set(mrpt::obs::IMU_WZ, wS.z + BIAS + rng.drawGaussian1D(0, 0.002));
            imu->set(mrpt::obs::IMU_X_ACC, gS.x);
            imu->set(mrpt::obs::IMU_Y_ACC, gS.y);
            imu->set(mrpt::obs::IMU_Z_ACC, gS.z);
            est.onNewObservation(imu);
        }
        if (i > 0)
        {
            odom = odom + mrpt::poses::CPose3D(V * T, 0, 0, W * T, 0, 0);
        }
        // A pose source with ~2 mm, ~0.1 deg of noise:
        mrpt::poses::CPose3DPDFGaussian pdf;
        pdf.mean = odom + mrpt::poses::CPose3D(
                              rng.drawGaussian1D(0, 0.002), rng.drawGaussian1D(0, 0.002),
                              rng.drawGaussian1D(0, 0.002), rng.drawGaussian1D(0, 0.002),
                              rng.drawGaussian1D(0, 0.002), rng.drawGaussian1D(0, 0.002));
        pdf.cov.setIdentity();
        pdf.cov *= 1e-4;
        est.fuse_pose(mrpt::Clock::fromDouble(t), pdf, "odom");
        const auto ns = est.estimated_navstate(mrpt::Clock::fromDouble(t), "odom");
        ASSERT_(ns.has_value());
        const auto err = mrpt::poses::Lie::SE<3>::log(mrpt::poses::CPose3D(ns->pose.mean) - odom);
        maxErrLin      = std::max(maxErrLin, err.blockCopy<3, 1>(0, 0).norm());
        maxErrAng      = std::max(maxErrAng, err.blockCopy<3, 1>(3, 0).norm());
    }
    std::cout << "max fused error: " << maxErrLin << " m, " << mrpt::RAD2DEG(maxErrAng) << " deg\n";
    ASSERT_LT_(maxErrLin, 0.02);
    ASSERT_LT_(maxErrAng, mrpt::DEG2RAD(0.6));

    // The estimate as offered to other modules (e.g. the LiDAR odometry de-skew):
    const auto b = static_cast<const mola::ImuBiasSource&>(est).imu_bias("imu");
    ASSERT_(b.has_value());
    std::cout << "gyro bias: true " << mrpt::RAD2DEG(BIAS) << " deg/s on each axis, estimated ["
              << mrpt::RAD2DEG(b->gyro.x) << " " << mrpt::RAD2DEG(b->gyro.y) << " "
              << mrpt::RAD2DEG(b->gyro.z) << "] deg/s\n";
    for (const double c : {b->gyro.x, b->gyro.y, b->gyro.z})
    {
        ASSERT_LT_(std::abs(c - BIAS), mrpt::DEG2RAD(0.1));
    }
}
}  // namespace

int main()
{
    try
    {
        run_test();
        std::cout << "Test successful." << std::endl;
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed:\n" << e.what() << std::endl;
        return 1;
    }
}
