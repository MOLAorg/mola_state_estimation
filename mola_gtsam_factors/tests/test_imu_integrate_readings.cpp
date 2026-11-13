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

// integrate_imu_readings(): readings sampled from a known trajectory with
// strong angular accelerations and an IMU away from the body origin must give
// a near-zero preintegration residual at the true states.

#include <gtsam/inference/Symbol.h>
#include <mola_gtsam_factors/FactorImuPreintegrationBodyVel.h>
#include <mola_gtsam_factors/imu_preintegration.h>

#include <cmath>
#include <cstdio>
#include <deque>
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

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::T;
using gtsam::symbol_shorthand::V;

constexpr double kG    = 9.81;
constexpr double kRate = 200.0;  // [Hz]
constexpr double kDiff = 1e-4;  // [s] step of the numerical derivatives

// Body pose: a shaky, handheld-like motion.
gtsam::Pose3 body_pose(double t)
{
    const gtsam::Rot3 R = gtsam::Rot3::Ypr(
        0.3 * std::sin(4.0 * t), 0.15 * std::sin(6.0 * t), 0.1 * std::sin(7.0 * t));
    const gtsam::Point3 p(
        1.5 * t + 0.2 * std::sin(3.0 * t), 0.3 * std::sin(2.0 * t), 0.1 * std::sin(5.0 * t));
    return {R, p};
}

// Body-frame linear velocity of the body origin.
gtsam::Vector3 body_velocity(double t)
{
    const gtsam::Vector3 vw =
        (body_pose(t + kDiff).translation() - body_pose(t - kDiff).translation()) / (2 * kDiff);
    return body_pose(t).rotation().unrotate(vw);
}

// What an ideal IMU at body_P_sensor reads at time t.
mola::factors::ImuReading ideal_reading(double t, const gtsam::Pose3& body_P_sensor)
{
    const auto sensor_position = [&](double tt)
    { return body_pose(tt).transformFrom(body_P_sensor.translation()); };

    const gtsam::Vector3 accWorld =
        (sensor_position(t + kDiff) - 2 * sensor_position(t) + sensor_position(t - kDiff)) /
        (kDiff * kDiff);
    const gtsam::Rot3 Rws = body_pose(t).rotation() * body_P_sensor.rotation();
    const gtsam::Rot3 dR = body_pose(t - kDiff).rotation().between(body_pose(t + kDiff).rotation());
    const gtsam::Vector3 wBody = gtsam::Rot3::Logmap(dR) / (2 * kDiff);

    mola::factors::ImuReading r;
    r.t   = t;
    r.acc = Rws.unrotate(accWorld + gtsam::Vector3(0, 0, kG));
    r.gyr = body_P_sensor.rotation().unrotate(wBody);
    return r;
}

std::deque<mola::factors::ImuReading> ideal_readings(
    double from, double to, const gtsam::Pose3& body_P_sensor)
{
    std::deque<mola::factors::ImuReading> out;
    for (int k = 0; from + k / kRate <= to; k++)
    {
        out.push_back(ideal_reading(from + k / kRate, body_P_sensor));
    }
    return out;
}

auto make_params(const gtsam::Pose3& body_P_sensor)
{
    auto p = gtsam::PreintegrationParams::MakeSharedU(kG);
    p->setGyroscopeCovariance(1e-8 * gtsam::I_3x3);
    p->setAccelerometerCovariance(1e-6 * gtsam::I_3x3);
    p->setIntegrationCovariance(1e-8 * gtsam::I_3x3);
    p->setBodyPSensor(body_P_sensor);
    return p;
}

void test_residual_at_truth(const gtsam::Pose3& body_P_sensor, const std::string& name)
{
    const auto readings = ideal_readings(0.0, 3.0, body_P_sensor);
    const auto params   = make_params(body_P_sensor);

    double maxRot = 0;
    double maxPos = 0;
    double maxVel = 0;
    // Link ends off the sampling instants too, as keyframes of other sensors:
    for (double ti = 0.0123; ti + 0.1 < 2.9; ti += 0.1)
    {
        const double                        tj = ti + 0.1;
        gtsam::PreintegratedImuMeasurements pim(params, gtsam::imuBias::ConstantBias());
        expect(
            mola::factors::integrate_imu_readings(readings, ti, tj, 1.0, 0.1, pim),
            name + ": the interval must be covered");
        expect(std::abs(pim.deltaTij() - 0.1) < 1e-9, name + ": integrated duration");

        const mola::factors::FactorImuPreintegrationBodyVel f(T(0), V(0), T(1), V(1), B(0), pim);
        const gtsam::Vector                                 e = f.evaluateError(
                                            body_pose(ti), body_velocity(ti), body_pose(tj), body_velocity(tj),
                                            gtsam::imuBias::ConstantBias());
        expect(e.allFinite(), name + ": non-finite residual");
        maxRot = std::max(maxRot, e.segment<3>(0).lpNorm<Eigen::Infinity>());
        maxPos = std::max(maxPos, e.segment<3>(3).lpNorm<Eigen::Infinity>());
        maxVel = std::max(maxVel, e.segment<3>(6).lpNorm<Eigen::Infinity>());
    }
    std::printf(
        "%s: max residual rot=%.2e rad pos=%.2e m vel=%.2e m/s\n", name.c_str(), maxRot, maxPos,
        maxVel);
    // Per 0.1 s link, against angular accelerations of ~5 rad/s^2 and a ~0.5 m
    // lever arm. What is left is the discretization of gtsam's integrator:
    expect(maxRot < 3e-5, name + ": rotation residual " + std::to_string(maxRot));
    expect(maxPos < 3e-4, name + ": position residual " + std::to_string(maxPos));
    expect(maxVel < 5e-3, name + ": velocity residual " + std::to_string(maxVel));
}

void test_gaps_and_coverage()
{
    const gtsam::Pose3 bPs;
    auto               readings = ideal_readings(0.0, 1.0, bPs);
    const auto         params   = make_params(bPs);
    {
        gtsam::PreintegratedImuMeasurements pim(params, gtsam::imuBias::ConstantBias());
        expect(
            !mola::factors::integrate_imu_readings(readings, 0.9, 1.1, 1.0, 0.1, pim),
            "readings must reach the end of the interval");
    }
    // Drop 0.2 s of readings:
    std::deque<mola::factors::ImuReading> holed;
    for (const auto& r : readings)
    {
        if (r.t < 0.4 || r.t > 0.6)
        {
            holed.push_back(r);
        }
    }
    {
        gtsam::PreintegratedImuMeasurements pim(params, gtsam::imuBias::ConstantBias());
        expect(
            !mola::factors::integrate_imu_readings(holed, 0.3, 0.7, 1.0, 0.1, pim),
            "a gap longer than the maximum must fail");
    }
    {
        gtsam::PreintegratedImuMeasurements pim(params, gtsam::imuBias::ConstantBias());
        expect(
            mola::factors::integrate_imu_readings(holed, 0.1, 0.3, 1.0, 0.1, pim),
            "an interval away from the gap is fine");
    }
}

void test_interval_before_first_reading()
{
    // Two readings, accelerating along x from 0 to 10 m/s^2 between t=0 and
    // t=0.1. Over [-0.1, 0.1], the first one is held back to -0.1, then the
    // readings are interpolated: the velocity change is 0 + 0.1 * 5 = 0.5 m/s.
    std::deque<mola::factors::ImuReading> readings(2);
    readings[0].t   = 0.0;
    readings[0].acc = gtsam::Vector3::Zero();
    readings[0].gyr = gtsam::Vector3::Zero();
    readings[1].t   = 0.1;
    readings[1].acc = gtsam::Vector3(10.0, 0, 0);
    readings[1].gyr = gtsam::Vector3::Zero();

    gtsam::PreintegratedImuMeasurements pim(
        make_params(gtsam::Pose3()), gtsam::imuBias::ConstantBias());
    expect(
        mola::factors::integrate_imu_readings(readings, -0.1, 0.1, 1.0, 0.11, pim),
        "an interval starting shortly before the first reading is covered");
    expect(
        std::abs(pim.deltaVij().x() - 0.5) < 1e-9,
        "held first reading: dv=" + std::to_string(pim.deltaVij().x()));
}

}  // namespace

int main()
{
    try
    {
        test_residual_at_truth(gtsam::Pose3(), "IMU at the body origin");
        test_residual_at_truth(
            gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.05, -0.03, 0.45)), "lever arm");
        test_residual_at_truth(
            gtsam::Pose3(gtsam::Rot3::Ypr(0.3, -0.2, 0.1), gtsam::Point3(0.05, -0.03, 0.45)),
            "lever arm and rotated IMU");
        test_gaps_and_coverage();
        test_interval_before_first_reading();
        std::cout << "All tests passed." << std::endl;
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }
}
