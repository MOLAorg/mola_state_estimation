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

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <mola_gtsam_factors/FactorGyroPreintegration.h>
#include <mola_gtsam_factors/imu_preintegration.h>

#include <cmath>
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

auto make_params()
{
#if GTSAM_USES_BOOST
    auto p = boost::make_shared<gtsam::PreintegratedRotationParams>();
#else
    auto p = std::make_shared<gtsam::PreintegratedRotationParams>();
#endif
    p->setGyroscopeCovariance(1e-6 * gtsam::I_3x3);
    return p;
}

// One second of 100 Hz readings of a sensor rotating at a constant rate w (in
// the sensor frame), plus a constant gyro bias.
std::deque<mola::factors::ImuReading> make_readings(
    const gtsam::Vector3& w, const gtsam::Vector3& bias)
{
    std::deque<mola::factors::ImuReading> r;
    for (int k = 0; k <= 100; k++)
    {
        mola::factors::ImuReading x;
        x.t   = 10.0 + 0.01 * k;
        x.acc = gtsam::Vector3(0, 0, 9.8);  // ignored
        x.gyr = w + bias;
        r.push_back(x);
    }
    return r;
}

void test_zero_error_at_truth()
{
    // A sensor yawed on the vehicle: a yaw rate is the same in both frames, a bias is not.
    const gtsam::Pose3   bPs(gtsam::Rot3::Yaw(M_PI / 2), gtsam::Point3(0.2, 0, 0.5));
    const gtsam::Vector3 wSensor(0.0, 0.0, 0.3);
    const gtsam::Vector3 bias(0.001, -0.002, 0.004);
    const auto           readings = make_readings(wSensor, bias);

    gtsam::PreintegratedAhrsMeasurements pim(make_params(), gtsam::Vector3::Zero());
    expect(
        mola::factors::integrate_gyro_readings(readings, 10.2, 10.7, 0.05, bPs.rotation(), pim),
        "readings cover the interval");
    expect(std::abs(pim.deltaTij() - 0.5) < 1e-9, "integrated 0.5 s");

    const gtsam::Pose3 Ti(gtsam::Rot3::Ypr(0.5, 0.1, -0.2), gtsam::Point3(1, 2, 3));
    const gtsam::Pose3 Tj(Ti.rotation() * gtsam::Rot3::Yaw(0.3 * 0.5), gtsam::Point3(4, 5, 6));
    const mola::factors::FactorGyroPreintegration f(T(0), T(1), B(0), pim, bPs.rotation());

    // The bias is given in the sensor frame, as the readings:
    const gtsam::imuBias::ConstantBias trueBias(gtsam::Vector3(0.1, 0.2, 0.3), bias);
    const double                       e = f.evaluateError(Ti, Tj, trueBias).norm();
    expect(e < 1e-6, "zero error with the true gyro bias, got " + std::to_string(e));

    // Without the bias, the error is the bias integrated over the interval:
    const double eNoBias = f.evaluateError(Ti, Tj, gtsam::imuBias::ConstantBias()).norm();
    expect(
        std::abs(eNoBias - bias.norm() * 0.5) < 1e-4,
        "unmodeled bias error " + std::to_string(eNoBias));

    // Translations and the accelerometer bias do not matter:
    const gtsam::Pose3                 Tj2(Tj.rotation(), gtsam::Point3(-7, 1, 0));
    const gtsam::imuBias::ConstantBias otherAcc(gtsam::Vector3(-3, 2, 1), bias);
    expect(f.evaluateError(Ti, Tj2, otherAcc).norm() < 1e-6, "translation-free");
    std::cout << "[Test Passed] zero error at truth, bias, translation-free\n";
}

void test_jacobians()
{
    const gtsam::Pose3 bPs(gtsam::Rot3::Ypr(0.1, 0.2, -0.3), gtsam::Point3(0.1, 0.0, 0.3));
    const auto readings = make_readings(gtsam::Vector3(0.1, -0.2, 0.5), gtsam::Vector3::Zero());
    gtsam::PreintegratedAhrsMeasurements pim(make_params(), gtsam::Vector3(0.001, 0.002, -0.001));
    expect(
        mola::factors::integrate_gyro_readings(readings, 10.0, 10.8, 0.05, bPs.rotation(), pim),
        "integrated");
    const mola::factors::FactorGyroPreintegration f(
        T(0), T(1), B(0), pim, bPs.rotation(), 1e-6 * gtsam::I_3x3);

    gtsam::Values vals;
    vals.insert(
        T(0), gtsam::Pose3(gtsam::Rot3::Ypr(0.31, -0.18, 0.41), gtsam::Point3(0.1, 0.2, 0.3)));
    vals.insert(
        T(1), gtsam::Pose3(gtsam::Rot3::Ypr(0.75, -0.21, 0.38), gtsam::Point3(0.9, 0.1, 0.35)));
    vals.insert(
        B(0), gtsam::imuBias::ConstantBias(
                  gtsam::Vector3(0.05, -0.02, 0.1), gtsam::Vector3(0.003, -0.002, 0.004)));
    expect(
        gtsam::internal::testFactorJacobians("FactorGyroPreintegration", f, vals, 1e-6, 1e-5),
        "analytic Jacobians match numerical ones");
    std::cout << "[Test Passed] Jacobians\n";
}

void test_reading_gaps()
{
    auto readings = make_readings(gtsam::Vector3(0, 0, 0.1), gtsam::Vector3::Zero());
    gtsam::PreintegratedAhrsMeasurements pim(make_params(), gtsam::Vector3::Zero());
    expect(
        !mola::factors::integrate_gyro_readings(readings, 10.5, 11.5, 0.05, gtsam::Rot3(), pim),
        "fails beyond the last reading");
    readings.erase(readings.begin() + 40, readings.begin() + 50);  // a 0.11 s gap
    gtsam::PreintegratedAhrsMeasurements pim2(make_params(), gtsam::Vector3::Zero());
    expect(
        !mola::factors::integrate_gyro_readings(readings, 10.1, 10.9, 0.05, gtsam::Rot3(), pim2),
        "fails across a reading gap");
    std::cout << "[Test Passed] reading gaps\n";
}

}  // namespace

int main()
{
    try
    {
        test_zero_error_at_truth();
        test_jacobians();
        test_reading_gaps();
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << "\n";
        return 1;
    }
}
