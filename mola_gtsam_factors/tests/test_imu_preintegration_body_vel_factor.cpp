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

#include <mola_gtsam_factors/FactorImuPreintegrationBodyVel.h>

#if !GTSAM_USES_BOOST

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/factorTesting.h>

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

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::F;
using gtsam::symbol_shorthand::T;
using gtsam::symbol_shorthand::V;

constexpr double kG = 9.80665;

// One second of readings at 100 Hz for a body with a fixed attitude R moving
// at a constant world velocity: no acceleration, so the accelerometer only
// measures the reaction to gravity, R^T (0, 0, g), and the gyro reads zero.
gtsam::PreintegratedImuMeasurements make_pim(const gtsam::Rot3& R)
{
    auto p = gtsam::PreintegrationParams::MakeSharedU(kG);
    p->setGyroscopeCovariance(1e-6 * gtsam::I_3x3);
    p->setAccelerometerCovariance(1e-4 * gtsam::I_3x3);
    p->setIntegrationCovariance(1e-8 * gtsam::I_3x3);
    gtsam::PreintegratedImuMeasurements pim(p, gtsam::imuBias::ConstantBias());
    const gtsam::Vector3                acc = R.unrotate(gtsam::Vector3(0, 0, kG));
    for (int k = 0; k < 100; k++)
    {
        pim.integrateMeasurement(acc, gtsam::Vector3::Zero(), 0.01);
    }
    return pim;
}

void test_zero_error_at_truth()
{
    const gtsam::Rot3    R = gtsam::Rot3::Ypr(0.5, 0.1, -0.2);
    const gtsam::Vector3 vw(1.0, -0.5, 0.2);
    const gtsam::Pose3   Ti(R, gtsam::Point3(1, 2, 3));
    const gtsam::Pose3   Tj(R, Ti.translation() + vw * 1.0);
    const gtsam::Vector3 vb = R.unrotate(vw);

    const auto                                          pim = make_pim(R);
    const mola::factors::FactorImuPreintegrationBodyVel f(T(0), V(0), T(1), V(1), B(0), pim);
    const gtsam::Vector e = f.evaluateError(Ti, vb, Tj, vb, gtsam::imuBias::ConstantBias());
    expect(
        e.norm() < 1e-6, "zero error at the true body velocities, got " + std::to_string(e.norm()));

    // Same as the wrapped factor with world velocities:
    const gtsam::Vector e2 = f.imuFactor().evaluateError(
        Ti, vw, Tj, vw, gtsam::imuBias::ConstantBias(), nullptr, nullptr, nullptr, nullptr,
        nullptr);
    expect((e - e2).norm() < 1e-12, "matches gtsam::ImuFactor with world velocities");
    std::cout << "[Test Passed] zero error at truth, matches gtsam::ImuFactor\n";
}

void test_jacobians()
{
    const gtsam::Rot3                                   R   = gtsam::Rot3::Ypr(0.3, -0.2, 0.4);
    const auto                                          pim = make_pim(R);
    const mola::factors::FactorImuPreintegrationBodyVel f(T(0), V(0), T(1), V(1), B(0), pim);

    gtsam::Values vals;
    vals.insert(
        T(0), gtsam::Pose3(gtsam::Rot3::Ypr(0.31, -0.18, 0.41), gtsam::Point3(0.1, 0.2, 0.3)));
    vals.insert(V(0), gtsam::Point3(0.7, -0.2, 0.1));
    vals.insert(
        T(1), gtsam::Pose3(gtsam::Rot3::Ypr(0.35, -0.21, 0.38), gtsam::Point3(0.9, 0.1, 0.35)));
    vals.insert(V(1), gtsam::Point3(0.6, -0.1, 0.05));
    vals.insert(
        B(0), gtsam::imuBias::ConstantBias(
                  gtsam::Vector3(0.05, -0.02, 0.1), gtsam::Vector3(0.001, -0.002, 0.003)));
    expect(
        gtsam::internal::testFactorJacobians("FactorImuPreintegrationBodyVel", f, vals, 1e-6, 1e-5),
        "analytic Jacobians match numerical ones");
    std::cout << "[Test Passed] Jacobians\n";
}
// A {map} tilted with respect to {enu}: the factor with T_enu_to_map must see
// no error where the plain one, which assumes a level {map}, does.
void test_enu_frame()
{
    const gtsam::Rot3    R_enu = gtsam::Rot3::Ypr(0.5, 0.1, -0.2);  // body attitude in {enu}
    const gtsam::Vector3 vw_enu(1.0, -0.5, 0.2);
    const gtsam::Pose3   Ei(R_enu, gtsam::Point3(1, 2, 3));
    const gtsam::Pose3   Ej(R_enu, Ei.translation() + vw_enu * 1.0);
    const gtsam::Vector3 vb = R_enu.unrotate(vw_enu);

    const gtsam::Pose3 T_enu_to_map(gtsam::Rot3::Ypr(0.4, 0.06, -0.04), gtsam::Point3(5, -3, 1));
    const gtsam::Pose3 Ti = T_enu_to_map.inverse() * Ei;  // in {map}
    const gtsam::Pose3 Tj = T_enu_to_map.inverse() * Ej;

    const auto                                             pim = make_pim(R_enu);
    const mola::factors::FactorImuPreintegrationBodyVelEnu f(
        F(0), T(0), V(0), T(1), V(1), B(0), pim);
    const mola::factors::FactorImuPreintegrationBodyVel level(T(0), V(0), T(1), V(1), B(0), pim);
    const gtsam::imuBias::ConstantBias                  b;

    const double eEnu   = f.evaluateError(T_enu_to_map, Ti, vb, Tj, vb, b).norm();
    const double eLevel = level.evaluateError(Ti, vb, Tj, vb, b).norm();
    expect(eEnu < 1e-6, "zero error in a tilted {map}, got " + std::to_string(eEnu));
    expect(eLevel > 1e-2, "the plain factor sees the tilt, got " + std::to_string(eLevel));

    // Neither the azimuth nor the translation of {enu} change the residual:
    const gtsam::Pose3 yawed =
        gtsam::Pose3(gtsam::Rot3::Yaw(0.7), gtsam::Point3(-2, 4, 0.5)) * T_enu_to_map;
    const gtsam::Pose3 Ti2 =
        yawed.inverse() * gtsam::Pose3(gtsam::Rot3::Yaw(0.7), gtsam::Point3::Zero()) * Ei;
    const gtsam::Pose3 Tj2 =
        yawed.inverse() * gtsam::Pose3(gtsam::Rot3::Yaw(0.7), gtsam::Point3::Zero()) * Ej;
    const double eYawed = f.evaluateError(yawed, Ti, vb, Tj, vb, b).norm();
    const double eBoth  = f.evaluateError(yawed, Ti2, vb, Tj2, vb, b).norm();
    expect(
        eYawed < 1e-6, "azimuth and translation of {enu} are free, got " + std::to_string(eYawed));
    expect(eBoth < 1e-6, "consistent poses in a yawed {enu}, got " + std::to_string(eBoth));

    // With an identity T_enu_to_map it is the plain factor:
    const gtsam::Vector a = f.evaluateError(gtsam::Pose3::Identity(), Ei, vb, Ej, vb, b);
    const gtsam::Vector c = level.evaluateError(Ei, vb, Ej, vb, b);
    expect((a - c).norm() < 1e-12, "identity T_enu_to_map matches the plain factor");

    gtsam::Values vals;
    vals.insert(F(0), T_enu_to_map);
    vals.insert(
        T(0), Ti * gtsam::Pose3(gtsam::Rot3::Ypr(0.01, -0.02, 0.01), gtsam::Point3(0.1, 0, 0)));
    vals.insert(V(0), gtsam::Point3(0.7, -0.2, 0.1));
    vals.insert(
        T(1), Tj * gtsam::Pose3(gtsam::Rot3::Ypr(-0.02, 0.01, 0.03), gtsam::Point3(0, 0.1, 0)));
    vals.insert(V(1), gtsam::Point3(0.6, -0.1, 0.05));
    vals.insert(
        B(0), gtsam::imuBias::ConstantBias(
                  gtsam::Vector3(0.05, -0.02, 0.1), gtsam::Vector3(0.001, -0.002, 0.003)));
    expect(
        gtsam::internal::testFactorJacobians(
            "FactorImuPreintegrationBodyVelEnu", f, vals, 1e-6, 1e-5),
        "analytic Jacobians match numerical ones (enu)");
    std::cout << "[Test Passed] tilted {map} with T_enu_to_map, Jacobians\n";
}

}  // namespace

int main()
{
    try
    {
        test_zero_error_at_truth();
        test_jacobians();
        test_enu_frame();
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << "\n";
        return 1;
    }
}

#else
int main() { return 0; }
#endif
