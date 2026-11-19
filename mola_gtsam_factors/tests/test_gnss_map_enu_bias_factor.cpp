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
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/expressionTesting.h>
#include <mola_gtsam_factors/FactorGnssMapEnu.h>
#include <mola_gtsam_factors/FactorGnssMapEnuBias.h>
#include <mola_gtsam_factors/FactorGnssMapEnuConstVel.h>

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

constexpr double kDeg2Rad = M_PI / 180.0;

using gtsam::symbol_shorthand::F;
using gtsam::symbol_shorthand::G;
using gtsam::symbol_shorthand::T;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::W;

const gtsam::Pose3 kEnuToMap(
    gtsam::Rot3::Ypr(25 * kDeg2Rad, 1 * kDeg2Rad, -2 * kDeg2Rad), gtsam::Point3(1, 2, 3));
const gtsam::Pose3 kVehicle(
    gtsam::Rot3::Ypr(40 * kDeg2Rad, 3 * kDeg2Rad, -1 * kDeg2Rad), gtsam::Point3(-4, 5, 0.5));
const gtsam::Point3 kAntenna(0.3, -0.1, 0.9);
const gtsam::Point3 kVel(1.5, 0.2, -0.1);  // body frame [m/s]
const gtsam::Point3 kAngVel(0.05, -0.1, 0.8);  // body frame [rad/s]

gtsam::Values values_with_bias(const gtsam::Point3& bias)
{
    gtsam::Values vals;
    vals.insert(F(0), kEnuToMap);
    vals.insert(T(0), kVehicle);
    vals.insert(V(0), kVel);
    vals.insert(W(0), kAngVel);
    vals.insert(G(0), bias);
    return vals;
}

void test_zero_bias_matches_plain_factors()
{
    const auto noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.05);
    const auto meas  = gtsam::Point3(10, 20, 30);
    const auto vals  = values_with_bias(gtsam::Point3(0, 0, 0));

    for (const double dt : {0.0, -0.05, 0.1})
    {
        const mola::factors::FactorGnssMapEnuBias biased(
            F(0), T(0), V(0), W(0), G(0), kAntenna, dt, meas, noise);
        const mola::factors::FactorGnssMapEnuConstVel constVel(
            F(0), T(0), V(0), W(0), kAntenna, dt, meas, noise);
        expect(
            (biased.unwhitenedError(vals) - constVel.unwhitenedError(vals)).norm() < 1e-12,
            "zero bias reduces to FactorGnssMapEnuConstVel, dt=" + std::to_string(dt));
    }

    const mola::factors::FactorGnssMapEnuBias biased(
        F(0), T(0), V(0), W(0), G(0), kAntenna, 0.0, meas, noise);
    const mola::factors::FactorGnssMapEnu plain(F(0), T(0), kAntenna, meas, noise);
    expect(
        (biased.unwhitenedError(vals) - plain.unwhitenedError(vals)).norm() < 1e-12,
        "zero bias and dt=0 reduce to FactorGnssMapEnu");
}

void test_bias_is_additive()
{
    const auto          noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.05);
    const auto          meas  = gtsam::Point3(10, 20, 30);
    const gtsam::Point3 bias(1.5, -2.0, 0.7);

    const mola::factors::FactorGnssMapEnuBias factor(
        F(0), T(0), V(0), W(0), G(0), kAntenna, 0.04, meas, noise);

    const auto e0 = factor.unwhitenedError(values_with_bias(gtsam::Point3(0, 0, 0)));
    const auto e1 = factor.unwhitenedError(values_with_bias(bias));
    expect((e1 - e0 - bias).norm() < 1e-12, "the bias shifts the predicted antenna position");
}

void test_jacobians()
{
    for (const double dt : {0.0, 0.07})
    {
        const mola::factors::FactorGnssMapEnuBias factor(
            F(0), T(0), V(0), W(0), G(0), kAntenna, dt, gtsam::Point3(1, 2, 3),
            gtsam::noiseModel::Isotropic::Sigma(3, 0.05));

        gtsam::Values other;
        other.insert(
            F(0), kEnuToMap.retract((gtsam::Vector6() << 0.1, -0.2, 0.3, 1, 2, 3).finished()));
        other.insert(
            T(0), kVehicle.retract((gtsam::Vector6() << -0.3, 0.1, 0.2, 3, 2, 1).finished()));
        other.insert(V(0), gtsam::Point3(-0.7, 1.1, 0.3));
        other.insert(W(0), gtsam::Point3(0.4, -0.2, 1.3));
        other.insert(G(0), gtsam::Point3(0.8, -1.2, 0.4));

        expect(
            gtsam::internal::testExpressionJacobians(
                "FactorGnssMapEnuBias", factor.expression({F(0), T(0), V(0), W(0), G(0)}), other,
                1e-6, 1e-5),
            "analytical Jacobians match numerical ones, dt=" + std::to_string(dt));
    }
}

}  // namespace

int main()
{
    try
    {
        test_zero_bias_matches_plain_factors();
        test_bias_is_additive();
        test_jacobians();
        std::cout << "All tests passed.\n";
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed: " << e.what() << "\n";
        return 1;
    }
}
