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

gtsam::Values values_at(const gtsam::Point3& v, const gtsam::Point3& w)
{
    gtsam::Values vals;
    vals.insert(F(0), kEnuToMap);
    vals.insert(T(0), kVehicle);
    vals.insert(V(0), v);
    vals.insert(W(0), w);
    return vals;
}

gtsam::Point3 antenna_enu_at(const double dt)
{
    // Ground truth: the exact constant-twist motion over dt.
    gtsam::Vector6 xi;
    xi << kAngVel * dt, kVel * dt;
    const gtsam::Pose3 moved = kVehicle * gtsam::Pose3::Expmap(xi);
    return (kEnuToMap * moved).transformFrom(kAntenna);
}

void test_zero_dt_matches_plain_factor()
{
    const auto noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.05);
    const auto meas  = gtsam::Point3(10, 20, 30);

    const mola::factors::FactorGnssMapEnu         plain(F(0), T(0), kAntenna, meas, noise);
    const mola::factors::FactorGnssMapEnuConstVel withDt(
        F(0), T(0), V(0), W(0), kAntenna, 0.0, meas, noise);

    const auto vals = values_at(kVel, kAngVel);
    expect(
        (plain.unwhitenedError(vals) - withDt.unwhitenedError(vals)).norm() < 1e-12,
        "dt=0 reduces to FactorGnssMapEnu");
}

void test_motion_compensation()
{
    const auto noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.05);
    for (const double dt : {-0.05, 0.03, 0.1})
    {
        const auto meas = antenna_enu_at(dt);

        const mola::factors::FactorGnssMapEnuConstVel withDt(
            F(0), T(0), V(0), W(0), kAntenna, dt, meas, noise);
        const mola::factors::FactorGnssMapEnu plain(F(0), T(0), kAntenna, meas, noise);

        const auto   vals     = values_at(kVel, kAngVel);
        const double errDt    = withDt.unwhitenedError(vals).norm();
        const double errPlain = plain.unwhitenedError(vals).norm();

        // First order in dt: the residual is O(dt^2 |w| (|v| + |w||l|)).
        expect(errDt < 0.015, "compensated residual is second order, dt=" + std::to_string(dt));
        expect(errPlain > 5 * errDt, "plain factor sees the motion, dt=" + std::to_string(dt));
    }
}

void test_jacobians()
{
    const mola::factors::FactorGnssMapEnuConstVel factor(
        F(0), T(0), V(0), W(0), kAntenna, 0.07, gtsam::Point3(1, 2, 3),
        gtsam::noiseModel::Isotropic::Sigma(3, 0.05));

    gtsam::Values other;
    other.insert(F(0), kEnuToMap.retract((gtsam::Vector6() << 0.1, -0.2, 0.3, 1, 2, 3).finished()));
    other.insert(T(0), kVehicle.retract((gtsam::Vector6() << -0.3, 0.1, 0.2, 3, 2, 1).finished()));
    other.insert(V(0), gtsam::Point3(-0.7, 1.1, 0.3));
    other.insert(W(0), gtsam::Point3(0.4, -0.2, 1.3));

    expect(
        gtsam::internal::testExpressionJacobians(
            "FactorGnssMapEnuConstVel", factor.expression({F(0), T(0), V(0), W(0)}), other, 1e-6,
            1e-5),
        "analytical Jacobians match numerical ones");
}

}  // namespace

int main()
{
    try
    {
        test_zero_dt_matches_plain_factor();
        test_motion_compensation();
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
