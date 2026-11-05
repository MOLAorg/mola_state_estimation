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

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/expressionTesting.h>
#include <gtsam/slam/PriorFactor.h>
#include <mola_gtsam_factors/MeasuredGravityTiltBiasFactor.h>

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

const gtsam::Pose3 kEnuToMap(gtsam::Rot3::Ypr(5 * kDeg2Rad, 0, 0), gtsam::Point3(1, 2, 3));
const gtsam::Pose3 kSensorOnVehicle(
    gtsam::Rot3::Ypr(90 * kDeg2Rad, 0, 180 * kDeg2Rad), gtsam::Point3(0.1, 0, 0.3));

gtsam::Pose3 vehicle_at(double yawDeg)
{
    return gtsam::Pose3(
        gtsam::Rot3::Ypr(yawDeg * kDeg2Rad, 3 * kDeg2Rad, -1 * kDeg2Rad),
        gtsam::Point3(-4, 5, 0.5));
}

// What a sensor tilted by b off its nominal mounting reads as "up":
gtsam::Vector3 observed_up(const gtsam::Pose3& vehicle, const gtsam::Vector2& b)
{
    const gtsam::Rot3 R    = (kEnuToMap * vehicle * kSensorOnVehicle).rotation();
    const gtsam::Rot3 tilt = gtsam::Rot3::Expmap(gtsam::Vector3(b.x(), b.y(), 0));
    return (R * tilt).unrotate(gtsam::Vector3(0, 0, 1));
}

auto noise() { return gtsam::noiseModel::Isotropic::Sigma(3, 2 * kDeg2Rad); }

void test_zero_error_and_jacobians()
{
    const gtsam::Vector2 b(2 * kDeg2Rad, -1.5 * kDeg2Rad);
    const auto           vehicle = vehicle_at(40);

    const mola::factors::MeasuredGravityTiltBiasFactor factor(
        F(0), T(0), G(0), kSensorOnVehicle, observed_up(vehicle, b), noise());

    gtsam::Values values;
    values.insert(F(0), kEnuToMap);
    values.insert(T(0), vehicle);
    values.insert(G(0), b);
    expect(factor.unwhitenedError(values).norm() < 1e-9, "zero error at the true values");

    gtsam::Values other;
    other.insert(F(0), kEnuToMap.retract((gtsam::Vector6() << 0.1, -0.2, 0.3, 1, 2, 3).finished()));
    other.insert(T(0), vehicle.retract((gtsam::Vector6() << -0.3, 0.1, 0.2, 3, 2, 1).finished()));
    other.insert(G(0), gtsam::Vector2(0.2, -0.1));
    expect(
        gtsam::internal::testExpressionJacobians(
            "MeasuredGravityTiltBiasFactor", factor.expression({F(0), T(0), G(0)}), other, 1e-6,
            1e-5),
        "analytical Jacobians match numerical ones");
}

// A mounting tilt is recovered once the vehicle attitude is known and the
// vehicle turns, and it then no longer pulls the attitude.
void test_bias_recovery()
{
    const gtsam::Vector2 trueBias(3 * kDeg2Rad, -2 * kDeg2Rad);

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values               init;
    graph.addPrior(F(0), kEnuToMap, gtsam::noiseModel::Isotropic::Sigma(6, 1e-6));
    graph.addPrior(
        G(0), gtsam::Vector2(0, 0), gtsam::noiseModel::Isotropic::Sigma(2, 5 * kDeg2Rad));
    init.insert(F(0), kEnuToMap);
    init.insert(G(0), gtsam::Vector2(0, 0));

    for (int i = 0; i < 8; i++)
    {
        const auto vehicle = vehicle_at(45.0 * i);
        graph.addPrior(T(i), vehicle, gtsam::noiseModel::Isotropic::Sigma(6, 0.1 * kDeg2Rad));
        graph.emplace_shared<mola::factors::MeasuredGravityTiltBiasFactor>(
            F(0), T(i), G(0), kSensorOnVehicle, observed_up(vehicle, trueBias), noise());
        init.insert(T(i), vehicle);
    }

    const auto result = gtsam::LevenbergMarquardtOptimizer(graph, init).optimize();
    const auto est    = result.at<gtsam::Vector2>(G(0));
    std::cout << "tilt bias: true " << trueBias.transpose() / kDeg2Rad << " deg, estimated "
              << est.transpose() / kDeg2Rad << " deg\n";
    expect((est - trueBias).norm() < 0.2 * kDeg2Rad, "tilt bias recovered");
}
}  // namespace

int main()
{
    try
    {
        test_zero_error_and_jacobians();
        test_bias_recovery();
        std::cout << "All tests passed.\n";
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << "\n";
        return 1;
    }
}
