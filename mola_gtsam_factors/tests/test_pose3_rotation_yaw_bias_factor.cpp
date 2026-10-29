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
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/expressionTesting.h>
#include <gtsam/slam/PriorFactor.h>
#include <mola_gtsam_factors/Pose3RotationYawBiasFactor.h>

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

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::F;
using gtsam::symbol_shorthand::T;

const gtsam::Pose3 kEnuToMap(
    gtsam::Rot3::Ypr(5 * kDeg2Rad, 1 * kDeg2Rad, -2 * kDeg2Rad), gtsam::Point3(1, 2, 3));
const gtsam::Pose3 kVehicle(
    gtsam::Rot3::Ypr(40 * kDeg2Rad, 3 * kDeg2Rad, -1 * kDeg2Rad), gtsam::Point3(-4, 5, 0.5));
const gtsam::Pose3 kSensorOnVehicle(
    gtsam::Rot3::Ypr(90 * kDeg2Rad, 0, 180 * kDeg2Rad), gtsam::Point3(0.1, 0, 0.3));

gtsam::Rot3 observed_orientation(const double bias)
{
    return gtsam::Rot3::Rz(bias) * (kEnuToMap * kVehicle * kSensorOnVehicle).rotation();
}

void test_zero_error_and_jacobians()
{
    const double bias = 12 * kDeg2Rad;

    const mola::factors::Pose3RotationYawBiasFactor factor(
        F(0), T(0), B(0), kSensorOnVehicle, observed_orientation(bias),
        gtsam::noiseModel::Isotropic::Sigma(3, 2 * kDeg2Rad));

    gtsam::Values values;
    values.insert(F(0), kEnuToMap);
    values.insert(T(0), kVehicle);
    values.insert(B(0), bias);

    expect(factor.unwhitenedError(values).norm() < 1e-9, "zero error at the true values");

    // Jacobians of the measurement expression, away from the solution so every
    // term is exercised (the factor's own ones are only exact at zero error):
    gtsam::Values other;
    other.insert(F(0), kEnuToMap.retract((gtsam::Vector6() << 0.1, -0.2, 0.3, 1, 2, 3).finished()));
    other.insert(T(0), kVehicle.retract((gtsam::Vector6() << -0.3, 0.1, 0.2, 3, 2, 1).finished()));
    other.insert(B(0), -0.4);
    expect(
        gtsam::internal::testExpressionJacobians(
            "Pose3RotationYawBiasFactor", factor.expression({F(0), T(0), B(0)}), other, 1e-6, 1e-5),
        "analytical Jacobians match numerical ones");
}

// The bias is observable once the vehicle heading is known (e.g. from LiDAR
// localization), and the heading follows the biased readings when it is not.
void test_bias_recovery()
{
    const double trueBias = -25 * kDeg2Rad;

    gtsam::NonlinearFactorGraph graph;
    graph.addPrior(F(0), kEnuToMap, gtsam::noiseModel::Isotropic::Sigma(6, 1e-6));
    graph.addPrior(B(0), 0.0, gtsam::noiseModel::Isotropic::Sigma(1, 30 * kDeg2Rad));
    // Vehicle pose known to 0.5 deg:
    graph.addPrior(T(0), kVehicle, gtsam::noiseModel::Isotropic::Sigma(6, 0.5 * kDeg2Rad));
    graph.emplace_shared<mola::factors::Pose3RotationYawBiasFactor>(
        F(0), T(0), B(0), kSensorOnVehicle, observed_orientation(trueBias),
        gtsam::noiseModel::Isotropic::Sigma(3, 1 * kDeg2Rad));

    gtsam::Values init;
    init.insert(F(0), kEnuToMap);
    init.insert(T(0), kVehicle);
    init.insert(B(0), 0.0);

    const auto result = gtsam::LevenbergMarquardtOptimizer(graph, init).optimize();

    const double estBias = result.at<double>(B(0));
    std::cout << "Estimated bias: " << estBias / kDeg2Rad << " deg (true " << trueBias / kDeg2Rad
              << " deg)\n";
    expect(std::abs(estBias - trueBias) < 1 * kDeg2Rad, "bias recovered within 1 deg");

    const double yawErr =
        gtsam::Rot3::Logmap(kVehicle.rotation().between(result.at<gtsam::Pose3>(T(0)).rotation()))
            .norm();
    expect(yawErr < 1 * kDeg2Rad, "vehicle orientation not pulled by the biased reading");
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
        std::cerr << "Test failed: " << e.what() << "\n";
        return 1;
    }
}
