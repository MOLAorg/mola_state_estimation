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
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <mola_gtsam_factors/FactorGyroPreintegrationPose.h>

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

using gtsam::symbol_shorthand::G;
using gtsam::symbol_shorthand::T;

constexpr double kDeg2Rad = M_PI / 180.0;
constexpr double GYRO_DT  = 0.01;  // [s]
constexpr double LINK_DT  = 0.1;  // [s]

std::shared_ptr<gtsam::PreintegratedAhrsMeasurements::Params> make_params()
{
    auto p = std::make_shared<gtsam::PreintegratedAhrsMeasurements::Params>();
    p->setGyroscopeCovariance(1e-6 * gtsam::I_3x3);
    // A sensor rotated on the vehicle, as in a real setup:
    p->body_P_sensor = gtsam::Pose3(gtsam::Rot3::Ypr(M_PI, 0.1, 0), gtsam::Point3(0.1, 0, 0.05));
    return p;
}

// Gyro readings of a body turning at w (vehicle frame) for one link, seen by the
// sensor, plus a constant sensor-frame bias:
gtsam::PreintegratedAhrsMeasurements integrate(
    const std::shared_ptr<gtsam::PreintegratedAhrsMeasurements::Params>& p,
    const gtsam::Vector3& wVehicle, const gtsam::Vector3& biasSensor, std::mt19937& rng,
    double noise)
{
    gtsam::PreintegratedAhrsMeasurements pim(p, gtsam::Vector3::Zero());
    const gtsam::Rot3                    Rbs = p->body_P_sensor->rotation();
    std::normal_distribution<double>     n(0.0, noise);
    for (int k = 0; k < static_cast<int>(std::lround(LINK_DT / GYRO_DT)); k++)
    {
        const gtsam::Vector3 w =
            Rbs.unrotate(wVehicle) + biasSensor + gtsam::Vector3(n(rng), n(rng), n(rng));
        pim.integrateMeasurement(w, GYRO_DT);
    }
    return pim;
}

void test_jacobians()
{
    std::mt19937 rng(1);
    const auto   p = make_params();
    const auto pim = integrate(p, gtsam::Vector3(0.1, -0.2, 0.5), gtsam::Vector3::Zero(), rng, 0.0);
    const mola::factors::FactorGyroPreintegrationPose f(T(0), T(1), G(0), pim, 1e-3);

    gtsam::Values v;
    v.insert(T(0), gtsam::Pose3(gtsam::Rot3::Ypr(0.3, -0.1, 0.2), gtsam::Point3(1, 2, 3)));
    v.insert(T(1), gtsam::Pose3(gtsam::Rot3::Ypr(0.35, -0.12, 0.19), gtsam::Point3(1.1, 2, 3)));
    v.insert(G(0), gtsam::Vector3(0.01, -0.02, 0.005));
    expect(gtsam::internal::testFactorJacobians("gyro", f, v, 1e-6, 1e-5), "numerical Jacobians");
}

// Constant turn: poses known with noise (as a LiDAR odometry would give them),
// a gyroscope with a constant 0.6 deg/s bias on every axis. The bias, starting
// from a zero prior of 0.02 rad/s, must be recovered.
void test_bias_recovery()
{
    std::mt19937         rng(2);
    const auto           p = make_params();
    const gtsam::Vector3 wVehicle(0.0, 0.0, 0.4);  // [rad/s]
    const gtsam::Vector3 trueBias = gtsam::Vector3::Constant(0.6 * kDeg2Rad);

    gtsam::NonlinearFactorGraph      graph;
    gtsam::Values                    init;
    std::normal_distribution<double> poseNoise(0.0, 0.002);  // [rad]

    const int    N = 100;  // 10 s
    gtsam::Pose3 gt;
    graph.addPrior(
        G(0), gtsam::Vector3::Zero().eval(), gtsam::noiseModel::Isotropic::Sigma(3, 0.02));
    for (int i = 0; i <= N; i++)
    {
        const auto noisy = gt.compose(gtsam::Pose3(
            gtsam::Rot3::RzRyRx(poseNoise(rng), poseNoise(rng), poseNoise(rng)),
            gtsam::Point3::Zero()));
        graph.addPrior(T(i), noisy, gtsam::noiseModel::Isotropic::Sigma(6, 0.002));
        init.insert(T(i), noisy);
        init.insert(G(i), gtsam::Vector3::Zero().eval());
        if (i < N)
        {
            const auto pim = integrate(p, wVehicle, trueBias, rng, 0.002);
            graph.emplace_shared<mola::factors::FactorGyroPreintegrationPose>(
                T(i), T(i + 1), G(i), pim, 1e-3);
            graph.emplace_shared<gtsam::BetweenFactor<gtsam::Vector3>>(
                G(i), G(i + 1), gtsam::Vector3::Zero().eval(),
                gtsam::noiseModel::Isotropic::Sigma(3, 1e-5));
            gt = gt.compose(
                gtsam::Pose3(gtsam::Rot3::Expmap(wVehicle * LINK_DT), gtsam::Point3(0.1, 0, 0)));
        }
    }
    const auto r   = gtsam::LevenbergMarquardtOptimizer(graph, init).optimize();
    const auto est = r.at<gtsam::Vector3>(G(N / 2));
    std::cout << "gyro bias: true " << trueBias.transpose() / kDeg2Rad << " deg/s, estimated "
              << est.transpose() / kDeg2Rad << " deg/s\n";
    expect((est - trueBias).norm() < 0.05 * kDeg2Rad, "gyro bias recovered to 0.05 deg/s");
}
}  // namespace

int main()
{
    try
    {
        test_jacobians();
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
