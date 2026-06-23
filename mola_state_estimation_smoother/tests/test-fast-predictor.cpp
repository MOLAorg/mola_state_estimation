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
 * @file   test-fast-predictor.cpp
 * @brief  Unit tests for the async-mode FastPredictor (tiny re-anchored graph).
 * @author Jose Luis Blanco Claraco
 * @date   Jun 22, 2026
 */

#include <mrpt/core/exceptions.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>

#include <cmath>
#include <iostream>
#include <utility>
#include <vector>

#include "../src/FastPredictor.h"  // internal header (linked from the package lib)

using namespace mola::state_estimation_smoother;
using mola::NavState;

namespace
{
mrpt::math::CMatrixDouble66 scaledIdentity66(double s)
{
    mrpt::math::CMatrixDouble66 m;
    m.setZero();
    for (int i = 0; i < 6; i++)
    {
        m(i, i) = s;
    }
    return m;
}

// Builds a tight-covariance anchor NavState at the given pose + twist.
NavState makeAnchor(const mrpt::poses::CPose3D& pose, const mrpt::math::TTwist3D& twist)
{
    NavState ns;
    ns.pose.mean     = pose;
    ns.pose.cov_inv  = scaledIdentity66(1e4);  // sigma ~0.01 m/rad
    ns.twist         = twist;
    ns.twist_inv_cov = scaledIdentity66(1e2);  // sigma ~0.1 m/s
    return ns;
}

mrpt::obs::CObservationOdometry::Ptr makeOdom(double t, double x, double y, double phi)
{
    auto o       = mrpt::obs::CObservationOdometry::Create();
    o->timestamp = mrpt::Clock::fromDouble(t);
    o->odometry  = mrpt::poses::CPose2D(x, y, phi);
    return o;
}

Parameters defaultParams()
{
    Parameters p;
    p.reference_frame_name = "map";
    p.kinematic_model      = KinematicModel::ConstantVelocity;
    // defaults are fine for the rest (max_time_to_use_velocity_model=2.0, etc.)
    return p;
}

// 1) No anchor -> nullopt.
void test_no_anchor()
{
    FastPredictor fp;
    const auto    params = defaultParams();
    ASSERT_(!fp.has_anchor());
    const auto r = fp.predict(mrpt::Clock::fromDouble(10.0), params);
    ASSERT_(!r.has_value());
}

// 2) Pure constant-velocity extrapolation from the anchor.
void test_const_vel_extrapolation()
{
    FastPredictor fp;
    const auto    params = defaultParams();

    const double t0 = 1000.0;
    const double v  = 2.0;  // m/s along +x
    fp.set_anchor(
        makeAnchor(mrpt::poses::CPose3D::Identity(), mrpt::math::TTwist3D(v, 0, 0, 0, 0, 0)),
        mrpt::Clock::fromDouble(t0), {});

    const double dt = 0.1;
    const auto   r  = fp.predict(mrpt::Clock::fromDouble(t0 + dt), params);
    ASSERT_(r.has_value());

    const double expected_x = v * dt;  // 0.2 m
    std::cout << "[const-vel] predicted x=" << r->pose.mean.x() << " expected~" << expected_x
              << "\n";
    ASSERT_LT_(std::abs(r->pose.mean.x() - expected_x), 0.05);
    ASSERT_LT_(std::abs(r->pose.mean.y()), 0.02);
    // velocity should be preserved:
    ASSERT_LT_(std::abs(r->twist.vx - v), 0.1);
}

// 3) Query too far in the future -> nullopt.
void test_too_far()
{
    FastPredictor fp;
    const auto    params = defaultParams();  // max_time_to_use_velocity_model=2.0

    const double t0 = 0.0;
    fp.set_anchor(
        makeAnchor(mrpt::poses::CPose3D::Identity(), mrpt::math::TTwist3D(1, 0, 0, 0, 0, 0)),
        mrpt::Clock::fromDouble(t0), {});

    const auto r = fp.predict(mrpt::Clock::fromDouble(t0 + 5.0), params);
    ASSERT_(!r.has_value());
}

// 4) Backward (in the past) query is still valid within the window.
void test_backward_query()
{
    FastPredictor fp;
    const auto    params = defaultParams();

    const double t0 = 500.0;
    const double v  = 2.0;
    fp.set_anchor(
        makeAnchor(mrpt::poses::CPose3D::Identity(), mrpt::math::TTwist3D(v, 0, 0, 0, 0, 0)),
        mrpt::Clock::fromDouble(t0), {});

    const auto r = fp.predict(mrpt::Clock::fromDouble(t0 - 0.1), params);
    ASSERT_(r.has_value());
    std::cout << "[backward] predicted x=" << r->pose.mean.x() << " expected~-0.2\n";
    ASSERT_LT_(std::abs(r->pose.mean.x() - (-0.2)), 0.05);
}

// 5) Buffered wheel-odometry pulls the prediction even when the anchor twist is zero.
void test_odom_influence()
{
    FastPredictor fp;
    const auto    params = defaultParams();

    const double t0 = 10.0;
    fp.set_anchor(
        makeAnchor(mrpt::poses::CPose3D::Identity(), mrpt::math::TTwist3D(0, 0, 0, 0, 0, 0)),
        mrpt::Clock::fromDouble(t0), {});

    // 0.3 m forward over 0.1 s:
    fp.push_observation(makeOdom(t0, 0.0, 0.0, 0.0), /*bufferLength*/ 1.0);
    fp.push_observation(makeOdom(t0 + 0.1, 0.3, 0.0, 0.0), 1.0);

    const auto r = fp.predict(mrpt::Clock::fromDouble(t0 + 0.1), params);
    ASSERT_(r.has_value());
    std::cout << "[odom] anchor twist=0, predicted x=" << r->pose.mean.x()
              << " (odom increment=0.3)\n";
    // Odometry must pull the prediction forward (well away from the zero-twist
    // const-vel result of ~0):
    ASSERT_GT_(r->pose.mean.x(), 0.1);
}

// 6) Cached frame_transform getter.
void test_frame_transform()
{
    FastPredictor                  fp;
    FastPredictor::FrameTransforms tf;
    tf["odom"] = mrpt::poses::CPose3DPDFGaussian(
        mrpt::poses::CPose3D(1.0, 2.0, 0.0, 0.0, 0.0, 0.0), scaledIdentity66(1e-4));

    fp.set_anchor(
        makeAnchor(mrpt::poses::CPose3D::Identity(), mrpt::math::TTwist3D()),
        mrpt::Clock::fromDouble(0.0), tf);

    ASSERT_(fp.frame_transform("odom").has_value());
    ASSERT_LT_(std::abs(fp.frame_transform("odom")->mean.x() - 1.0), 1e-9);
    ASSERT_(!fp.frame_transform("nonexistent").has_value());

    // clear() drops the anchor + transforms:
    fp.clear();
    ASSERT_(!fp.has_anchor());
    ASSERT_(!fp.frame_transform("odom").has_value());
}

}  // namespace

int main()
{
    int                                                   numErrors = 0;
    const std::vector<std::pair<const char*, void (*)()>> tests     = {
            {"no_anchor", &test_no_anchor},
            {"const_vel_extrapolation", &test_const_vel_extrapolation},
            {"too_far", &test_too_far},
            {"backward_query", &test_backward_query},
            {"odom_influence", &test_odom_influence},
            {"frame_transform", &test_frame_transform},
    };

    for (const auto& [name, fn] : tests)
    {
        try
        {
            fn();
            std::cout << "✅ " << name << "\n";
        }
        catch (const std::exception& e)
        {
            std::cerr << "❌ " << name << ": " << e.what() << "\n";
            numErrors++;
        }
    }

    std::cout << "\nRESULT: " << (tests.size() - numErrors) << " passed, " << numErrors
              << " failed.\n";
    return numErrors == 0 ? 0 : 1;
}
