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
#include <mola_gtsam_factors/FactorTricycleKinematic.h>

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

using gtsam::symbol_shorthand::T;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::W;

// Analytic Jacobians vs numerical ones, in the circular-arc branch, at large
// turn angles and with a non-zero residual.
void test_jacobians_arc()
{
    const auto noise = gtsam::noiseModel::Isotropic::Sigma(6, 1.0);
    for (const double w : {0.5, -1.0, 3.0})
    {
        for (const double dt : {0.1, 0.5, 1.0})
        {
            const mola::factors::FactorTricycleKinematic f(T(0), V(0), W(0), T(1), dt, noise);

            const gtsam::Pose3  Ti(gtsam::Rot3::Ypr(2.5, 0.3, -0.4), gtsam::Point3(1, 2, 3));
            const gtsam::Point3 bV(2.0, 0.1, -0.2);
            const gtsam::Point3 bW(0.05, -0.03, w);
            const gtsam::Pose3  Tj =
                mola::factors::FactorTricycleKinematic::tricycleKinematicModel(
                    Ti, bV, bW, dt, 1e-4) *
                gtsam::Pose3(gtsam::Rot3::Ypr(0.02, -0.01, 0.03), gtsam::Point3(0.03, -0.02, 0.01));

            gtsam::Values vals;
            vals.insert(T(0), Ti);
            vals.insert(V(0), bV);
            vals.insert(W(0), bW);
            vals.insert(T(1), Tj);
            expect(
                gtsam::internal::testFactorJacobians(
                    "FactorTricycleKinematic", f, vals, 1e-6, 1e-6),
                "analytic Jacobians match numerical ones for w=" + std::to_string(w) +
                    " dt=" + std::to_string(dt));
        }
    }
    std::cout << "[Test Passed] Jacobians (arc branch)\n";
}
}  // namespace

int main()
{
    try
    {
        test_jacobians_arc();
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << "\n";
        return 1;
    }
}
