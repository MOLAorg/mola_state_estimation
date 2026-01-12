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
 * @file   FactorTricycleKinematic.cpp
 * @brief  GTSAM factor for tricycle kinematic model
 * @author Jose Luis Blanco Claraco
 * @date   Jan 12, 2026
 */

#include <mola_gtsam_factors/FactorTricycleKinematic.h>

namespace mola::factors
{

FactorTricycleKinematic::FactorTricycleKinematic()
    : Base(gtsam::noiseModel::Isotropic::Sigma(6, 1.0), 0, 0, 0, 0)
{
}

FactorTricycleKinematic::FactorTricycleKinematic(
    gtsam::Key kTi, gtsam::Key kVi, gtsam::Key kWi, gtsam::Key kTj, const double dt,
    const gtsam::SharedNoiseModel& model, const double w_threshold,
    const bool approximateDerivatives)
    : Base(model, kTi, kVi, kWi, kTj),
      dt_(dt),
      w_threshold_(w_threshold),
      approximateDerivatives_(approximateDerivatives)
{
}

gtsam::Vector FactorTricycleKinematic::evaluateError(
    const gtsam::Pose3& Ti, const gtsam::Point3& bVi, const gtsam::Point3& bWi,
    const gtsam::Pose3& Tj,
#if GTSAM_USES_BOOST
    boost::optional<gtsam::Matrix&> H1, boost::optional<gtsam::Matrix&> H2,
    boost::optional<gtsam::Matrix&> H3, boost::optional<gtsam::Matrix&> H4
#else
    boost::optional<gtsam::Matrix&> H1, boost::optional<gtsam::Matrix&> H2,
    boost::optional<gtsam::Matrix&> H3, boost::optional<gtsam::Matrix&> H4
#endif
) const
{
    const double v     = bVi.x();  // linear velocity
    const double w     = bWi.z();  // angular velocity
    const double theta = w * dt_;

    gtsam::Pose3 delta_pose;
    // Local Jacobians for delta_pose w.r.t v and w
    gtsam::Vector6 J_v = gtsam::Vector6::Zero();
    gtsam::Vector6 J_w = gtsam::Vector6::Zero();

    if (std::abs(w) < w_threshold_)
    {
        // Straight line motion
        delta_pose = gtsam::Pose3(gtsam::Rot3::Identity(), gtsam::Point3(v * dt_, 0, 0));

        // Derivatives for small w (Taylor expansions)
        J_v << 0, 0, 0, dt_, 0, 0;
        J_w << 0, 0, dt_, 0, 0.5 * v * dt_ * dt_, 0;
    }
    else
    {
        // Circular arc motion
        const double R  = v / w;
        const double s  = std::sin(theta);
        const double c  = std::cos(theta);
        const double dx = R * s;
        const double dy = R * (1.0 - c);
        delta_pose      = gtsam::Pose3(gtsam::Rot3::Rz(theta), gtsam::Point3(dx, dy, 0.0));

        // Analytical derivatives in delta_pose's local tangent space
        // J_v = [0, 0, 0, sin(theta)/w, (cos(theta)-1)/w, 0]^T
        J_v << 0, 0, 0, s / w, (c - 1.0) / w, 0;

        // J_w = [0, 0, dt, v*(theta - sin(theta))/w^2, 0, 0]^T
        J_w << 0, 0, dt_, v * (theta - s) / (w * w), 0, 0;
    }

    // 1. Prediction: T_pred = Ti * delta_pose
    gtsam::Matrix66    H_comp_Ti, H_comp_delta;
    const gtsam::Pose3 Tj_pred = Ti.compose(delta_pose, H_comp_Ti, H_comp_delta);

    // 2. Error: Tj_pred.between(Tj)
    gtsam::Matrix66      H_btw_pred, H_btw_actual;
    const gtsam::Pose3   error_pose = Tj_pred.between(Tj, H_btw_pred, H_btw_actual);
    const gtsam::Vector6 error      = gtsam::Pose3::Logmap(error_pose);

    // 3. Chain Rule for Jacobians
    if (H1 || H2 || H3 || H4)
    {
        // Logmap Jacobian (De_Log)
        gtsam::Matrix66 De_Log = gtsam::Pose3::LogmapDerivative(error_pose);

        if (H1)
        {  // Jacobian w.r.t Ti
            *H1 = De_Log * H_btw_pred * H_comp_Ti;
        }

        if (H2)  // Jacobian w.r.t bVi (Point3)
        {
            *H2        = gtsam::Matrix::Zero(6, 3);
            H2->col(0) = De_Log * H_btw_pred * H_comp_delta * J_v;
        }

        if (H3)  // Jacobian w.r.t bWi (Point3)
        {
            *H3        = gtsam::Matrix::Zero(6, 3);
            H3->col(2) = De_Log * H_btw_pred * H_comp_delta * J_w;
        }

        if (H4)
        {  // Jacobian w.r.t Tj
            *H4 = De_Log * H_btw_actual;
        }
    }

    return error;
}

}  // namespace mola::factors