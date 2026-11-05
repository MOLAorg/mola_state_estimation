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
 * @file   MeasuredGravityTiltBiasFactor.h
 * @brief  Gravity (normalized accelerometer) observation with a tilt bias state
 * @author Jose Luis Blanco Claraco
 * @date   Nov 5, 2026
 */

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/expressions.h>
#include <mola_gtsam_factors/gtsam_detect_version.h>

namespace mola::factors
{
/**
 * Like MeasuredGravityFactor, but the measured up direction is first rotated
 * by a small tilt b = (bx, by) [rad] about the sensor x and y axes, a state
 * shared by consecutive readings: it absorbs what the sensor reports
 * consistently off the true vertical (mounting error, accelerometer bias),
 * which an absolute factor would otherwise impose on the vehicle attitude.
 *
 * Error: rotate(R(T_enu2map * Pi * sensorPose) * Exp([bx, by, 0]), up_s) - (0,0,1).
 */
class MeasuredGravityTiltBiasFactor
    : public gtsam::ExpressionFactorN<
          gtsam::Vector3 /*return type*/, gtsam::Pose3 /*T_enu2map*/, gtsam::Pose3 /*Pi*/,
          gtsam::Vector2 /*tilt bias*/>
{
   private:
    using This = MeasuredGravityTiltBiasFactor;
    using Base =
        gtsam::ExpressionFactorN<gtsam::Vector3, gtsam::Pose3, gtsam::Pose3, gtsam::Vector2>;

    gtsam::Pose3  sensorPoseOnVehicle_ = gtsam::Pose3::Identity();
    gtsam::Point3 observedNormalizedAcc_b_;

   public:
    MeasuredGravityTiltBiasFactor();

    MeasuredGravityTiltBiasFactor(
        gtsam::Key kT_enu2map, gtsam::Key kPi, gtsam::Key kTiltBias,
        const gtsam::Pose3& sensorPoseOnVehicle, const gtsam::Vector3& observedNormalizedAcc_b,
        const gtsam::SharedNoiseModel& model);

    gtsam::NonlinearFactor::shared_ptr clone() const override;

    gtsam::Expression<gtsam::Point3> expression(
        const std::array<gtsam::Key, NARY_EXPRESSION_SIZE>& keys) const override;

    void print(
        const std::string&         s,
        const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override;

    bool equals(const gtsam::NonlinearFactor& expected, double tol = 1e-9) const override;
};

}  // namespace mola::factors
