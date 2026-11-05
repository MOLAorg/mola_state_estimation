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
 * @file   FactorGyroPreintegrationPose.h
 * @brief  Preintegrated gyroscope rotation between two Pose3 keyframes, with a
 *         gyroscope bias state (gtsam::AHRSFactor on Pose3 keys)
 * @author Jose Luis Blanco Claraco
 * @date   Nov 5, 2026
 */

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/AHRSFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace mola::factors
{
/**
 * The rotation part of two keyframe poses against the gyroscope readings
 * integrated between them, corrected to first order for the gyroscope bias at
 * the first keyframe:
 *
 *   e = Log( DeltaR_ij(b)^-1 * R_i^-1 * R_j )
 *
 * as gtsam::AHRSFactor (whose keys are Rot3), here on Pose3 keys. The noise
 * model is the preintegrated covariance plus a diagonal floor.
 */
class FactorGyroPreintegrationPose
    : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3, gtsam::Vector3>
{
   private:
    using This = FactorGyroPreintegrationPose;
    using Base = gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3, gtsam::Vector3>;

    gtsam::AHRSFactor ahrs_;

   public:
    FactorGyroPreintegrationPose() = default;

    /// \param rotationFloorSigma added (in quadrature) to the preintegrated
    /// rotation sigmas [rad].
    FactorGyroPreintegrationPose(
        gtsam::Key kTi, gtsam::Key kTj, gtsam::Key kBias,
        const gtsam::PreintegratedAhrsMeasurements& pim, double rotationFloorSigma);

    gtsam::NonlinearFactor::shared_ptr clone() const override;

    gtsam::Vector evaluateError(
        const gtsam::Pose3& Ti, const gtsam::Pose3& Tj, const gtsam::Vector3& bias,
        gtsam::OptionalMatrixType H1, gtsam::OptionalMatrixType H2,
        gtsam::OptionalMatrixType H3) const override;

    const gtsam::PreintegratedAhrsMeasurements& preintegrated() const
    {
        return ahrs_.preintegratedMeasurements();
    }
};

}  // namespace mola::factors
