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
#pragma once

#include <gtsam/navigation/AHRSFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <mola_gtsam_factors/FactorImuPreintegrationBodyVel.h>  // detail::OptionalMatrix
#include <mola_gtsam_factors/gtsam_detect_version.h>

namespace mola::factors
{
/** Gyroscope-only preintegration factor between the rotations of two keyframe
 *  poses: the relative rotation integrated from the raw gyro readings (with the
 *  gyro bias correction to first order), as gtsam::AHRSFactor, but on Pose3
 *  keyframes and on the gyroscope part of an imuBias::ConstantBias variable, the
 *  same variables full IMU preintegration uses. Translations are not involved.
 *
 *  `pim` must hold the readings already rotated into the vehicle frame
 *  (integrate_gyro_readings() with R_body_sensor), with no body_P_sensor in its
 *  parameters and its bias linearization point in the vehicle frame: the bias
 *  variable is in the sensor frame, as for gtsam::ImuFactor, and rotated here
 *  (gtsam::AHRSFactor corrects the bias in the vehicle frame).
 *
 *  Keys: T_i (Pose3), T_j (Pose3), B_i (imuBias::ConstantBias).
 */
class FactorGyroPreintegration
    : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3, gtsam::imuBias::ConstantBias>
{
   private:
    using This = FactorGyroPreintegration;
    using Base = gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3, gtsam::imuBias::ConstantBias>;

    gtsam::AHRSFactor ahrs_;
    gtsam::Rot3       R_bs_;  //!< sensor -> vehicle

   public:
    using Base::evaluateError;  // the overloads without Jacobians

    /** `covFloor` [rad^2] is added to the preintegrated covariance, which for
     *  intervals of a few milliseconds is small enough to make the whole system
     *  numerically singular next to the other factors. */
    FactorGyroPreintegration(
        gtsam::Key kTi, gtsam::Key kTj, gtsam::Key kBi,
        const gtsam::PreintegratedAhrsMeasurements& pim, const gtsam::Rot3& R_body_sensor,
        const gtsam::Matrix3& covFloor = gtsam::Matrix3::Zero())
        : Base(
              gtsam::noiseModel::Gaussian::Covariance(pim.preintMeasCov() + covFloor), kTi, kTj,
              kBi),
          ahrs_(kTi, kTj, kBi, pim),
          R_bs_(R_body_sensor)
    {
    }

    gtsam::NonlinearFactor::shared_ptr clone() const override
    {
#if GTSAM_USES_BOOST
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(boost::make_shared<This>(*this));
#else
        return std::static_pointer_cast<gtsam::NonlinearFactor>(std::make_shared<This>(*this));
#endif
    }

    gtsam::Vector evaluateError(
        const gtsam::Pose3& Ti, const gtsam::Pose3& Tj, const gtsam::imuBias::ConstantBias& bi,
        detail::OptionalMatrix H1, detail::OptionalMatrix H2,
        detail::OptionalMatrix H3) const override
    {
        gtsam::Matrix       Hri;
        gtsam::Matrix       Hrj;
        gtsam::Matrix       Hb;
        const gtsam::Vector e = ahrs_.evaluateError(
            Ti.rotation(), Tj.rotation(), R_bs_.rotate(bi.gyroscope()), detail::jacobian(Hri),
            detail::jacobian(Hrj), detail::jacobian(Hb));

        // Pose3 local coordinates are [rotation, translation]; those of
        // imuBias::ConstantBias are [accelerometer, gyroscope].
        if (H1)
        {
            *H1               = gtsam::Matrix36::Zero();
            H1->leftCols<3>() = Hri;
        }
        if (H2)
        {
            *H2               = gtsam::Matrix36::Zero();
            H2->leftCols<3>() = Hrj;
        }
        if (H3)
        {
            *H3                = gtsam::Matrix36::Zero();
            H3->rightCols<3>() = Hb * R_bs_.matrix();
        }
        return e;
    }

    const gtsam::AHRSFactor& ahrsFactor() const { return ahrs_; }
};

}  // namespace mola::factors
