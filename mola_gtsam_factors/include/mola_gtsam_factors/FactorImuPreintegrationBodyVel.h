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

#include <mola_gtsam_factors/gtsam_detect_version.h>

#if !GTSAM_USES_BOOST

#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace mola::factors
{
/** IMU preintegration factor between two keyframes whose linear velocity is
 *  expressed in the BODY frame, as in mola_state_estimation_smoother.
 *
 *  It wraps gtsam::ImuFactor (rotation, velocity and position increments
 *  preintegrated from raw readings, with the bias correction to first order):
 *  the world-frame velocities that factor expects are R_i * v_i and
 *  R_j * v_j, and the Jacobians are chained through that rotation. The bias
 *  random walk is a separate factor between the two bias variables.
 *
 *  Keys: T_i (Pose3), V_i (Point3, body), T_j, V_j, B_i (imuBias::ConstantBias).
 */
class FactorImuPreintegrationBodyVel
    : public gtsam::NoiseModelFactorN<
          gtsam::Pose3, gtsam::Point3, gtsam::Pose3, gtsam::Point3, gtsam::imuBias::ConstantBias>
{
   private:
    using This = FactorImuPreintegrationBodyVel;
    using Base = gtsam::NoiseModelFactorN<
        gtsam::Pose3, gtsam::Point3, gtsam::Pose3, gtsam::Point3, gtsam::imuBias::ConstantBias>;

    gtsam::ImuFactor imu_;

   public:
    using Base::evaluateError;  // the overloads without Jacobians

    FactorImuPreintegrationBodyVel() = default;

    /** `covFloor` (9x9, in the order of the preintegrated state: rotation,
     *  position, velocity) is added to the preintegrated covariance, which for
     *  intervals of a few milliseconds is small enough to make the whole
     *  system numerically singular next to the other factors. */
    FactorImuPreintegrationBodyVel(
        gtsam::Key kTi, gtsam::Key kVi, gtsam::Key kTj, gtsam::Key kVj, gtsam::Key kBi,
        const gtsam::PreintegratedImuMeasurements& pim,
        const gtsam::Matrix9&                      covFloor = gtsam::Matrix9::Zero())
        : Base(
              gtsam::noiseModel::Gaussian::Covariance(pim.preintMeasCov() + covFloor), kTi, kVi,
              kTj, kVj, kBi),
          imu_(kTi, kVi, kTj, kVj, kBi, pim)
    {
    }

    gtsam::NonlinearFactor::shared_ptr clone() const override
    {
        return std::static_pointer_cast<gtsam::NonlinearFactor>(std::make_shared<This>(*this));
    }

    gtsam::Vector evaluateError(
        const gtsam::Pose3& Ti, const gtsam::Point3& vbi, const gtsam::Pose3& Tj,
        const gtsam::Point3& vbj, const gtsam::imuBias::ConstantBias& bi,
        gtsam::OptionalMatrixType H1, gtsam::OptionalMatrixType H2, gtsam::OptionalMatrixType H3,
        gtsam::OptionalMatrixType H4, gtsam::OptionalMatrixType H5) const override
    {
        gtsam::Matrix3       dVwi_dRi;
        gtsam::Matrix3       dVwi_dVbi;
        gtsam::Matrix3       dVwj_dRj;
        gtsam::Matrix3       dVwj_dVbj;
        const gtsam::Vector3 vwi = Ti.rotation().rotate(vbi, dVwi_dRi, dVwi_dVbi);
        const gtsam::Vector3 vwj = Tj.rotation().rotate(vbj, dVwj_dRj, dVwj_dVbj);

        gtsam::Matrix       Hpi;
        gtsam::Matrix       Hvi;
        gtsam::Matrix       Hpj;
        gtsam::Matrix       Hvj;
        gtsam::Matrix       Hb;
        const gtsam::Vector e =
            imu_.evaluateError(Ti, vwi, Tj, vwj, bi, &Hpi, &Hvi, &Hpj, &Hvj, &Hb);

        // The world velocity depends on the pose through its rotation only;
        // Pose3 local coordinates are [rotation, translation].
        if (H1)
        {
            gtsam::Matrix36 D = gtsam::Matrix36::Zero();
            D.leftCols<3>()   = dVwi_dRi;
            *H1               = Hpi + Hvi * D;
        }
        if (H2)
        {
            *H2 = Hvi * dVwi_dVbi;
        }
        if (H3)
        {
            gtsam::Matrix36 D = gtsam::Matrix36::Zero();
            D.leftCols<3>()   = dVwj_dRj;
            *H3               = Hpj + Hvj * D;
        }
        if (H4)
        {
            *H4 = Hvj * dVwj_dVbj;
        }
        if (H5)
        {
            *H5 = Hb;
        }
        return e;
    }

    const gtsam::ImuFactor& imuFactor() const { return imu_; }
};

}  // namespace mola::factors

#endif
