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

#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <mola_gtsam_factors/gtsam_detect_version.h>

namespace mola::factors
{
namespace detail
{
// The optional Jacobian argument of evaluateError(), and how to pass a matrix
// to it, in both GTSAM APIs:
#if GTSAM_USES_BOOST
using OptionalMatrix = boost::optional<gtsam::Matrix&>;
inline OptionalMatrix jacobian(gtsam::Matrix& m) { return OptionalMatrix(m); }
#else
using OptionalMatrix = gtsam::OptionalMatrixType;
inline OptionalMatrix jacobian(gtsam::Matrix& m) { return &m; }
#endif
}  // namespace detail

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
#if GTSAM_USES_BOOST
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(boost::make_shared<This>(*this));
#else
        return std::static_pointer_cast<gtsam::NonlinearFactor>(std::make_shared<This>(*this));
#endif
    }

    gtsam::Vector evaluateError(
        const gtsam::Pose3& Ti, const gtsam::Point3& vbi, const gtsam::Pose3& Tj,
        const gtsam::Point3& vbj, const gtsam::imuBias::ConstantBias& bi, detail::OptionalMatrix H1,
        detail::OptionalMatrix H2, detail::OptionalMatrix H3, detail::OptionalMatrix H4,
        detail::OptionalMatrix H5) const override
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
        const gtsam::Vector e = imu_.evaluateError(
            Ti, vwi, Tj, vwj, bi, detail::jacobian(Hpi), detail::jacobian(Hvi),
            detail::jacobian(Hpj), detail::jacobian(Hvj), detail::jacobian(Hb));

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

/** The same factor for keyframe poses in a frame {map} that need not be
 *  gravity-aligned: gravity is along -z of {enu}, and T_enu_to_map (the pose
 *  of {map} in {enu}) is one more variable. Only its roll and pitch change the
 *  residual, not its azimuth or translation.
 *
 *  Keys: T_enu_to_map (Pose3), T_i (Pose3, in {map}), V_i (Point3, body), T_j,
 *  V_j, B_i (imuBias::ConstantBias).
 */
class FactorImuPreintegrationBodyVelEnu
    : public gtsam::NoiseModelFactorN<
          gtsam::Pose3, gtsam::Pose3, gtsam::Point3, gtsam::Pose3, gtsam::Point3,
          gtsam::imuBias::ConstantBias>
{
   private:
    using This = FactorImuPreintegrationBodyVelEnu;
    using Base = gtsam::NoiseModelFactorN<
        gtsam::Pose3, gtsam::Pose3, gtsam::Point3, gtsam::Pose3, gtsam::Point3,
        gtsam::imuBias::ConstantBias>;

    FactorImuPreintegrationBodyVel inEnu_;

   public:
    using Base::evaluateError;  // the overloads without Jacobians

    FactorImuPreintegrationBodyVelEnu() = default;

    /** `covFloor` as in FactorImuPreintegrationBodyVel. */
    FactorImuPreintegrationBodyVelEnu(
        gtsam::Key kEnuToMap, gtsam::Key kTi, gtsam::Key kVi, gtsam::Key kTj, gtsam::Key kVj,
        gtsam::Key kBi, const gtsam::PreintegratedImuMeasurements& pim,
        const gtsam::Matrix9& covFloor = gtsam::Matrix9::Zero())
        : Base(
              gtsam::noiseModel::Gaussian::Covariance(pim.preintMeasCov() + covFloor), kEnuToMap,
              kTi, kVi, kTj, kVj, kBi),
          inEnu_(kTi, kVi, kTj, kVj, kBi, pim, covFloor)
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
        const gtsam::Pose3& T_enu_to_map, const gtsam::Pose3& Ti, const gtsam::Point3& vbi,
        const gtsam::Pose3& Tj, const gtsam::Point3& vbj, const gtsam::imuBias::ConstantBias& bi,
        detail::OptionalMatrix H1, detail::OptionalMatrix H2, detail::OptionalMatrix H3,
        detail::OptionalMatrix H4, detail::OptionalMatrix H5,
        detail::OptionalMatrix H6) const override
    {
        // Body velocities are the same in both frames; only the poses move.
        gtsam::Matrix6     dEi_dF;
        gtsam::Matrix6     dEi_dTi;
        gtsam::Matrix6     dEj_dF;
        gtsam::Matrix6     dEj_dTj;
        const gtsam::Pose3 Ei = T_enu_to_map.compose(Ti, dEi_dF, dEi_dTi);
        const gtsam::Pose3 Ej = T_enu_to_map.compose(Tj, dEj_dF, dEj_dTj);

        gtsam::Matrix       Hei;
        gtsam::Matrix       Hvi;
        gtsam::Matrix       Hej;
        gtsam::Matrix       Hvj;
        gtsam::Matrix       Hb;
        const gtsam::Vector e = inEnu_.evaluateError(
            Ei, vbi, Ej, vbj, bi, detail::jacobian(Hei), detail::jacobian(Hvi),
            detail::jacobian(Hej), detail::jacobian(Hvj), detail::jacobian(Hb));

        if (H1)
        {
            *H1 = Hei * dEi_dF + Hej * dEj_dF;
        }
        if (H2)
        {
            *H2 = Hei * dEi_dTi;
        }
        if (H3)
        {
            *H3 = Hvi;
        }
        if (H4)
        {
            *H4 = Hej * dEj_dTj;
        }
        if (H5)
        {
            *H5 = Hvj;
        }
        if (H6)
        {
            *H6 = Hb;
        }
        return e;
    }

    const FactorImuPreintegrationBodyVel& inEnuFactor() const { return inEnu_; }
};

}  // namespace mola::factors
