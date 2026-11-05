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

#include <mola_gtsam_factors/FactorGyroPreintegrationPose.h>

namespace mola::factors
{
namespace
{
gtsam::SharedNoiseModel noise_with_floor(
    const gtsam::PreintegratedAhrsMeasurements& pim, double floorSigma)
{
    gtsam::Matrix3 cov = pim.preintMeasCov();
    cov += floorSigma * floorSigma * gtsam::I_3x3;
    return gtsam::noiseModel::Gaussian::Covariance(cov);
}
}  // namespace

FactorGyroPreintegrationPose::FactorGyroPreintegrationPose(
    gtsam::Key kTi, gtsam::Key kTj, gtsam::Key kBias,
    const gtsam::PreintegratedAhrsMeasurements& pim, double rotationFloorSigma)
    : Base(noise_with_floor(pim, rotationFloorSigma), kTi, kTj, kBias), ahrs_(kTi, kTj, kBias, pim)
{
}

gtsam::NonlinearFactor::shared_ptr FactorGyroPreintegrationPose::clone() const
{
#if GTSAM_USES_BOOST
    return boost::static_pointer_cast<This>(gtsam::NonlinearFactor::shared_ptr(new This(*this)));
#else
    return std::static_pointer_cast<gtsam::NonlinearFactor>(std::make_shared<This>(*this));
#endif
}

gtsam::Vector FactorGyroPreintegrationPose::evaluateError(
    const gtsam::Pose3& Ti, const gtsam::Pose3& Tj, const gtsam::Vector3& bias,
    gtsam::OptionalMatrixType H1, gtsam::OptionalMatrixType H2, gtsam::OptionalMatrixType H3) const
{
    gtsam::Matrix36   dRi;
    gtsam::Matrix36   dRj;
    const gtsam::Rot3 Ri = Ti.rotation(H1 ? &dRi : nullptr);
    const gtsam::Rot3 Rj = Tj.rotation(H2 ? &dRj : nullptr);

    gtsam::Matrix A1;
    gtsam::Matrix A2;
    gtsam::Vector e = ahrs_.evaluateError(Ri, Rj, bias, H1 ? &A1 : nullptr, H2 ? &A2 : nullptr, H3);
    if (H1)
    {
        *H1 = A1 * dRi;
    }
    if (H2)
    {
        *H2 = A2 * dRj;
    }
    return e;
}

}  // namespace mola::factors
