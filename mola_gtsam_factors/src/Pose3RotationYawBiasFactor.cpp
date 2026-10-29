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
 * @file   Pose3RotationYawBiasFactor.cpp
 * @brief  Absolute attitude observation with an unknown azimuth (yaw) bias
 * @author Jose Luis Blanco Claraco
 * @date   Oct 29, 2026
 */

#include <mola_gtsam_factors/Pose3RotationYawBiasFactor.h>

namespace mola::factors
{
namespace
{
// Rz(yaw + d) = Rz(yaw) * Exp(d * e_z), so the Jacobian in the Rot3 tangent
// space (right perturbations) is e_z.
gtsam::Rot3 rot3_from_yaw(const double& yaw, gtsam::OptionalJacobian<3, 1> H)
{
    if (H)
    {
        *H << 0, 0, 1;
    }
    return gtsam::Rot3::Rz(yaw);
}
}  // namespace

Pose3RotationYawBiasFactor::Pose3RotationYawBiasFactor() = default;

Pose3RotationYawBiasFactor::Pose3RotationYawBiasFactor(
    gtsam::Key kT_enu2map, gtsam::Key kPi, gtsam::Key kYawBias,
    const gtsam::Pose3& sensorPoseOnVehicle, const gtsam::Rot3& observedOrientation,
    const gtsam::SharedNoiseModel& model)
    : Base({kT_enu2map, kPi, kYawBias}, model, /* error */ observedOrientation),
      sensorPoseOnVehicle_(sensorPoseOnVehicle)
{
    this->initialize(This::expression({kT_enu2map, kPi, kYawBias}));
}

gtsam::NonlinearFactor::shared_ptr Pose3RotationYawBiasFactor::clone() const
{
#if GTSAM_USES_BOOST
    return boost::static_pointer_cast<This>(gtsam::NonlinearFactor::shared_ptr(new This(*this)));
#else
    return std::static_pointer_cast<gtsam::NonlinearFactor>(std::make_shared<This>(*this));
#endif
}

gtsam::Expression<gtsam::Rot3> Pose3RotationYawBiasFactor::expression(
    const std::array<gtsam::Key, NARY_EXPRESSION_SIZE>& keys) const
{
    const gtsam::Expression<gtsam::Pose3> T_enu2map_(keys[0]);
    const gtsam::Expression<gtsam::Pose3> Pi_(keys[1]);
    const gtsam::Expression<double>       bias_(keys[2]);
    const gtsam::Expression<gtsam::Pose3> p(sensorPoseOnVehicle_);

    const gtsam::Expression<gtsam::Rot3> R_bias(&rot3_from_yaw, bias_);

    return gtsam::compose(
        R_bias, gtsam::rotation(gtsam::compose(T_enu2map_, gtsam::compose(Pi_, p))));
}

void Pose3RotationYawBiasFactor::print(
    const std::string& s, const gtsam::KeyFormatter& keyFormatter) const
{
    std::cout << s << "Pose3RotationYawBiasFactor(" << keyFormatter(Factor::keys_[0]) << ", "
              << keyFormatter(Factor::keys_[1]) << ", " << keyFormatter(Factor::keys_[2]) << ")\n";
    gtsam::traits<gtsam::Rot3>::Print(measured_, "  measured: ");
    this->noiseModel_->print("  noise model: ");
}

bool Pose3RotationYawBiasFactor::equals(const gtsam::NonlinearFactor& expected, double tol) const
{
    const This* e = dynamic_cast<const This*>(&expected);
    return e != nullptr && Base::equals(*e, tol);
}

}  // namespace mola::factors
