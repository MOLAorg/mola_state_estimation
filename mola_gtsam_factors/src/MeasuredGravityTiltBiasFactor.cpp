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

#include <mola_gtsam_factors/MeasuredGravityTiltBiasFactor.h>

namespace mola::factors
{
namespace
{
// Exp([bx, by, 0]) and its Jacobian with respect to (bx, by).
gtsam::Rot3 rot3_from_tilt(const gtsam::Vector2& b, gtsam::OptionalJacobian<3, 2> H)
{
    gtsam::Matrix3 Hexp;
    const auto     R = gtsam::Rot3::Expmap(gtsam::Vector3(b.x(), b.y(), 0.0), H ? &Hexp : nullptr);
    if (H)
    {
        *H = Hexp.leftCols<2>();
    }
    return R;
}
}  // namespace

MeasuredGravityTiltBiasFactor::MeasuredGravityTiltBiasFactor() = default;

MeasuredGravityTiltBiasFactor::MeasuredGravityTiltBiasFactor(
    gtsam::Key kT_enu2map, gtsam::Key kPi, gtsam::Key kTiltBias,
    const gtsam::Pose3& sensorPoseOnVehicle, const gtsam::Vector3& observedNormalizedAcc_b,
    const gtsam::SharedNoiseModel& model)
    : Base({kT_enu2map, kPi, kTiltBias}, model, /* error */ {.0, .0, 1.}),
      sensorPoseOnVehicle_(sensorPoseOnVehicle),
      observedNormalizedAcc_b_(observedNormalizedAcc_b)
{
    this->initialize(This::expression({kT_enu2map, kPi, kTiltBias}));
}

gtsam::NonlinearFactor::shared_ptr MeasuredGravityTiltBiasFactor::clone() const
{
#if GTSAM_USES_BOOST
    return boost::static_pointer_cast<This>(gtsam::NonlinearFactor::shared_ptr(new This(*this)));
#else
    return std::static_pointer_cast<gtsam::NonlinearFactor>(std::make_shared<This>(*this));
#endif
}

gtsam::Expression<gtsam::Point3> MeasuredGravityTiltBiasFactor::expression(
    const std::array<gtsam::Key, NARY_EXPRESSION_SIZE>& keys) const
{
    const gtsam::Expression<gtsam::Pose3>   T_enu2map_(keys[0]);
    const gtsam::Expression<gtsam::Pose3>   Pi_(keys[1]);
    const gtsam::Expression<gtsam::Vector2> bias_(keys[2]);
    const gtsam::Expression<gtsam::Pose3>   p(sensorPoseOnVehicle_);
    const gtsam::Expression<gtsam::Point3>  measuredUp(observedNormalizedAcc_b_);

    const gtsam::Expression<gtsam::Rot3> R_tilt(&rot3_from_tilt, bias_);

    return {gtsam::rotate(
        gtsam::compose(gtsam::rotation(gtsam::compose(T_enu2map_, gtsam::compose(Pi_, p))), R_tilt),
        measuredUp)};
}

void MeasuredGravityTiltBiasFactor::print(
    const std::string& s, const gtsam::KeyFormatter& keyFormatter) const
{
    std::cout << s << "MeasuredGravityTiltBiasFactor(" << keyFormatter(Factor::keys_[0]) << ", "
              << keyFormatter(Factor::keys_[1]) << ", " << keyFormatter(Factor::keys_[2]) << ")\n";
    gtsam::traits<gtsam::Point3>::Print(measured_, "  measured: ");
    this->noiseModel_->print("  noise model: ");
}

bool MeasuredGravityTiltBiasFactor::equals(const gtsam::NonlinearFactor& expected, double tol) const
{
    const This* e = dynamic_cast<const This*>(&expected);
    return e != nullptr && Base::equals(*e, tol);
}

}  // namespace mola::factors
