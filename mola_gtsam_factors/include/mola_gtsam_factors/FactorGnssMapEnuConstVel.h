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

/**
 * @file   FactorGnssMapEnuConstVel.h
 * @brief  GNSS factor for a fix taken at a time offset from its keyframe
 * @author Jose Luis Blanco Claraco
 * @date   Nov 5, 2026
 */

#pragma once

// GTSAM:
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/expressions.h>
#include <mola_gtsam_factors/gtsam_detect_version.h>

#include <cmath>
#include <iostream>

namespace mola::factors
{
/**
 * @brief GNSS position factor for a fix stamped `dt` seconds after (or before,
 *        if negative) the keyframe it is attached to.
 *
 * Like FactorGnssMapEnu, but the antenna is first moved to the fix time with
 * the keyframe's own body-frame twist (constant over `dt`, to first order):
 *
 *   p_body(dt) = sensorOnVehicle + dt * (v + w x sensorOnVehicle)
 *   ENU        = T_enu2map * T_map2i * p_body(dt)
 *
 * so a vehicle moving at speed |v| is not pulled by |v| * dt towards a fix
 * that was taken elsewhere. With dt = 0 it is FactorGnssMapEnu.
 *
 * Keys: T_enu2map (Pose3), T_map2i (Pose3), v_i (Point3, body frame),
 *       w_i (Point3, body frame).
 */
class FactorGnssMapEnuConstVel
    : public gtsam::ExpressionFactorN<
          gtsam::Point3 /*return*/, gtsam::Pose3 /* T_enu_2_map */, gtsam::Pose3 /* T_map_2_i */,
          gtsam::Point3 /* v_i */, gtsam::Point3 /* w_i */
          >
{
   private:
    using This = FactorGnssMapEnuConstVel;
    using Base = gtsam::ExpressionFactorN<
        gtsam::Point3, gtsam::Pose3, gtsam::Pose3, gtsam::Point3, gtsam::Point3>;

    gtsam::Point3 sensorOnVehicle_ = {0, 0, 0};
    double        dt_              = 0;

   public:
    /// Default constructor for serialization
    FactorGnssMapEnuConstVel();

    /**
     * @param kT_enu2map Key for the ENU-to-map transform
     * @param kT_map2i Key for the keyframe pose in map coordinates
     * @param kV Key for the keyframe linear velocity (body frame)
     * @param kW Key for the keyframe angular velocity (body frame)
     * @param sensorOnVehicle GNSS antenna position in the vehicle frame
     * @param dt Fix time minus keyframe time [s]
     * @param observedENU Measured GNSS position in ENU
     * @param model Noise model (3D position uncertainty)
     */
    FactorGnssMapEnuConstVel(
        gtsam::Key kT_enu2map, gtsam::Key kT_map2i, gtsam::Key kV, gtsam::Key kW,
        const gtsam::Point3& sensorOnVehicle, double dt, const gtsam::Point3& observedENU,
        const gtsam::SharedNoiseModel& model)
        : Base({kT_enu2map, kT_map2i, kV, kW}, model, observedENU),
          sensorOnVehicle_(sensorOnVehicle),
          dt_(dt)
    {
        this->initialize(expression({kT_enu2map, kT_map2i, kV, kW}));
    }

    /// @return Deep copy of this factor
    gtsam::NonlinearFactor::shared_ptr clone() const override
    {
#if GTSAM_USES_BOOST
        return boost::static_pointer_cast<This>(
            gtsam::NonlinearFactor::shared_ptr(new This(*this)));
#else
        return std::static_pointer_cast<gtsam::NonlinearFactor>(std::make_shared<This>(*this));
#endif
    }

    gtsam::Expression<gtsam::Point3> expression(
        const std::array<gtsam::Key, NARY_EXPRESSION_SIZE>& keys) const override
    {
        gtsam::Pose3_  T_enu2map(keys[0]);
        gtsam::Pose3_  T_map2i(keys[1]);
        gtsam::Point3_ v(keys[2]);
        gtsam::Point3_ w(keys[3]);

        const gtsam::Point3 lever = sensorOnVehicle_;
        const double        dt    = dt_;

        // Antenna in the keyframe's body frame at the fix time (first order):
        const auto movedLever = [lever, dt](
                                    const gtsam::Point3& vel, const gtsam::Point3& angVel,
                                    gtsam::OptionalJacobian<3, 3> Hv,
                                    gtsam::OptionalJacobian<3, 3> Hw) -> gtsam::Point3
        {
            if (Hv)
            {
                *Hv = dt * gtsam::I_3x3;
            }
            if (Hw)
            {
                // d(w x l)/dw = -skew(l)
                *Hw = -dt * gtsam::skewSymmetric(lever);
            }
            return lever + dt * (vel + angVel.cross(lever));
        };

        const gtsam::Point3_ antennaInBody(movedLever, v, w);

        return {gtsam::transformFrom(T_enu2map * T_map2i, antennaInBody)};
    }

    /** Print factor details */
    void print(
        const std::string&         s,
        const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override
    {
        std::cout << s << "FactorGnssMapEnuConstVel(" << keyFormatter(Factor::keys_[0]) << ", "
                  << keyFormatter(Factor::keys_[1]) << ", " << keyFormatter(Factor::keys_[2])
                  << ", " << keyFormatter(Factor::keys_[3]) << ") dt=" << dt_ << "\n";
        gtsam::traits<gtsam::Point3>::Print(sensorOnVehicle_, "  sensorOnVehicle: ");
        gtsam::traits<gtsam::Point3>::Print(measured_, "  measured: ");
        this->noiseModel_->print("  noise model: ");
    }

    /** equals */
    bool equals(const gtsam::NonlinearFactor& expected, double tol = 1e-9) const override
    {
        const This* e = dynamic_cast<const This*>(&expected);
        return e != nullptr && Base::equals(*e, tol) &&
               gtsam::traits<gtsam::Point3>::Equals(e->sensorOnVehicle_, sensorOnVehicle_, tol) &&
               std::abs(e->dt_ - dt_) <= tol;
    }

   private:
#if GTSAM_USES_BOOST
    friend class boost::serialization::access;
    template <class ARCHIVE>
    void serialize(ARCHIVE& ar, const unsigned int /*version*/)
    {
        ar& BOOST_SERIALIZATION_NVP(measured_);  // params before base class
        ar& BOOST_SERIALIZATION_NVP(sensorOnVehicle_);
        ar& BOOST_SERIALIZATION_NVP(dt_);
        ar& boost::serialization::make_nvp(
            "FactorGnssMapEnuConstVel", boost::serialization::base_object<Base>(*this));
    }
#endif
};
}  // namespace mola::factors
