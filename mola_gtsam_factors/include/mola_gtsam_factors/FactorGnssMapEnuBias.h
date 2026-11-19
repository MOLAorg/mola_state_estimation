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
 * @file   FactorGnssMapEnuBias.h
 * @brief  GNSS factor with explicit T_enu_to_map and a receiver position bias
 * @author Jose Luis Blanco Claraco
 * @date   Nov 19, 2026
 */

#pragma once

// GTSAM:
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/expressions.h>
#include <mola_gtsam_factors/gtsam_detect_version.h>

#include <iostream>

namespace mola::factors
{
/**
 * @brief GNSS position factor with a receiver bias state.
 *
 * Measurement model (antenna position in ENU, from the fix):
 *
 *   z = T_enu2map * T_map2i * l + b
 *
 * where `l` is the antenna (lever arm) in the vehicle frame and `b` the slowly
 * varying position error of the receiver, in ENU axes, as a variable of the
 * graph (e.g. a chain of GaussMarkovPoint3 variables). This is
 * FactorGnssMapEnu plus `b`; FactorGnssMapEnuConstVelBias is the version for a
 * fix stamped away from its keyframe.
 */
class FactorGnssMapEnuBias : public gtsam::ExpressionFactorN<
                                 gtsam::Point3 /*return*/, gtsam::Pose3 /* T_enu_2_map */,
                                 gtsam::Pose3 /* T_map_2_i */, gtsam::Point3 /* bias */
                                 >
{
   private:
    using This = FactorGnssMapEnuBias;
    using Base = gtsam::ExpressionFactorN<gtsam::Point3, gtsam::Pose3, gtsam::Pose3, gtsam::Point3>;

    gtsam::Point3 sensorOnVehicle_ = {0, 0, 0};

   public:
    /// Default constructor for serialization
    FactorGnssMapEnuBias();

    /**
     * @param kT_enu2map Key for the ENU-to-map transform
     * @param kT_map2i Key for the vehicle pose in map coordinates
     * @param kBias Key for the receiver bias (Point3, ENU axes) [m]
     * @param sensorOnVehicle GNSS antenna position in the vehicle frame
     * @param observedENU Measured GNSS position in ENU
     * @param model Noise model (3D position uncertainty, white part only)
     */
    FactorGnssMapEnuBias(
        gtsam::Key kT_enu2map, gtsam::Key kT_map2i, gtsam::Key kBias,
        const gtsam::Point3& sensorOnVehicle, const gtsam::Point3& observedENU,
        const gtsam::SharedNoiseModel& model)
        : Base({kT_enu2map, kT_map2i, kBias}, model, observedENU), sensorOnVehicle_(sensorOnVehicle)
    {
        this->initialize(expression({kT_enu2map, kT_map2i, kBias}));
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
        gtsam::Point3_ bias(keys[2]);

        return {gtsam::transformFrom(T_enu2map * T_map2i, gtsam::Point3_(sensorOnVehicle_)) + bias};
    }

    /** Print factor details */
    void print(
        const std::string&         s,
        const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override
    {
        std::cout << s << "FactorGnssMapEnuBias(" << keyFormatter(Factor::keys_[0]) << ", "
                  << keyFormatter(Factor::keys_[1]) << ", " << keyFormatter(Factor::keys_[2])
                  << ")\n";
        gtsam::traits<gtsam::Point3>::Print(sensorOnVehicle_, "  sensorOnVehicle: ");
        gtsam::traits<gtsam::Point3>::Print(measured_, "  measured: ");
        this->noiseModel_->print("  noise model: ");
    }

    /** equals */
    bool equals(const gtsam::NonlinearFactor& expected, double tol = 1e-9) const override
    {
        const This* e = dynamic_cast<const This*>(&expected);
        return e != nullptr && Base::equals(*e, tol) &&
               gtsam::traits<gtsam::Point3>::Equals(e->sensorOnVehicle_, sensorOnVehicle_, tol);
    }

   private:
#if GTSAM_USES_BOOST
    friend class boost::serialization::access;
    template <class ARCHIVE>
    void serialize(ARCHIVE& ar, const unsigned int /*version*/)
    {
        ar& BOOST_SERIALIZATION_NVP(measured_);  // params before base class
        ar& BOOST_SERIALIZATION_NVP(sensorOnVehicle_);
        ar& boost::serialization::make_nvp(
            "FactorGnssMapEnuBias", boost::serialization::base_object<Base>(*this));
    }
#endif
};
}  // namespace mola::factors
