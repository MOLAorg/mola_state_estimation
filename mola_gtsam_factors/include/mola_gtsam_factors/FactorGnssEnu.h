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

#pragma once

// GTSAM:
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/expressions.h>
#include <mola_gtsam_factors/gtsam_detect_version.h>

namespace mola::factors
{
class FactorGnssEnu
    : public gtsam::ExpressionFactorN<gtsam::Point3 /*return*/, gtsam::Pose3 /*pose*/>
{
   private:
    using This = FactorGnssEnu;
    using Base = gtsam::ExpressionFactorN<gtsam::Point3 /*return*/, gtsam::Pose3 /*pose*/>;

    gtsam::Point3 sensorOnVehicle_ = {0, 0, 0};

   public:
    /// default constructor
    FactorGnssEnu();

    FactorGnssEnu(
        gtsam::Key kPi, const gtsam::Point3& sensorOnVehicle,  // NOLINT
        const gtsam::Point3& observedENU, const gtsam::SharedNoiseModel& model)
        : Base({kPi}, model, observedENU), sensorOnVehicle_(sensorOnVehicle)
    {
        this->initialize(expression({kPi}));
    }

    /// @return a deep copy of this factor
    gtsam::NonlinearFactor::shared_ptr clone() const override
    {
#if GTSAM_USES_BOOST
        return boost::static_pointer_cast<This>(
            gtsam::NonlinearFactor::shared_ptr(new This(*this)));
#else
        return std::static_pointer_cast<gtsam::NonlinearFactor>(std::make_shared<This>(*this));
#endif
    }

    // Return measurement expression
    gtsam::Expression<gtsam::Point3> expression(
        const std::array<gtsam::Key, NARY_EXPRESSION_SIZE>& keys) const override
    {
        gtsam::Pose3_ Pi(keys[0]);

        return {gtsam::transformFrom(Pi, gtsam::Point3_(sensorOnVehicle_))};
    }

    /** implement functions needed for Testable */

    /** print */
    void print(
        const std::string&         s,
        const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override
    {
        std::cout << s << "FactorGnssEnu(" << keyFormatter(Factor::keys_[0]) << ")\n";
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
            "FactorGnssEnu", boost::serialization::base_object<Base>(*this));
    }
#endif
};
}  // namespace mola::factors