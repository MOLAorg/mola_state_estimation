/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
 Closed-source licenses available upon request, for this odometry package
 alone or in combination with the complete SLAM system.
*/

/**
 * @file   FactorAngularVelocityIntegration.h
 * @brief  GTSAM factor
 * @author Jose Luis Blanco Claraco
 * @date   Jun 13, 2024
 */

#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/expressions.h>
#include <mola_state_estimation_smoother/gtsam_detect_version.h>

namespace mola::state_estimation_smoother
{
/**
 * Factor for constant angular velocity model, equivalent to expression:
 *
 * Pi +0.5*dt*(gtsam::rotate(Ri, bVi) + gtsam::rotate(Rj, bVj)) - Pj = errZero
 *
 * Note that angular and linear velocities are stored in Values in the body "b"
 * frame, hence the "b" prefix, and the need for the orientations "R".
 */
class FactorTrapezoidalIntegrator : public gtsam::ExpressionFactorN<
                                        gtsam::Point3 /*return type*/,  //
                                        gtsam::Point3, gtsam::Point3, gtsam::Rot3,  // Pi, bVi, Ri
                                        gtsam::Point3, gtsam::Point3, gtsam::Rot3>
{
   private:
    using This = FactorTrapezoidalIntegrator;
    using Base = gtsam::ExpressionFactorN<
        gtsam::Point3 /*return type*/, gtsam::Point3, gtsam::Point3, gtsam::Rot3, gtsam::Point3,
        gtsam::Point3, gtsam::Rot3>;

    double dt_ = .0;

   public:
    /// default constructor
    FactorTrapezoidalIntegrator() = default;

    FactorTrapezoidalIntegrator(
        gtsam::Key kPi, gtsam::Key kVi, gtsam::Key kRi,  //
        gtsam::Key kPj, gtsam::Key kVj, gtsam::Key kRj,  //
        const double dt, const gtsam::SharedNoiseModel& model)
        : Base({kPi, kVi, kRi, kPj, kVj, kRj}, model, /* error=0 */ {0, 0, 0}), dt_(dt)
    {
        this->initialize(This::expression({kPi, kVi, kRi, kPj, kVj, kRj}));
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
        gtsam::Expression<gtsam::Point3> Pi_(keys[0]);
        gtsam::Expression<gtsam::Point3> bVi_(keys[1]);
        gtsam::Expression<gtsam::Rot3>   Ri_(keys[2]);

        gtsam::Expression<gtsam::Point3> Pj_(keys[3]);
        gtsam::Expression<gtsam::Point3> bVj_(keys[4]);
        gtsam::Expression<gtsam::Rot3>   Rj_(keys[5]);

        return {Pi_ + 0.5 * dt_ * (gtsam::rotate(Ri_, bVi_) + gtsam::rotate(Rj_, bVj_)) - Pj_};
    }

    /** implement functions needed for Testable */

    /** print */
    void print(
        const std::string&         s,
        const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override
    {
        std::cout << s << "FactorTrapezoidalIntegrator(" << keyFormatter(Factor::keys_[0]) << ","
                  << keyFormatter(Factor::keys_[1]) << "," << keyFormatter(Factor::keys_[2]) << ","
                  << keyFormatter(Factor::keys_[3]) << "," << keyFormatter(Factor::keys_[4]) << ","
                  << keyFormatter(Factor::keys_[5]) << ")\n";
        gtsam::traits<double>::Print(dt_, "  dt: ");
        gtsam::traits<gtsam::Point3>::Print(measured_, "  measured: ");
        this->noiseModel_->print("  noise model: ");
    }

    /** equals */
    bool equals(const gtsam::NonlinearFactor& expected, double tol = 1e-9) const override
    {
        const This* e = dynamic_cast<const This*>(&expected);
        return e != nullptr && Base::equals(*e, tol) &&
               gtsam::traits<double>::Equals(e->dt_, dt_, tol);
    }

   private:
#if GTSAM_USES_BOOST
    /** Serialization function */
    friend class boost::serialization::access;
    template <class ARCHIVE>
    void serialize(ARCHIVE& ar, const unsigned int /*version*/)
    {
        // **IMPORTANT** We need to deserialize parameters before the base
        // class, since it calls expression() and we need all parameters ready
        // at that point.
        ar& BOOST_SERIALIZATION_NVP(measured_);
        ar& BOOST_SERIALIZATION_NVP(dt_);
        ar& boost::serialization::make_nvp(
            "FactorTrapezoidalIntegrator", boost::serialization::base_object<Base>(*this));
    }
#endif
};

}  // namespace mola::state_estimation_smoother
