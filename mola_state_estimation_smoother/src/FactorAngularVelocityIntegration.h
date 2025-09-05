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
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/expressions.h>
#include <mola_state_estimation_smoother/gtsam_detect_version.h>

namespace mola::state_estimation_smoother
{
/**
 * Factor for angular velocity integration model, equivalent to expressions:
 *
 *   gtsam::Expression<gtsam::Point3> deltaWi = dt * bWi;
 *   gtsam::Expression<gtsam::Rot3> expmap_(&gtsam::Rot3::Expmap, deltaWi);
 *
 *   gtsam::between( gtsam::compose(Ri, expmap_), Rj ) = Rot_Identity
 *
 */
class FactorAngularVelocityIntegration : public gtsam::ExpressionFactorN<
                                             gtsam::Rot3 /*return type*/, gtsam::Rot3 /* Ri */,
                                             gtsam::Point3 /* bWi */, gtsam::Rot3 /* Rj */
                                             >
{
   private:
    using This = FactorAngularVelocityIntegration;
    using Base = gtsam::ExpressionFactorN<
        gtsam::Rot3 /*return type*/, gtsam::Rot3 /* Ri */, gtsam::Point3 /* bWi */,
        gtsam::Rot3 /* Rj */
        >;

    double dt_ = .0;

   public:
    FactorAngularVelocityIntegration()           = default;
    ~FactorAngularVelocityIntegration() override = default;

    FactorAngularVelocityIntegration(const FactorAngularVelocityIntegration&)            = default;
    FactorAngularVelocityIntegration& operator=(const FactorAngularVelocityIntegration&) = default;
    FactorAngularVelocityIntegration(FactorAngularVelocityIntegration&&)                 = default;
    FactorAngularVelocityIntegration& operator=(FactorAngularVelocityIntegration&&)      = default;

    FactorAngularVelocityIntegration(
        gtsam::Key kRi, gtsam::Key kbWi, gtsam::Key kRj, const double dt,
        const gtsam::SharedNoiseModel& model)
        : Base({kRi, kbWi, kRj}, model, gtsam::Rot3()), dt_(dt)
    {
        this->initialize(This::expression({kRi, kbWi, kRj}));
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
    gtsam::Expression<gtsam::Rot3> expression(
        const std::array<gtsam::Key, NARY_EXPRESSION_SIZE>& keys) const override
    {
        gtsam::Rot3_                     Ri(keys[0]);
        gtsam::Point3_                   bWi(keys[1]);
        gtsam::Rot3_                     Rj(keys[2]);
        gtsam::Expression<gtsam::Point3> deltaWi = dt_ * bWi;
        gtsam::Expression<gtsam::Rot3>   expmap_(&gtsam::Rot3::Expmap, deltaWi);

        return gtsam::between(gtsam::compose(Ri, expmap_), Rj);
    }

    /** implement functions needed for Testable */

    /** print */
    void print(
        const std::string&         s,
        const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override
    {
        std::cout << s << "FactorAngularVelocityIntegration(" << keyFormatter(Factor::keys_[0])
                  << "," << keyFormatter(Factor::keys_[1]) << "," << keyFormatter(Factor::keys_[2])
                  << ")\n";
        gtsam::traits<gtsam::Rot3>::Print(measured_, "  measured: ");
        gtsam::traits<double>::Print(dt_, "  dt: ");
        this->noiseModel_->print("  noise model: ");
    }

    /** equals */
    bool equals(const gtsam::NonlinearFactor& expected, double tol = 1e-9) const override
    {
        const This* e = dynamic_cast<const This*>(&expected);
        return e != nullptr && Base::equals(*e, tol) &&
               gtsam::traits<double>::Equals(dt_, e->dt_, tol);
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
            "FactorAngularVelocityIntegration", boost::serialization::base_object<Base>(*this));
    }
#endif
};

}  // namespace mola::state_estimation_smoother
