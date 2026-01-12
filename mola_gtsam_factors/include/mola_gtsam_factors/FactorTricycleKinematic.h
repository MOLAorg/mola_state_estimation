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
 * @file   FactorTricycleKinematic.h
 * @brief  GTSAM factor for tricycle kinematic model
 * @author Jose Luis Blanco Claraco
 * @date   Jan 12, 2026
 */

#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <mola_gtsam_factors/gtsam_detect_version.h>

#include <cmath>

namespace mola::factors
{

/**
 * Factor for tricycle kinematic model using Euler integration.
 *
 * Models motion as a circular arc based on linear velocity v and angular velocity w
 * at the initial timestep (Euler integration).
 *
 * The kinematic model:
 * - For w ≈ 0: straight line motion
 * - For w ≠ 0: circular arc with radius R = v/w
 *
 * State variables:
 * - Ti: SE(3) pose at time i
 * - bVi: linear velocity vector in body frame [vx, vy, vz]
 * - bWi: angular velocity vector in body frame [wx, wy, wz]
 * - Tj: SE(3) pose at time j
 *
 * The factor constrains Tj to be the predicted pose after following
 * the tricycle kinematic model from Ti for duration dt using velocities at time i.
 */
class FactorTricycleKinematic : public gtsam::NoiseModelFactor4<
                                    gtsam::Pose3, gtsam::Point3, gtsam::Point3,  // Ti, bVi, bWi
                                    gtsam::Pose3>  // Tj
{
   private:
    using This = FactorTricycleKinematic;
    using Base = gtsam::NoiseModelFactor4<
        gtsam::Pose3, gtsam::Point3, gtsam::Point3,  // Ti, bVi, bWi
        gtsam::Pose3>;  // Tj

    double dt_          = 0.0;
    double w_threshold_ = 1e-4;  // Threshold for considering w ≈ 0

   public:
    /// default constructor for serialization
    FactorTricycleKinematic();

    /**
     * Constructor
     * @param kTi Key for initial pose
     * @param kVi Key for initial linear velocity (body frame)
     * @param kWi Key for initial angular velocity (body frame)
     * @param kTj Key for final pose
     * @param dt Time interval
     * @param model Noise model (should be 6D for Pose3)
     * @param w_threshold Threshold below which angular velocity is considered zero
     */
    FactorTricycleKinematic(
        gtsam::Key kTi, gtsam::Key kVi, gtsam::Key kWi, gtsam::Key kTj, const double dt,
        const gtsam::SharedNoiseModel& model, const double w_threshold = 1e-4);

    /// @return a deep copy of this factor
    gtsam::NonlinearFactor::shared_ptr clone() const override
    {
#if GTSAM_USES_BOOST
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
            gtsam::NonlinearFactor::shared_ptr(new This(*this)));
#else
        return std::static_pointer_cast<gtsam::NonlinearFactor>(std::make_shared<This>(*this));
#endif
    }

    /**
     * Compute predicted pose using tricycle kinematic model with Euler integration
     */
    static gtsam::Pose3 tricycleKinematicModel(
        const gtsam::Pose3& Ti, const gtsam::Point3& bVi, const gtsam::Point3& bWi, double dt,
        double w_threshold);

    /**
     * Error function
     * Computes the error as the relative pose between predicted and actual Tj
     * Returns a 6D vector: [rotation_error (3D), translation_error (3D)]
     */
    gtsam::Vector evaluateError(
        const gtsam::Pose3& Ti, const gtsam::Point3& bVi, const gtsam::Point3& bWi,
        const gtsam::Pose3& Tj,
#if GTSAM_USES_BOOST
        boost::optional<gtsam::Matrix&> H1, boost::optional<gtsam::Matrix&> H2,
        boost::optional<gtsam::Matrix&> H3, boost::optional<gtsam::Matrix&> H4
#else
        gtsam::Matrix* H1 = nullptr, gtsam::Matrix* H2 = nullptr, gtsam::Matrix* H3 = nullptr,
        gtsam::Matrix* H4 = nullptr
#endif
    ) const override;

    /** print */
    void print(
        const std::string&         s,
        const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override
    {
        std::cout << s << "FactorTricycleKinematic(" << keyFormatter(this->keys_[0]) << ","
                  << keyFormatter(this->keys_[1]) << "," << keyFormatter(this->keys_[2]) << ","
                  << keyFormatter(this->keys_[3]) << ")\n";
        gtsam::traits<double>::Print(dt_, "  dt: ");
        gtsam::traits<double>::Print(w_threshold_, "  w_threshold: ");
        this->noiseModel_->print("  noise model: ");
    }

    /** equals */
    bool equals(const gtsam::NonlinearFactor& expected, double tol = 1e-9) const override
    {
        const This* e = dynamic_cast<const This*>(&expected);
        return e != nullptr && Base::equals(*e, tol) &&
               gtsam::traits<double>::Equals(e->dt_, dt_, tol) &&
               gtsam::traits<double>::Equals(e->w_threshold_, w_threshold_, tol);
    }

   private:
#if GTSAM_USES_BOOST
    /** Serialization function */
    friend class boost::serialization::access;
    template <class ARCHIVE>
    void serialize(ARCHIVE& ar, const unsigned int /*version*/)
    {
        ar& boost::serialization::make_nvp(
            "FactorTricycleKinematic", boost::serialization::base_object<Base>(*this));
        ar& BOOST_SERIALIZATION_NVP(dt_);
        ar& BOOST_SERIALIZATION_NVP(w_threshold_);
    }
#endif
};

}  // namespace mola::factors