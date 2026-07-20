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
 * @file   extrapolation.h
 * @brief  Kinematic short-term extrapolation shared by the backend smoother and
 *         the fast predictor, so both integrate a body twist identically.
 * @author Jose Luis Blanco Claraco
 */
#pragma once

#include <mola_state_estimation_smoother/Parameters.h>
#include <mrpt/core/bits_math.h>  // mrpt::square
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CVectorFixed.h>
#include <mrpt/math/TTwist3D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/Lie/SE.h>

#include <cmath>

namespace mola::state_estimation_smoother
{
/** Integrates a constant body-frame twist over `dt` seconds, returning the
 *  relative pose increment T_i_to_j (right-composed onto an anchor: T_j = T_i
 *  (+) delta). Branches on the configured kinematic model so the short-term
 *  extrapolation matches the motion model used to build the inter-keyframe
 *  factors: full SE(3) exp of the 6D body twist for ConstantVelocity; the planar
 *  arc from forward velocity v=vx and yaw rate w=wz for Tricycle, mirroring
 *  mola::factors::FactorTricycleKinematic exactly.
 */
inline mrpt::poses::CPose3D body_twist_delta(
    const Parameters& params, const mrpt::math::TTwist3D& twist, double dt)
{
    switch (params.kinematic_model)
    {
        case KinematicModel::Tricycle:
        {
            constexpr double w_threshold = 1e-4;  // [rad/s], as FactorTricycleKinematic
            const double     v           = twist.vx;
            const double     w           = twist.wz;
            if (std::abs(w) < w_threshold)
            {
                return mrpt::poses::CPose3D(v * dt, .0, .0, .0, .0, .0);
            }
            const double R     = v / w;
            const double theta = w * dt;
            const double dx    = R * std::sin(theta);
            const double dy    = R * (1.0 - std::cos(theta));
            // CPose3D(x, y, z, yaw, pitch, roll); the arc rotates about +z (yaw).
            return mrpt::poses::CPose3D(dx, dy, .0, theta, .0, .0);
        }
        case KinematicModel::ConstantVelocity:
        default:
        {
            mrpt::math::CVectorFixed<double, 6> twistDt;
            twistDt[0] = twist.vx;
            twistDt[1] = twist.vy;
            twistDt[2] = twist.vz;
            twistDt[3] = twist.wx;
            twistDt[4] = twist.wy;
            twistDt[5] = twist.wz;
            twistDt *= dt;
            return mrpt::poses::CPose3D(mrpt::poses::Lie::SE<3>::exp(twistDt));
        }
    }
}

/** Covariance-aware short-term pose extrapolation: returns the pose PDF of
 *  `anchorPose` (+) Exp(twist*dt) with first-order uncertainty propagation.
 *
 *  Two sources of uncertainty are combined:
 *   - the anchor pose covariance, transported through the composition (handled
 *     exactly by MRPT's CPose3DPDFGaussian::operator+, i.e. the Adjoint term);
 *   - the body-frame increment covariance: the current-velocity uncertainty
 *     `twistCov` carried over the interval (dt^2 * twistCov), plus an
 *     acceleration process-noise term (~ 0.5*a*dt^2 position/attitude drift).
 *
 *  The increment covariance is expressed in the body tangent [vx vy vz wx wy wz]
 *  (the ordering of NavState::twist_inv_cov) and mapped to the increment's
 *  [x y z yaw pitch roll] covariance with J = d(pose)/d(tangent) ~= I for the
 *  small increments in this regime (exact in the translation channel, first
 *  order in rotation). Swap in the exact SE(3) exp Jacobian if the rotation
 *  channel ever needs it.
 */
inline mrpt::poses::CPose3DPDFGaussian extrapolate_pose_pdf(
    const Parameters& params, const mrpt::poses::CPose3DPDFGaussian& anchorPose,
    const mrpt::math::TTwist3D& twist, const mrpt::math::CMatrixDouble66& twistCov, double dt)
{
    mrpt::poses::CPose3DPDFGaussian deltaPdf;
    deltaPdf.mean = body_twist_delta(params, twist, dt);

    mrpt::math::CMatrixDouble66 incrCov = twistCov;
    incrCov *= mrpt::square(dt);

    const double halfDt2 = 0.5 * dt * dt;
    for (int i = 0; i < 3; i++)
    {
        incrCov(i, i) += mrpt::square(params.sigma_random_walk_acceleration_linear * halfDt2);
        incrCov(3 + i, 3 + i) +=
            mrpt::square(params.sigma_random_walk_acceleration_angular * halfDt2);
    }
    deltaPdf.cov = incrCov;

    return anchorPose + deltaPdf;
}

}  // namespace mola::state_estimation_smoother
