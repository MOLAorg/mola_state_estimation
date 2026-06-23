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
 * @file   factor_builders.h
 * @brief  Internal helpers to emit GTSAM factors into a given factor graph.
 * @author Jose Luis Blanco Claraco
 * @date   Jun 22, 2026
 *
 * These free functions hold the single source of truth for constructing the
 * kinematic and twist factors used by the smoother. They are shared between the
 * main backend graph (StateEstimationSmoother) and the lightweight fast
 * predictor (FastPredictor), so both use identical factor definitions.
 *
 * The GTSAM symbol scheme (T/V/W per keyframe, F(0)=T_enu_to_map, F(0)+i=
 * T_map_to_odom_frame_i) is also centralized here.
 */
#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>
#include <mola_gtsam_factors/FactorAngularVelocityIntegration.h>
#include <mola_gtsam_factors/FactorConstLocalVelocity.h>
#include <mola_gtsam_factors/FactorTrapezoidalIntegrator.h>
#include <mola_gtsam_factors/FactorTricycleKinematic.h>
#include <mola_gtsam_factors/id.h>
#include <mola_state_estimation_smoother/Parameters.h>
#include <mrpt/core/exceptions.h>

namespace mola::state_estimation_smoother
{

// GTSAM symbol scheme, shared by the backend and the fast predictor.
using gtsam::symbol_shorthand::F;  // Frame of references (Pose3):
                                   // F(0): T_enu_to_map
                                   // F(i): T_map_to_odometry_frame_i (i>=1)
using gtsam::symbol_shorthand::T;  // Poses                      (Pose3)
using gtsam::symbol_shorthand::V;  // Lin velocity (body frame)  (Point3)
using gtsam::symbol_shorthand::W;  // Ang velocity (body frame)  (Point3)

inline const gtsam::Key symbol_T_enu_to_map         = F(0);
inline const gtsam::Key symbol_T_map_to_odom_i_base = F(0);  // odom[i] = base + i (i>=1)

inline constexpr unsigned int REFERENCE_FRAME_ID = 0;  // (for symbol_T_enu_to_map)

inline constexpr double TRICYCLE_LARGE_SIGMAS = 1e6;

/** Adds the constant local-velocity kinematic factors linking keyframes
 *  `from` and `to` separated by `dt` seconds (Eqs (1),(4) in the MOLA RSS2019
 *  paper). Mutates the passed-in graph only; does no logging.
 */
inline void add_const_vel_kinematics(
    gtsam::NonlinearFactorGraph& graph, const Parameters& params, mola::id_t from, mola::id_t to,
    double dt)
{
    // trick to easily handle queries on exactly an existing keyframe:
    if (dt == 0)
    {
        dt = 1e-5;
    }
    ASSERT_GT_(dt, 0.);

    const double std_lin_vel = params.sigma_random_walk_acceleration_linear;
    const double std_ang_vel = params.sigma_random_walk_acceleration_angular;

    const auto kTi  = T(from);
    const auto kTj  = T(to);
    const auto kbVi = V(from);
    const auto kbVj = V(to);
    const auto kbWi = W(from);
    const auto kbWj = W(to);

    // See line 3 of eq (4) in the MOLA RSS2019 paper.
    graph.emplace_shared<mola::factors::FactorConstLocalVelocityPose>(
        kTi, kbVi, kTj, kbVj, gtsam::noiseModel::Isotropic::Sigma(3, std_lin_vel * dt));

    // \omega is in the body frame; see line 4 of eq (4).
    graph.emplace_shared<mola::factors::FactorConstLocalVelocityPose>(
        kTi, kbWi, kTj, kbWj, gtsam::noiseModel::Isotropic::Sigma(3, std_ang_vel * dt));

    auto noise_kinematicsPosition =
        gtsam::noiseModel::Isotropic::Sigma(3, params.sigma_integrator_position);
    auto noise_kinematicsOrientation =
        gtsam::noiseModel::Isotropic::Sigma(3, params.sigma_integrator_orientation);

    // Impl. line 2 of eq (1).
    graph.emplace_shared<mola::factors::FactorTrapezoidalIntegratorPose>(
        kTi, kbVi, kTj, kbVj, dt, noise_kinematicsPosition);

    // Impl. line 1 of eq (4).
    graph.emplace_shared<mola::factors::FactorAngularVelocityIntegrationPose>(
        kTi, kbWi, kTj, dt, noise_kinematicsOrientation);
}

/** Adds the tricycle kinematic factors linking keyframes `from` and `to`
 *  separated by `dt` seconds. Mutates the passed-in graph only; does no logging.
 */
inline void add_tricycle_kinematics(
    gtsam::NonlinearFactorGraph& graph, const Parameters& params, mola::id_t from, mola::id_t to,
    double dt)
{
    if (dt == 0)
    {
        dt = 1e-5;
    }
    ASSERT_GT_(dt, 0.);

    const double std_lin_vel = params.sigma_random_walk_acceleration_linear;
    const double std_ang_vel = params.sigma_random_walk_acceleration_angular;

    const auto kTi  = T(from);
    const auto kTj  = T(to);
    const auto kbVi = V(from);
    const auto kbVj = V(to);
    const auto kbWi = W(from);
    const auto kbWj = W(to);

    graph.emplace_shared<mola::factors::FactorConstLocalVelocityPose>(
        kTi, kbVi, kTj, kbVj, gtsam::noiseModel::Isotropic::Sigma(3, std_lin_vel * dt));
    graph.emplace_shared<mola::factors::FactorConstLocalVelocityPose>(
        kTi, kbWi, kTj, kbWj, gtsam::noiseModel::Isotropic::Sigma(3, std_ang_vel * dt));

    // In the tricycle model, body v_y must be zero:
    {
        const Eigen::Vector3d sigmas = {
            TRICYCLE_LARGE_SIGMAS, std_lin_vel * dt, TRICYCLE_LARGE_SIGMAS};
        graph.emplace_shared<gtsam::PriorFactor<gtsam::Point3>>(
            kbVj, gtsam::Point3::Zero(), gtsam::noiseModel::Diagonal::Sigmas(sigmas));
    }
    // In the tricycle model, body w_x,w_y must be zero:
    {
        const Eigen::Vector3d sigmas = {std_ang_vel * dt, std_ang_vel * dt, TRICYCLE_LARGE_SIGMAS};
        graph.emplace_shared<gtsam::PriorFactor<gtsam::Point3>>(
            kbWj, gtsam::Point3::Zero(), gtsam::noiseModel::Diagonal::Sigmas(sigmas));
    }

    gtsam::Vector6 sigmas;
    const auto     sigmaPos   = params.sigma_integrator_position;
    const auto     sigmaAngle = params.sigma_integrator_orientation;
    sigmas << sigmaAngle, sigmaAngle, sigmaAngle, sigmaPos, sigmaPos, sigmaPos;
    auto noise_kinematics = gtsam::noiseModel::Diagonal::Sigmas(sigmas);

    graph.emplace_shared<mola::factors::FactorTricycleKinematic>(
        kTi, kbVi, kbWi, kTj, dt, noise_kinematics);
}

/** Dispatches to the configured kinematic model. */
inline void add_kinematic_factors(
    gtsam::NonlinearFactorGraph& graph, const Parameters& params, mola::id_t from, mola::id_t to,
    double dt)
{
    switch (params.kinematic_model)
    {
        case KinematicModel::ConstantVelocity:
            add_const_vel_kinematics(graph, params, from, to, dt);
            break;
        case KinematicModel::Tricycle:
            add_tricycle_kinematics(graph, params, from, to, dt);
            break;
        default:
            THROW_EXCEPTION("Invalid kinematic_model value");
    }
}

/** Adds linear+angular velocity Gaussian priors (twist observation) at keyframe
 *  `kf`. Mutates the passed-in graph only.
 */
inline void add_twist_priors(
    gtsam::NonlinearFactorGraph& graph, mola::id_t kf, const gtsam::Vector3& v,
    const gtsam::Matrix3& vCov, const gtsam::Vector3& w, const gtsam::Matrix3& wCov)
{
    graph.addPrior(V(kf), v, gtsam::noiseModel::Gaussian::Covariance(vCov));
    graph.addPrior(W(kf), w, gtsam::noiseModel::Gaussian::Covariance(wCov));
}

}  // namespace mola::state_estimation_smoother
