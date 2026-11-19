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
 * @file   GaussMarkovPoint3.h
 * @brief  Factors of a first-order Gauss-Markov process on a Point3 (e.g. a sensor bias)
 * @author Jose Luis Blanco Claraco
 * @date   Nov 19, 2026
 */

#pragma once

// GTSAM:
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace mola::factors
{
/**
 * @brief A first-order Gauss-Markov process on a Point3, sampled at irregular
 * times, one graph variable per sample (e.g. the position bias of a GNSS
 * receiver, see FactorGnssMapEnuBias):
 *
 *   b(k) = Phi b(k-1) + w(k),   Phi = diag(phi_i),  phi_i = exp(-dt / tau),
 *   w_i(k) ~ N(0, s_i^2 (1 - phi_i^2)),
 *
 * started from its stationary distribution N(0, diag(s_i^2)), the prior of the
 * first variable of a chain. Its marginal variance never exceeds s_i^2; the
 * random walk is the limit tau -> inf with s_i sqrt(2 / tau) per sqrt(s) fixed.
 *
 * An axis with s_i = 0 is not a process: it is held at zero, each variable
 * with its own tight prior (HELD_SIGMA) and no link to the previous one.
 *
 * Usage: `prior_noise()` for the first variable (mean zero), `link()` between
 * consecutive ones, `transition()` and `stationary_cov()` to predict a later
 * value from an estimate.
 */
class GaussMarkovPoint3
{
   public:
    /// Sigma [same units] holding at zero an axis that is not a process
    static constexpr double HELD_SIGMA = 1e-3;

    /** Links of (almost) simultaneous samples are floored at s_i divided by
     *  this, or the chain becomes too stiff with respect to its prior to be
     *  factorized (GTSAM rejects badly conditioned Cholesky pivots). */
    static constexpr double MAX_PRIOR_TO_LINK_SIGMA_RATIO = 1000.0;

    /**
     * @param stationarySigmas s_i per axis; 0 holds that axis at zero
     * @param timeConstant tau [s], > 0
     */
    GaussMarkovPoint3(const gtsam::Vector3& stationarySigmas, double timeConstant);

    /// The stationary sigmas s_i
    [[nodiscard]] const gtsam::Vector3& stationary_sigmas() const { return sigmas_; }

    /// The time constant tau [s]
    [[nodiscard]] double time_constant() const { return tau_; }

    /// phi = exp(-dt / tau) of the axes that are processes
    [[nodiscard]] double phi(double dt) const;

    /// Phi(dt): phi on the process axes, 0 on the held ones
    [[nodiscard]] gtsam::Matrix3 transition(double dt) const;

    /// diag(s_i^2): the variance a value tends to, with nothing observing it
    [[nodiscard]] gtsam::Matrix3 stationary_cov() const;

    /// Noise of the prior N(0, .) of the first variable of a chain
    [[nodiscard]] gtsam::SharedNoiseModel prior_noise() const;

    /// The factor linking variable `next` to `prev`, sampled `dt` [s] later
    [[nodiscard]] gtsam::NonlinearFactor::shared_ptr link(
        gtsam::Key prev, gtsam::Key next, double dt) const;

   private:
    gtsam::Vector3 sigmas_;
    double         tau_ = 1.0;
};

}  // namespace mola::factors
