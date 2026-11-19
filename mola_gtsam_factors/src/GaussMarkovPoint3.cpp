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
 * @file   GaussMarkovPoint3.cpp
 * @brief  Factors of a first-order Gauss-Markov process on a Point3 (e.g. a sensor bias)
 * @author Jose Luis Blanco Claraco
 * @date   Nov 19, 2026
 */

#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/expressions.h>
#include <mola_gtsam_factors/GaussMarkovPoint3.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mola::factors
{

GaussMarkovPoint3::GaussMarkovPoint3(const gtsam::Vector3& stationarySigmas, double timeConstant)
    : sigmas_(stationarySigmas), tau_(timeConstant)
{
    if (!(timeConstant > 0) || !(sigmas_.array() >= 0).all())
    {
        throw std::invalid_argument(
            "GaussMarkovPoint3: the time constant must be > 0 and the sigmas >= 0");
    }
}

double GaussMarkovPoint3::phi(double dt) const { return std::exp(-std::abs(dt) / tau_); }

gtsam::Matrix3 GaussMarkovPoint3::transition(double dt) const
{
    const double   p   = phi(dt);
    gtsam::Matrix3 Phi = gtsam::Matrix3::Zero();
    for (int i = 0; i < 3; i++)
    {
        Phi(i, i) = sigmas_[i] > 0 ? p : 0.0;
    }
    return Phi;
}

gtsam::Matrix3 GaussMarkovPoint3::stationary_cov() const
{
    return sigmas_.cwiseProduct(sigmas_).asDiagonal();
}

gtsam::SharedNoiseModel GaussMarkovPoint3::prior_noise() const
{
    gtsam::Vector3 s;
    for (int i = 0; i < 3; i++)
    {
        s[i] = sigmas_[i] > 0 ? sigmas_[i] : HELD_SIGMA;
    }
    return gtsam::noiseModel::Diagonal::Sigmas(s);
}

gtsam::NonlinearFactor::shared_ptr GaussMarkovPoint3::link(
    gtsam::Key prev, gtsam::Key next, double dt) const
{
    const double   p = phi(dt);
    gtsam::Vector3 sigmaW;
    for (int i = 0; i < 3; i++)
    {
        const double s = sigmas_[i];
        // A held axis: only its own tight prior, no link (see prior_noise()).
        sigmaW[i] = s > 0 ? std::max(s * std::sqrt(1.0 - p * p), s / MAX_PRIOR_TO_LINK_SIGMA_RATIO)
                          : HELD_SIGMA;
    }
    const gtsam::Matrix3 Phi       = transition(dt);
    const auto           propagate = [Phi](const gtsam::Point3& b, gtsam::OptionalJacobian<3, 3> H)
    {
        if (H)
        {
            *H = Phi;
        }
        return gtsam::Point3(Phi * b);
    };
    const gtsam::Point3_ prevPropagated(propagate, gtsam::Point3_(prev));
    return gtsam::NonlinearFactor::shared_ptr(new gtsam::ExpressionFactor<gtsam::Point3>(
        gtsam::noiseModel::Diagonal::Sigmas(sigmaW), gtsam::Point3(0, 0, 0),
        gtsam::Point3_(next) - prevPropagated));
}

}  // namespace mola::factors
