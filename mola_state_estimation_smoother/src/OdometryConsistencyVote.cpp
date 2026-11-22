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
 * @file   OdometryConsistencyVote.cpp
 * @brief  Sliding-window consistency vote between a pose source, the wheel
 *         odometry and a gyroscope (see Parameters::OdometryVote).
 * @author Jose Luis Blanco Claraco
 */

#include "OdometryConsistencyVote.h"

#include <mrpt/core/bits_math.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/poses/Lie/SO.h>

#include <cmath>

namespace mola::state_estimation_smoother
{
namespace
{
// Readings further apart than this are not integrated across [s].
constexpr double kMaxGyroGap = 0.5;

// A window counts as covered if its data reach this close to both ends [s].
constexpr double kCoverageSlack = 0.5;

// Wheels standing still: below these rates, for at least kStationaryTime.
constexpr double kStationarySpeed     = 0.01;  // [m/s]
constexpr double kStationaryTurnRate  = 0.01;  // [rad/s]
constexpr double kStationaryTime      = 1.0;  // [s]
constexpr double kBiasForgetting      = 300.0;  // [s] time constant
constexpr double kMinBiasAveragedTime = 2.0;  // [s]

struct Sums
{
    double dist  = 0;
    double dyaw  = 0;
    double first = 0;
    double last  = 0;
    bool   any   = false;
};

template <class Q>
Sums sum_window(const Q& q, double t0, double t1)
{
    Sums s;
    for (const auto& st : q)
    {
        if (st.t <= t0 || st.t > t1)
        {
            continue;
        }
        if (!s.any)
        {
            s.first = st.t;
        }
        s.any  = true;
        s.last = st.t;
        s.dist += st.dist;
        s.dyaw += st.dyaw;
    }
    return s;
}

bool covers(const Sums& s, double t0, double t1, double typicalStep)
{
    return s.any && s.first <= t0 + kCoverageSlack + typicalStep && s.last >= t1 - kCoverageSlack;
}
}  // namespace

void OdometryConsistencyVote::prune(std::deque<Step>& q, double t) const
{
    while (!q.empty() && q.front().t < t - 2 * p_.window)
    {
        q.pop_front();
    }
}

void OdometryConsistencyVote::add_wheels(double t, const mrpt::poses::CPose2D& odom)
{
    if (lastWheels_ && t <= lastWheels_->first)
    {
        return;
    }
    if (lastWheels_)
    {
        const double dt = t - lastWheels_->first;
        const double dist =
            std::hypot(odom.x() - lastWheels_->second.x(), odom.y() - lastWheels_->second.y());
        const double dyaw = mrpt::math::wrapToPi(odom.phi() - lastWheels_->second.phi());
        wheels_.push_back({t, dist, dyaw});
        if (dist > kStationarySpeed * dt || std::abs(dyaw) > kStationaryTurnRate * dt)
        {
            lastWheelsMotion_ = t;
            pendingBias_.clear();
        }
        else if (lastWheelsMotion_ && t - *lastWheelsMotion_ > kStationaryTime)
        {
            // Gyroscope readings up to this one were taken standing still:
            while (!pendingBias_.empty() && pendingBias_.front().t <= t)
            {
                const auto&  g = pendingBias_.front();
                const double f = std::exp(-g.dist / kBiasForgetting);
                biasSum_       = biasSum_ * f + g.dyaw;
                biasTime_      = biasTime_ * f + g.dist;
                pendingBias_.pop_front();
            }
        }
    }
    else
    {
        lastWheelsMotion_ = t;
    }
    lastWheels_ = {t, odom};
    prune(wheels_, t);
}

void OdometryConsistencyVote::add_gyro(double t, double wz)
{
    if (lastGyroStamp_ && t <= *lastGyroStamp_)
    {
        return;
    }
    if (lastGyroStamp_)
    {
        const double dt = t - *lastGyroStamp_;
        if (dt < kMaxGyroGap)
        {
            gyro_.push_back({t, dt, wz * dt});
            if (lastWheelsMotion_)
            {
                pendingBias_.push_back({t, dt, wz * dt});
                prune(pendingBias_, t);
            }
        }
    }
    lastGyroStamp_ = t;
    prune(gyro_, t);
}

std::optional<double> OdometryConsistencyVote::gyro_bias() const
{
    if (biasTime_ < kMinBiasAveragedTime)
    {
        return std::nullopt;
    }
    return biasSum_ / biasTime_;
}

void OdometryConsistencyVote::add_pose(double t, const mrpt::poses::CPose3D& pose)
{
    if (lastPose_ && t <= lastPose_->first)
    {
        return;
    }
    if (lastPose_)
    {
        const mrpt::poses::CPose3D inc = pose - lastPose_->second;
        const auto                 w   = mrpt::poses::Lie::SO<3>::log(inc.getRotationMatrix());
        pose_.push_back({t, inc.translation().norm(), w[2]});
    }
    lastPose_ = {t, pose};
    prune(pose_, t);
}

OdometryConsistencyVote::Verdict OdometryConsistencyVote::verdict(double t) const
{
    Verdict      v;
    const double t0 = t - p_.window;

    const Sums sp = sum_window(pose_, t0, t);
    const Sums sw = sum_window(wheels_, t0, t);
    // The first step of a window ends one reading period after its start:
    constexpr double kTypicalStep = 0.2;
    if (covers(sp, t0, t, kTypicalStep) && covers(sw, t0, t, kTypicalStep))
    {
        v.pose_distance   = sp.dist;
        v.wheels_distance = sw.dist;
        v.translation     = std::max(sp.dist, sw.dist) > p_.min_distance &&
                        std::abs(sp.dist - sw.dist) > p_.tolerance_abs + p_.tolerance_rel * sw.dist;
    }

    const auto bias = gyro_bias();
    const Sums sg   = sum_window(gyro_, t0, t);
    if (bias && covers(sp, t0, t, kTypicalStep) && covers(sg, t0, t, kTypicalStep))
    {
        // For the gyroscope, dist holds the integrated time and dyaw the raw integral:
        const double gyroYaw = sg.dyaw - *bias * sg.dist;
        v.yaw_difference     = sp.dyaw - gyroYaw;
        v.rotation           = std::abs(v.yaw_difference) > mrpt::DEG2RAD(p_.yaw_tolerance_deg);
    }
    return v;
}

}  // namespace mola::state_estimation_smoother
