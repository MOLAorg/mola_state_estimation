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
 * @file   OdometryConsistencyVote.h
 * @brief  Sliding-window consistency vote between a pose source, the wheel
 *         odometry and a gyroscope (see Parameters::OdometryVote).
 * @author Jose Luis Blanco Claraco
 */
#pragma once

#include <mola_state_estimation_smoother/Parameters.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>

#include <deque>
#include <optional>

namespace mola::state_estimation_smoother
{
/** Compares, over the last `window` seconds, the distance traveled by a pose
 * source with that of the wheel odometry, and its heading change with the
 * integrated gyroscope. Distances are path lengths (sums of increment norms),
 * so they do not depend on the frame or the heading of each source. The
 * gyroscope bias is averaged while the wheels stand still; until then there
 * is no heading vote. Times are in seconds, any common clock.
 */
class OdometryConsistencyVote
{
   public:
    explicit OdometryConsistencyVote(const Parameters::OdometryVote& p) : p_(p) {}

    /// A dead-reckoned wheel odometry pose (planar, in its own frame).
    void add_wheels(double t, const mrpt::poses::CPose2D& odom);

    /// A gyroscope reading: the angular velocity about the vehicle z axis [rad/s].
    void add_gyro(double t, double wz);

    /// A pose of the voted source, in its own (drifting) frame.
    void add_pose(double t, const mrpt::poses::CPose3D& pose);

    struct Verdict
    {
        bool   translation     = false;  //!< its distance disagrees with the wheels
        bool   rotation        = false;  //!< its heading disagrees with the gyroscope
        double pose_distance   = 0;  //!< [m] in the window
        double wheels_distance = 0;  //!< [m] in the window
        double yaw_difference  = 0;  //!< [rad] pose source minus gyroscope, in the window
    };

    /// The vote over (t - window, t]. No flag without data covering the whole window.
    [[nodiscard]] Verdict verdict(double t) const;

    /// Gyroscope bias in use [rad/s], if already estimated.
    [[nodiscard]] std::optional<double> gyro_bias() const;

   private:
    struct Step
    {
        double t    = 0;  //!< end of the increment
        double dist = 0;  //!< [m], or [s] for the gyroscope (its dt)
        double dyaw = 0;  //!< [rad], raw for the gyroscope
    };

    Parameters::OdometryVote p_;

    std::deque<Step> pose_;
    std::deque<Step> wheels_;
    std::deque<Step> gyro_;

    std::optional<std::pair<double, mrpt::poses::CPose3D>> lastPose_;
    std::optional<std::pair<double, mrpt::poses::CPose2D>> lastWheels_;
    std::optional<double>                                  lastGyroStamp_;

    /// Last time the wheels moved, for the stationary gyroscope bias.
    std::optional<double> lastWheelsMotion_;

    /// Gyroscope readings not yet confirmed as stationary by a newer wheel reading.
    std::deque<Step> pendingBias_;

    /// Exponentially forgotten sums of the stationary gyroscope readings.
    double biasSum_  = 0;
    double biasTime_ = 0;

    void prune(std::deque<Step>& q, double t) const;
};

}  // namespace mola::state_estimation_smoother
