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
 * @file   GyroHistory.h
 * @brief  Recent gyroscope readings, to predict the attitude change since a
 *         prediction anchor (Parameters::imu_propagation).
 * @author Jose Luis Blanco Claraco
 */
#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <deque>
#include <mutex>
#include <optional>
#include <regex>
#include <string>
#include <utility>

namespace mola::state_estimation_smoother
{
/** Recent angular velocity readings of one IMU, in the vehicle frame.
 *
 *  A prediction extrapolates its anchor pose (a solved keyframe, or a front
 *  end's last raw pose) to the query time. The rotation part of that increment
 *  is the integral of these readings, minus the gyroscope bias estimate, where
 *  they cover the interval; a constant angular velocity cannot follow a turn
 *  that reverses within it (e.g. a hand-held sensor swinging).
 *
 *  Thread-safe: written by fuse_imu() as readings arrive (before the async
 *  backend sees them) and read by both prediction paths.
 */
class GyroHistory
{
   public:
    /// Largest gap [s] between readings (or between a reading and an end of
    /// the interval) that the integration bridges.
    static constexpr double MAX_GAP = 0.05;

    explicit GyroHistory(double keepSeconds) : keepSeconds_(keepSeconds) {}

    /// Only readings whose sensor label matches this are used (default: any).
    void set_label_regex(const std::string& re)
    {
        std::lock_guard<std::mutex> lck(mtx_);
        labelRe_.emplace(re);
    }

    /** Adds one reading (time [s], angular velocity [rad/s] in the vehicle
     *  frame). Only the first matching label seen is used; readings not newer
     *  than the last one, or not finite, are ignored (a non-finite one leaves
     *  a gap, bridged up to MAX_GAP like any other). */
    void add(const std::string& label, double t, const Eigen::Vector3d& w)
    {
        std::lock_guard<std::mutex> lck(mtx_);
        if (!w.allFinite() || !std::isfinite(t) ||
            (labelRe_ && !std::regex_match(label, *labelRe_)))
        {
            return;
        }
        if (label_.empty())
        {
            label_ = label;
        }
        if (label != label_ || (!readings_.empty() && t <= readings_.back().first))
        {
            return;
        }
        readings_.emplace_back(t, w);
        while (!readings_.empty() && readings_.front().first < t - keepSeconds_)
        {
            readings_.pop_front();
        }
    }

    /// The sensor label in use (empty before the first matching reading).
    [[nodiscard]] std::string label() const
    {
        std::lock_guard<std::mutex> lck(mtx_);
        return label_;
    }

    /// Sets the gyroscope bias [rad/s] (vehicle frame) subtracted from the readings.
    void set_bias(const Eigen::Vector3d& b)
    {
        std::lock_guard<std::mutex> lck(mtx_);
        bias_ = b;
    }

    /// Forgets the readings and the bias.
    void clear()
    {
        std::lock_guard<std::mutex> lck(mtx_);
        readings_.clear();
        label_.clear();
        bias_.setZero();
    }

    /** The vehicle rotation from t0 to t1 [s] (R_{t0,t1}, in the frame at t0),
     *  or nothing if t1 <= t0 or the readings do not cover the interval. A
     *  reading inside the interval holds from the previous one's stamp (or t0)
     *  to its own. After the last reading inside, or over an interval with
     *  none inside, the newest reading not after that point holds up to t1: a
     *  reading stamped after t1 is never used, so the result does not depend
     *  on whether it has been received yet. */
    [[nodiscard]] std::optional<Eigen::Matrix3d> rotation_between(double t0, double t1) const
    {
        if (t1 <= t0)
        {
            return std::nullopt;
        }
        std::lock_guard<std::mutex> lck(mtx_);

        // First reading after t0, and the end of the ones not after t1:
        auto itBegin = readings_.begin();
        while (itBegin != readings_.end() && itBegin->first <= t0)
        {
            ++itBegin;
        }
        auto itEnd = itBegin;
        while (itEnd != readings_.end() && itEnd->first <= t1)
        {
            ++itEnd;
        }

        Eigen::Matrix3d R    = Eigen::Matrix3d::Identity();
        const auto      step = [&](const Eigen::Vector3d& w, double dt)
        {
            const Eigen::Vector3d phi   = (w - bias_) * dt;
            const double          angle = phi.norm();
            if (angle > 0)
            {
                R = R * Eigen::AngleAxisd(angle, phi / angle).toRotationMatrix();
            }
        };

        if (itBegin == itEnd)
        {
            // No reading inside the interval: hold the newest one before it.
            if (itBegin == readings_.begin() || t1 - std::prev(itBegin)->first > MAX_GAP)
            {
                return std::nullopt;
            }
            step(std::prev(itBegin)->second, t1 - t0);
            return R;
        }
        if (itBegin->first - t0 > MAX_GAP || t1 - std::prev(itEnd)->first > MAX_GAP)
        {
            return std::nullopt;
        }
        double tPrev = t0;
        for (auto it = itBegin; it != itEnd; ++it)
        {
            if (it->first - tPrev > MAX_GAP)
            {
                return std::nullopt;
            }
            step(it->second, it->first - tPrev);
            tPrev = it->first;
        }
        step(std::prev(itEnd)->second, t1 - tPrev);
        return R;
    }

   private:
    const double                                   keepSeconds_;
    mutable std::mutex                             mtx_;
    std::optional<std::regex>                      labelRe_;
    std::string                                    label_;
    std::deque<std::pair<double, Eigen::Vector3d>> readings_;  //!< oldest first
    Eigen::Vector3d                                bias_ = Eigen::Vector3d::Zero();
};

}  // namespace mola::state_estimation_smoother
