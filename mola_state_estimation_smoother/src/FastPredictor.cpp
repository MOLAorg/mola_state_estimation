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
 * @file   FastPredictor.cpp
 * @brief  Lock-free, sub-ms pose predictor anchored on the latest backend
 *         snapshot.
 * @author Jose Luis Blanco Claraco
 */

#include "FastPredictor.h"

#include <mrpt/core/bits_math.h>  // mrpt::square
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPose3DPDFGaussianInf.h>
#include <mrpt/system/datetime.h>  // timeDifference

#include <cmath>

#include "extrapolation.h"

namespace mola::state_estimation_smoother
{

void FastPredictor::set_snapshot(std::shared_ptr<const Snapshot> snap)
{
    std::lock_guard<std::mutex> lck(mtx_);
    snapshot_ = std::move(snap);
}

std::shared_ptr<const Snapshot> FastPredictor::snapshot() const
{
    std::lock_guard<std::mutex> lck(mtx_);
    return snapshot_;
}

void FastPredictor::note_observation_stamp(const mrpt::Clock::time_point& obsStamp)
{
    std::lock_guard<std::mutex> lck(mtx_);
    // Keep the freshest observation stamp (inputs are usually monotonic), so a
    // late arrival never drags the published stamp backwards.
    if (!lastObsStamp_ || mrpt::system::timeDifference(*lastObsStamp_, obsStamp) > 0)
    {
        lastObsStamp_     = obsStamp;
        lastObsWallclock_ = mrpt::Clock::now();
    }
}

std::optional<mrpt::Clock::time_point> FastPredictor::get_current_extrapolated_stamp() const
{
    std::lock_guard<std::mutex> lck(mtx_);
    if (!lastObsStamp_)
    {
        return std::nullopt;
    }
    return mrpt::Clock::fromDouble(
        (mrpt::Clock::nowDouble() - mrpt::Clock::toDouble(lastObsWallclock_)) +
        mrpt::Clock::toDouble(*lastObsStamp_));
}

void FastPredictor::clear()
{
    std::lock_guard<std::mutex> lck(mtx_);
    snapshot_.reset();
    lastObsStamp_.reset();
}

std::optional<NavState> FastPredictor::predict(
    const mrpt::Clock::time_point& t_query, const std::string& frame_id,
    const Parameters& params) const
{
    std::shared_ptr<const Snapshot> snap;
    {
        std::lock_guard<std::mutex> lck(mtx_);
        snap = snapshot_;
    }
    if (!snap || !snap->valid)
    {
        return std::nullopt;
    }

    // dt from the anchor (newest solved keyframe) to the requested time.
    const double dt = mrpt::system::timeDifference(snap->anchorStamp, t_query);
    if (std::abs(dt) > params.max_time_to_use_velocity_model)
    {
        return std::nullopt;
    }

    // Start from the anchor state and grow the twist uncertainty by the
    // random-walk model over the extrapolation interval (mirrors the
    // synchronous estimated_navstate()).
    NavState ret = snap->anchor;

    // The covariance propagation below (matrix inversions, information-form
    // conversions and Gaussian pose composition) can throw on a non
    // positive-definite covariance; estimated_navstate() delegates here directly
    // in async mode, so absorb it and report "not ready yet" instead of letting
    // it terminate the caller's thread.
    try
    {
        // Anchor twist covariance (before random-walk growth), reused as the
        // current-velocity uncertainty of the pose increment below.
        const mrpt::math::CMatrixDouble66 anchorTwistCov = snap->anchor.twist_inv_cov.inverse_LLt();
        {
            auto twist_cov = anchorTwistCov;
            for (int i = 0; i < 3; i++)
            {
                twist_cov(0 + i, 0 + i) +=
                    mrpt::square(params.sigma_random_walk_acceleration_linear * dt);
                twist_cov(3 + i, 3 + i) +=
                    mrpt::square(params.sigma_random_walk_acceleration_angular * dt);
            }
            ret.twist_inv_cov = twist_cov.inverse_LLt();
        }

        // Reference ({map}) frame: extrapolate the anchor pose forward,
        // propagating covariance.
        if (frame_id == params.reference_frame_name)
        {
            mrpt::poses::CPose3DPDFGaussian anchorPose;
            anchorPose.copyFrom(ret.pose);
            ret.pose.copyFrom(
                extrapolate_pose_pdf(params, anchorPose, ret.twist, anchorTwistCov, dt));
            return ret;
        }

        // Odometry frame {odom_i}: resolve its numeric id.
        const auto& str2id = snap->frameNames.getDirectMap();
        const auto  itName = str2id.find(frame_id);
        if (itName == str2id.end())
        {
            return std::nullopt;
        }
        const auto requestedFrameIdx = itName->second;

        // Frame-local prediction: anchor on the source's OWN last raw pose in
        // {odom_i} and extrapolate by the body-twist increment, so the prediction
        // stays immune to {map} corrections (geo-ref / loop closure / per-solve
        // jitter). Falls back to the global conversion before the first raw pose.
        const auto itRaw = snap->lastRawPoseBySource.find(requestedFrameIdx);
        if (itRaw == snap->lastRawPoseBySource.end())
        {
            const auto itFrame = snap->frameTransforms.find(requestedFrameIdx);
            if (itFrame == snap->frameTransforms.end())
            {
                return std::nullopt;
            }
            mrpt::poses::CPose3DPDFGaussian anchorPose;
            anchorPose.copyFrom(ret.pose);
            const auto mapPred =
                extrapolate_pose_pdf(params, anchorPose, ret.twist, anchorTwistCov, dt);
            // Transform the {map}-frame prediction into {odom_i}: pred (-) T_frame_wrt_map.
            ret.pose.copyFrom(mapPred - itFrame->second);
            return ret;
        }

        const auto&  rawAnchor = itRaw->second;
        const double dtPred    = mrpt::system::timeDifference(rawAnchor.stamp, t_query);
        if (std::abs(dtPred) > params.max_time_to_use_velocity_model)
        {
            return std::nullopt;
        }

        ret.pose.copyFrom(
            extrapolate_pose_pdf(params, rawAnchor.pose, ret.twist, anchorTwistCov, dtPred));

        return ret;
    }
    catch (const std::exception&)
    {
        // Under-constrained covariance: report "not ready yet".
        return std::nullopt;
    }
}

}  // namespace mola::state_estimation_smoother
