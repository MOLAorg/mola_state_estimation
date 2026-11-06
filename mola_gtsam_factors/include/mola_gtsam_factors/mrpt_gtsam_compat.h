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
 * @file   mrpt_gtsam_compat.h
 * @brief  SE(3) pose covariance conversions between MRPT and GTSAM that are
 *         exact to first order with any MRPT 3.x version.
 * @author Jose Luis Blanco Claraco
 * @date   Nov 2026
 */
#pragma once

#include <gtsam/geometry/Pose3.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/gtsam_wrappers.h>
#include <mrpt/version.h>

#include <cmath>

namespace mola::gtsam_compat
{
#if MRPT_VERSION >= 0x030602

using mrpt::gtsam_wrappers::to_gtsam_se3_cov6;
using mrpt::gtsam_wrappers::to_mrpt_se3_pdf;

#else

// MRPT <= 3.6.1 has no to_mrpt_se3_pdf(), and its to_gtsam_se3_cov6() does not
// return the covariance in GTSAM local coordinates. These are the
// implementations of newer MRPT versions: remove them once one is required.

/** Jacobian of the GTSAM local coordinates (rx,ry,rz,tx,ty,tz) of
 * xi = Logmap(mean^-1 * pose) with respect to MRPT's (x,y,z,yaw,pitch,roll),
 * evaluated at the mean.
 */
inline gtsam::Matrix6 jacobian_gtsam_local_wrt_ypr(const mrpt::poses::CPose3D& mean)
{
    const double   cp = std::cos(mean.pitch());
    const double   sp = std::sin(mean.pitch());
    const double   cr = std::cos(mean.roll());
    const double   sr = std::sin(mean.roll());
    gtsam::Matrix6 J  = gtsam::Matrix6::Zero();
    // clang-format off
    J.block<3, 3>(0, 3) <<
        -sp,      0,   1,
        cp * sr,  cr,  0,
        cp * cr, -sr,  0;
    // clang-format on
    J.block<3, 3>(3, 0) = mean.getRotationMatrix().asEigen().transpose();
    return J;
}

/** Converts an MRPT Gaussian SE(3) pose into a GTSAM Pose3 and its covariance
 * in GTSAM local coordinates (rx,ry,rz,tx,ty,tz). First-order conversion.
 */
inline void to_gtsam_se3_cov6(
    const mrpt::poses::CPose3DPDFGaussian& in, gtsam::Pose3& pose_out, gtsam::Matrix6& cov_out)
{
    pose_out = gtsam::Pose3(
        gtsam::Rot3(in.mean.getRotationMatrix().asEigen()),
        gtsam::Point3(in.mean.x(), in.mean.y(), in.mean.z()));
    const gtsam::Matrix6 J = jacobian_gtsam_local_wrt_ypr(in.mean);
    cov_out                = J * in.cov.asEigen() * J.transpose();
}

/** Inverse of to_gtsam_se3_cov6(). First-order conversion.
 * \exception std::exception At pitch = +-90 deg, where (yaw,pitch,roll) is
 * singular.
 */
inline mrpt::poses::CPose3DPDFGaussian to_mrpt_se3_pdf(
    const gtsam::Pose3& pose, const gtsam::Matrix6& cov)
{
    mrpt::poses::CPose3DPDFGaussian out;
    out.mean = mrpt::poses::CPose3D(mrpt::math::CMatrixDouble44(pose.matrix()));
    ASSERTMSG_(
        std::abs(std::cos(out.mean.pitch())) > 1e-12,
        "to_mrpt_se3_pdf(): (yaw,pitch,roll) is singular at pitch = +-90 deg");
    const gtsam::Matrix6 Ji = jacobian_gtsam_local_wrt_ypr(out.mean).inverse();
    out.cov.asEigen()       = Ji * cov * Ji.transpose();
    return out;
}

#endif

}  // namespace mola::gtsam_compat
