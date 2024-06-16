/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 *
 * Copyright (C) 2018-2024 Jose Luis Blanco, University of Almeria
 * Licensed under the GNU GPL v3 for non-commercial applications.
 *
 * This file is part of MOLA.
 * MOLA is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * MOLA is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * MOLA. If not, see <https://www.gnu.org/licenses/>.
 * ------------------------------------------------------------------------- */
/**
 * @file   NavState.h
 * @brief  State vector for SE(3) pose + velocity
 * @author Jose Luis Blanco Claraco
 * @date   Jan 22, 2024
 */
#pragma once

#include <mrpt/math/TTwist3D.h>
#include <mrpt/poses/CPose3DPDFGaussianInf.h>

namespace mola
{
/** The state returned by NavStateFuse
 *
 * \ingroup mola_navstate_fuse_grp
 */
struct NavState
{
    NavState()  = default;
    ~NavState() = default;

    /** SE(3) pose estimation, including information matrix, given
     *  in the requested frame_id.
     */
    mrpt::poses::CPose3DPDFGaussianInf pose;

    /** Linear and angular velocity estimation, given in the local vehicle
     *  frame. */
    mrpt::math::TTwist3D twist;

    /** Inverse covariance matrix (information) of twist,
     *  with variable order in the matrix: [vx vy vz wx wy wz]
     */
    mrpt::math::CMatrixDouble66 twist_inv_cov;

    std::string asString() const;
};

}  // namespace mola
