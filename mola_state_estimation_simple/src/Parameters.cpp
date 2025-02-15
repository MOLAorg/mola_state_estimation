/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 *
 * Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria
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
 * @file   Parameters.cpp
 * @brief  Parameters for IMU preintegration.
 * @author Jose Luis Blanco Claraco
 * @date   Sep 19, 2021
 */

#include <mola_state_estimation_simple/Parameters.h>

namespace mola::state_estimation_simple
{

void Parameters::loadFrom(const mrpt::containers::yaml& cfg)
{
    MCP_LOAD_REQ(cfg, max_time_to_use_velocity_model);

    MCP_LOAD_OPT(cfg, sigma_random_walk_acceleration_linear);
    MCP_LOAD_OPT(cfg, sigma_random_walk_acceleration_angular);
    MCP_LOAD_OPT(cfg, enforce_planar_motion);

    if (cfg.has("initial_twist"))
    {
        ASSERT_(cfg["initial_twist"].isSequence() && cfg["initial_twist"].asSequence().size() == 6);

        auto&      tw  = initial_twist;
        const auto seq = cfg["initial_twist"].asSequenceRange();
        for (size_t i = 0; i < 6; i++) tw[i] = seq.at(i).as<double>();
    }
}

}  // namespace mola::state_estimation_simple
