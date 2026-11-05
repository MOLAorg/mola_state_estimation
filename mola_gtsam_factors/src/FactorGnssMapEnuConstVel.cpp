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
 * @file   FactorGnssMapEnuConstVel.cpp
 * @brief  GNSS factor for a fix taken at a time offset from its keyframe
 * @author Jose Luis Blanco Claraco
 * @date   Nov 5, 2026
 */

#include <mola_gtsam_factors/FactorGnssMapEnuConstVel.h>

namespace mola::factors
{

FactorGnssMapEnuConstVel::FactorGnssMapEnuConstVel() = default;

}