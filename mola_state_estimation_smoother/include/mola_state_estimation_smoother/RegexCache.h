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

// TODO: Remove from this package once  mola_kernel 2.5.0 is out.

/**
 * @file   RegexCache.h
 * @brief  Cached regular expression
 * @author Jose Luis Blanco Claraco
 * @date   Jan 29, 2026
 */
#pragma once

#include <mutex>
#include <optional>
#include <regex>
#include <string>

namespace mola
{

/** Holds a reference to a cached regex. Recompiles only if the expression changed.
 *  Thread-safe: multiple sensor threads may call get_regex() concurrently.
 */
class RegexCache
{
   public:
    // Returns a copy of the cached regex, recompiling only if the expression changed.
    std::regex get_regex(const std::string& regExpression)
    {
        std::lock_guard<std::mutex> lck(mutex_);
        if (!cachedRegex_ || regExpression != cachedExpression_)
        {
            cachedExpression_ = regExpression;
            cachedRegex_.emplace(cachedExpression_, std::regex::ECMAScript);
        }
        return *cachedRegex_;
    }

   private:
    mutable std::mutex        mutex_;
    std::string               cachedExpression_;
    std::optional<std::regex> cachedRegex_;
};

}  // namespace mola