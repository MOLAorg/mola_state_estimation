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
 * @file   test-newest-last-fixed-lag-smoother.cpp
 * @brief  NewestLastFixedLagSmoother keeps the newest variables at the root of
 *         the Bayes tree, and gives the same estimates as GTSAM's smoother.
 * @author Jose Luis Blanco Claraco
 * @date   Nov 8, 2026
 */

#include <mrpt/core/exceptions.h>

#include <iostream>

#include "../src/NewestLastFixedLagSmoother.h"

#if GTSAM_USES_BOOST

#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <algorithm>
#include <memory>
#include <random>
#include <vector>

using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;
using mola::state_estimation_smoother::NewestLastFixedLagSmoother;

namespace
{
constexpr double       LAG              = 5.0;
constexpr unsigned int NUM_RANDOM_SEEDS = 40;

gtsam::ISAM2Params isam2_params()
{
    gtsam::ISAM2Params params;
    params.findUnusedFactorSlots = true;
    return params;
}

size_t depth_of(const gtsam::ISAM2& isam, const gtsam::Key key)
{
    size_t depth = 0;
    for (auto clique = isam[key]; clique->parent(); clique = clique->parent())
    {
        ++depth;
    }
    return depth;
}

// Exposes the constraint groups that an update would use.
class ConstraintsProbe : public NewestLastFixedLagSmoother
{
   public:
    using NewestLastFixedLagSmoother::NewestLastFixedLagSmoother;

    int max_group(const gtsam::KeySet& newFactorKeys, const double stamp) const
    {
        const auto groups =
            orderingConstraints(findKeysBefore(stamp - smootherLag_), newFactorKeys, stamp);
        int maxGroup = 0;
        for (const auto& [key, group] : *groups)
        {
            maxGroup = std::max(maxGroup, group);
        }
        return maxGroup;
    }
};

void expect_same_estimates(const gtsam::Values& a, const gtsam::Values& b, const double tol)
{
    ASSERT_EQUAL_(a.size(), b.size());
    for (const auto& kv : a)
    {
        ASSERT_(b.exists(kv.key));
        const gtsam::Pose2 pa = a.at<gtsam::Pose2>(kv.key);
        const gtsam::Pose2 pb = b.at<gtsam::Pose2>(kv.key);
        ASSERT_(pa.equals(pb, tol));
    }
}

// Two variables per step, linked to each other and to the previous step: each
// update's new factors involve four keys, so the newest keys get their own group.
void test_newest_at_root()
{
    const auto                         noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
    gtsam::IncrementalFixedLagSmoother base(LAG, isam2_params());
    NewestLastFixedLagSmoother         newest(LAG, isam2_params(), true);

    size_t maxBaseDepth = 0;
    for (size_t i = 0; i < 20; ++i)
    {
        gtsam::NonlinearFactorGraph              factors;
        gtsam::Values                            values;
        gtsam::FixedLagSmoother::KeyTimestampMap stamps;
        const gtsam::Pose2                       guess(static_cast<double>(i), 0.0, 0.0);
        values.insert(X(i), guess);
        values.insert(V(i), guess);
        stamps[X(i)] = static_cast<double>(i);
        stamps[V(i)] = static_cast<double>(i);
        factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2>>(
            X(i), V(i), gtsam::Pose2(0.0, 0.1, 0.0), noise);
        if (i == 0)
        {
            factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose2>>(X(0), gtsam::Pose2(), noise);
        }
        else
        {
            const gtsam::Pose2 step(1.0, 0.0, 0.05);
            factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2>>(X(i - 1), X(i), step, noise);
            factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2>>(V(i - 1), V(i), step, noise);
            factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2>>(
                X(i - 1), V(i), step * gtsam::Pose2(0.0, 0.1, 0.0), noise);
        }
        base.update(factors, values, stamps);
        newest.update(factors, values, stamps);
        ASSERT_EQUAL_(depth_of(newest.getISAM2(), X(i)), 0U);

        base.update();
        newest.update();
        ASSERT_EQUAL_(depth_of(newest.getISAM2(), X(i)), 0U);
        ASSERT_EQUAL_(depth_of(newest.getISAM2(), V(i)), 0U);
        maxBaseDepth = std::max(maxBaseDepth, depth_of(base.getISAM2(), X(i)));

        expect_same_estimates(base.calculateEstimate(), newest.calculateEstimate(), 1e-6);
    }
    std::cout << "[newest-at-root] max depth of the newest key with GTSAM's ordering: "
              << maxBaseDepth << ", with newest keys last: 0\n";
}

// A chain: the new factors of every update involve two keys, too few for a
// third constraint group, while each update from the seventh on marginalizes
// one key.
void test_few_new_keys_while_marginalizing()
{
    const auto                         noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
    gtsam::IncrementalFixedLagSmoother base(LAG, isam2_params());
    ConstraintsProbe                   newest(LAG, isam2_params(), true);

    for (size_t i = 0; i < 30; ++i)
    {
        gtsam::NonlinearFactorGraph              factors;
        gtsam::Values                            values;
        gtsam::FixedLagSmoother::KeyTimestampMap stamps;
        values.insert(X(i), gtsam::Pose2(static_cast<double>(i), 0.0, 0.0));
        stamps[X(i)] = static_cast<double>(i);
        if (i > LAG)
        {
            // An update that marginalizes: a third group needs three new-factor keys.
            const double stamp = static_cast<double>(i);
            ASSERT_EQUAL_(newest.max_group({X(i - 1), X(i)}, stamp), 1);
            ASSERT_EQUAL_(newest.max_group({X(i - 2), X(i - 1), X(i)}, stamp), 2);
        }
        if (i == 0)
        {
            factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose2>>(X(0), gtsam::Pose2(), noise);
        }
        else
        {
            factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2>>(
                X(i - 1), X(i), gtsam::Pose2(1.0, 0.0, 0.02), noise);
        }
        base.update(factors, values, stamps);
        newest.update(factors, values, stamps);
        base.update();
        newest.update();
        expect_same_estimates(base.calculateEstimate(), newest.calculateEstimate(), 1e-6);
    }
    std::cout << "[few-new-keys] 30 updates, same estimates as GTSAM's smoother\n";
}
// Whether every key of the Bayes tree still has a value: a marginalized key
// left in a clique breaks the next update.
bool tree_is_consistent(const gtsam::ISAM2& isam)
{
    const gtsam::Values& theta = isam.getLinearizationPoint();
    for (const auto& [key, clique] : isam.nodes())
    {
        for (const gtsam::Key k : clique->conditional()->keys())
        {
            if (!theta.exists(k))
            {
                return false;
            }
        }
    }
    return true;
}

// Random windows: new variables linked to one or two variables still in the
// window, some loop closures, repeated timestamps and extra empty updates.
void test_marginalization_on_random_graphs()
{
    const auto noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
    for (unsigned int seed = 0; seed < NUM_RANDOM_SEEDS; ++seed)
    {
        std::mt19937       rng(seed);
        gtsam::ISAM2Params params      = isam2_params();
        params.relinearizeThreshold    = 0.001;
        params.relinearizeSkip         = 1;
        const double               lag = 1.0 + static_cast<double>(rng() % 5);
        NewestLastFixedLagSmoother smoother(lag, params, true);

        size_t nextKey = 0;
        double stamp   = 0.0;
        for (int step = 0; step < 60; ++step)
        {
            gtsam::NonlinearFactorGraph              factors;
            gtsam::Values                            values;
            gtsam::FixedLagSmoother::KeyTimestampMap stamps;
            if (rng() % 4 != 0)
            {
                stamp += 0.5 * static_cast<double>(1 + rng() % 3);
            }
            std::vector<size_t> window;
            for (const auto& [key, keyStamp] : smoother.timestamps())
            {
                if (keyStamp > stamp - lag)
                {
                    window.push_back(gtsam::Symbol(key).index());
                }
            }
            const size_t numNew = step == 0 ? 1 : rng() % 3;
            for (size_t n = 0; n < numNew; ++n)
            {
                const size_t key = nextKey++;
                values.insert(X(key), gtsam::Pose2(static_cast<double>(key), 0.0, 0.0));
                stamps[X(key)] = stamp;
                if (window.empty())
                {
                    factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose2>>(
                        X(key), gtsam::Pose2(static_cast<double>(key), 0.0, 0.0), noise);
                }
                const size_t numLinks = window.empty() ? 0 : 1 + rng() % 2;
                for (size_t l = 0; l < numLinks; ++l)
                {
                    const size_t other = window[rng() % window.size()];
                    factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2>>(
                        X(other), X(key),
                        gtsam::Pose2(
                            static_cast<double>(key) - static_cast<double>(other), 0.0, 0.0),
                        noise);
                }
                window.push_back(key);
            }
            if (window.size() >= 2 && rng() % 3 == 0)
            {
                const size_t a = window[rng() % window.size()];
                const size_t b = window[rng() % window.size()];
                if (a != b)
                {
                    factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2>>(
                        X(a), X(b),
                        gtsam::Pose2(static_cast<double>(b) - static_cast<double>(a), 0.0, 0.0),
                        noise);
                }
            }
            smoother.update(factors, values, stamps);
            ASSERT_(tree_is_consistent(smoother.getISAM2()));
            const unsigned int numExtra = rng() % 3;
            for (unsigned int e = 0; e < numExtra; ++e)
            {
                smoother.update();
                ASSERT_(tree_is_consistent(smoother.getISAM2()));
            }
        }
    }
    std::cout << "[random-graphs] " << NUM_RANDOM_SEEDS
              << " random windows marginalized without leftovers\n";
}
}  // namespace

int main()
{
    try
    {
        test_newest_at_root();
        test_few_new_keys_while_marginalizing();
        test_marginalization_on_random_graphs();
        std::cout << "Test passed." << std::endl;
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed:\n" << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}

#else

int main()
{
    // The class only changes the ordering of GTSAM 4.2.
    std::cout << "Test skipped: only for GTSAM 4.2." << std::endl;
    return 0;
}

#endif
