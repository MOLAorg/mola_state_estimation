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
 * @file   test-imu-bias-provider.cpp
 * @brief  Unit test: the smoother tracks the gyroscope bias of each IMU at
 *         standstills (imu_bias_tracking) and offers it as a mola::ImuBiasSource;
 *         a slow turn in place, which the IMU alone takes as a bias, is vetoed by
 *         the motion seen by a (late) LiDAR odometry input.
 * @author Jose Luis Blanco Claraco
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>

#if defined(MOLA_SMOOTHER_HAS_IMU_BIAS_SOURCE)

#include <mola_kernel/MinimalModuleContainer.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/core/get_env.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/random/RandomGenerators.h>

#include <cmath>
#include <deque>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace mola
{
/** A module that uses the IMU biases, as a LiDAR odometry would. */
class MockBiasConsumer : public ExecutableBase
{
    DEFINE_MRPT_OBJECT(MockBiasConsumer, mola)
   public:
    void initialize(const mrpt::containers::yaml&) override {}
    void spinOnce() override {}
};
}  // namespace mola
IMPLEMENTS_MRPT_OBJECT(MockBiasConsumer, mola::ExecutableBase, mola)

namespace
{
const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

constexpr double IMU_RATE      = 100.0;  // [Hz]
constexpr double LIO_RATE      = 10.0;  // [Hz]
constexpr double LIO_LATENCY   = 0.2;  // [s] poses are fused this late
constexpr double GYRO_NOISE    = 0.0035;  // [rad/s] per reading, motors on
constexpr double ACC_NOISE     = 0.03;  // [m/s^2]
constexpr double TURN_RATE     = 0.0052;  // [rad/s] (0.3 deg/s): slow turn in place
constexpr double T_FIRST_STOP  = 5.0;  // [s]
constexpr double T_TURN        = 20.0;  // [s]
constexpr double T_SECOND_STOP = 5.0;  // [s]
constexpr double T0            = 1.8e9;  // a realistic epoch [s]

// Biases like those of a MEMS IMU integrated in a LiDAR [rad/s]:
const mrpt::math::TVector3D BIAS{0.0004, -0.0051, -0.0035};

std::string make_config(bool tracking)
{
    return std::string(R"###(
params:
    vehicle_frame_name: "base_link"
    reference_frame_name: "map"
    kinematic_model: KinematicModel::ConstantVelocity
    sliding_window_length: 3.0
    relative_pose_increment_sigma_lin: 0.02
    relative_pose_increment_sigma_ang: 0.005
    max_time_to_use_velocity_model: 2.0
    min_time_difference_to_create_new_frame: 0.05
    sigma_random_walk_acceleration_linear: 1.0
    sigma_random_walk_acceleration_angular: 1.0
    sigma_integrator_position: 0.10
    sigma_integrator_orientation: 0.10
    sigma_twist_from_consecutive_poses_linear: 1.0
    sigma_twist_from_consecutive_poses_angular: 1.0
    estimate_geo_reference: false
    async_backend: false
    imu_normalized_gravity_alignment_sigma: 0
    imu_angular_velocity_sigma: 0.1
    imu_bias_labels_re: "lidar_imu"
    imu_bias_tracker:
      max_hint_latency: 1.0
    imu_bias_tracking: )###") +
           (tracking ? "true" : "false") + "\n";
}

struct Result
{
    std::optional<mola::ImuBias> bias;  //!< final estimate, via the interface
    std::vector<double>          update_times;  //!< advertised updates [s from start]
    std::vector<std::string>     update_labels;
    bool                         untracked_known = false;
};

/// Simulates the run; `withLio`: the LiDAR odometry poses are fused, in `lioFrame`.
Result run(bool tracking, bool withLio, const std::string& lioFrame = "lidar_odom")
{
    auto& rng = mrpt::random::getRandomGenerator();
    rng.randomize(3);

    auto est = std::make_shared<mola::state_estimation_smoother::StateEstimationSmoother>();
    est->initialize(mrpt::containers::yaml::FromText(make_config(tracking)));
    auto                         consumer = std::make_shared<mola::MockBiasConsumer>();
    mola::MinimalModuleContainer container({est, consumer});

    Result     r;
    const auto found = consumer->findService<mola::ImuBiasSource>();
    ASSERT_EQUAL_(found.size(), 1u);
    auto source = std::dynamic_pointer_cast<mola::ImuBiasSource>(found.front());
    ASSERT_(source);
    source->subscribe_to_imu_bias_updates(
        [&](const mola::ImuBias& b)
        {
            r.update_labels.push_back(b.sensor_label);
            r.update_times.push_back(mrpt::Clock::toDouble(*b.last_update) - T0);
        });

    {
        auto cov = mrpt::math::CMatrixDouble66::Identity();
        cov *= 1e-6;
        est->fuse_pose(
            mrpt::Clock::fromDouble(T0),
            mrpt::poses::CPose3DPDFGaussian(mrpt::poses::CPose3D::Identity(), cov), "map");
    }

    const double tEnd = T_FIRST_STOP + T_TURN + T_SECOND_STOP;
    std::deque<std::pair<double, mrpt::poses::CPose3D>> lioQueue;
    double                                              yaw  = 0;
    const int                                           nImu = static_cast<int>(tEnd * IMU_RATE);
    for (int i = 0; i < nImu; i++)
    {
        const double t       = i / IMU_RATE;
        const bool   turning = t >= T_FIRST_STOP && t < T_FIRST_STOP + T_TURN;
        const double wz      = turning ? TURN_RATE : 0.0;
        yaw += wz / IMU_RATE;

        for (const char* label : {"lidar_imu", "other_imu"})
        {
            auto o         = mrpt::obs::CObservationIMU::Create();
            o->sensorLabel = label;
            o->timestamp   = mrpt::Clock::fromDouble(T0 + t);
            o->set(mrpt::obs::IMU_WX, BIAS.x + rng.drawGaussian1D(0, GYRO_NOISE));
            o->set(mrpt::obs::IMU_WY, BIAS.y + rng.drawGaussian1D(0, GYRO_NOISE));
            o->set(mrpt::obs::IMU_WZ, wz + BIAS.z + rng.drawGaussian1D(0, GYRO_NOISE));
            o->set(mrpt::obs::IMU_X_ACC, rng.drawGaussian1D(0, ACC_NOISE));
            o->set(mrpt::obs::IMU_Y_ACC, rng.drawGaussian1D(0, ACC_NOISE));
            o->set(mrpt::obs::IMU_Z_ACC, 9.81 + rng.drawGaussian1D(0, ACC_NOISE));
            est->onNewObservation(o);
        }
        est->spinOnce();  // advertises the updates

        // LiDAR odometry: 10 Hz, with the jitter of a real one at rest (5 mm, 0.02 deg
        // per scan, all axes), fused late.
        if (withLio && i % static_cast<int>(IMU_RATE / LIO_RATE) == 0)
        {
            const double sa = mrpt::DEG2RAD(0.02);
            lioQueue.emplace_back(
                t, mrpt::poses::CPose3D::FromXYZYawPitchRoll(
                       rng.drawGaussian1D(0, 0.005), rng.drawGaussian1D(0, 0.005),
                       rng.drawGaussian1D(0, 0.005), yaw + rng.drawGaussian1D(0, sa),
                       rng.drawGaussian1D(0, sa), rng.drawGaussian1D(0, sa)));
        }
        while (!lioQueue.empty() && t - lioQueue.front().first >= LIO_LATENCY)
        {
            mrpt::poses::CPose3DPDFGaussian p;
            p.mean = lioQueue.front().second;
            p.cov.setIdentity();
            p.cov *= 1e-4;
            est->fuse_pose(mrpt::Clock::fromDouble(T0 + lioQueue.front().first), p, lioFrame);
            lioQueue.pop_front();
        }
    }

    r.bias            = source->imu_bias("lidar_imu");
    r.untracked_known = source->imu_bias("other_imu").has_value();
    if (tracking)
    {
        // Queried at an exact later time:
        const auto tq = mrpt::Clock::fromDouble(T0 + tEnd + 10.0) + mrpt::Clock::duration(1);
        const auto bq = source->imu_bias("lidar_imu", tq);
        ASSERT_(bq && bq->timestamp == tq);
        ASSERT_GT_(bq->gyro_cov(2, 2), r.bias->gyro_cov(2, 2));
    }
    return r;
}

void test_disabled()
{
    const auto r = run(false, true);
    ASSERT_(!r.bias.has_value());
    ASSERT_(r.update_times.empty());
}

void check_lio_vetoes_slow_turn(const std::string& lioFrame)
{
    const auto r = run(true, true, lioFrame);
    for (const auto& l : r.update_labels)
    {
        ASSERT_EQUAL_(l, std::string("lidar_imu"));
    }
    ASSERT_(r.bias && r.bias->last_update);
    ASSERT_(!r.untracked_known);  // not in imu_bias_labels_re

    const double err = (r.bias->gyro - BIAS).norm();
    if (VERBOSE)
    {
        std::cout << "with LIO in '" << lioFrame << "': " << r.update_times.size()
                  << " updates, error " << err << " rad/s, bias " << r.bias->gyro.asString()
                  << "\n";
    }
    ASSERT_LT_(err, 4e-4);
    // Updates only at the two stops, none during the turn (LIO saw it):
    ASSERT_GE_(r.update_times.size(), 6u);
    for (const double t : r.update_times)
    {
        const bool inTurn = t > T_FIRST_STOP + 0.5 && t < T_FIRST_STOP + T_TURN;
        ASSERT_(!inTurn);
    }
}

// LIO odometry (relative poses), and LIO localizing in a map (absolute poses):
void test_lio_vetoes_slow_turn()
{
    check_lio_vetoes_slow_turn("lidar_odom");
    check_lio_vetoes_slow_turn("map");
}

void test_without_lio_the_turn_is_taken_as_bias()
{
    // Documented limitation, checked so the test above passes for the right
    // reason: without motion hints, the slow turn ends up in the z bias.
    const auto r = run(true, false);
    ASSERT_(r.bias);
    std::size_t inTurn = 0;
    for (const double t : r.update_times)
    {
        inTurn += (t > T_FIRST_STOP + 1.5 && t < T_FIRST_STOP + T_TURN) ? 1 : 0;
    }
    if (VERBOSE)
    {
        std::cout << "without LIO: " << inTurn << " updates during the turn\n";
    }
    ASSERT_GT_(inTurn, 10u);
}

}  // namespace

int main()
{
    try
    {
        test_disabled();
        test_lio_vetoes_slow_turn();
        test_without_lio_the_turn_is_taken_as_bias();
        std::cout << "test-imu-bias-provider: all passed\n";
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}

#else
int main()
{
    std::cout << "test-imu-bias-provider: skipped (no ImuBiasSource in this build)\n";
    return 0;
}
#endif
