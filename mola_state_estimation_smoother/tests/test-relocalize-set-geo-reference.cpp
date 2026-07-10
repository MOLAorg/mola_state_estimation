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
 * @file   test-relocalize-set-geo-reference.cpp
 * @brief  Regression test for the "relocalize" front-end integration path
 *         (a georeferenced map is already loaded; the geo-reference is
 *         pushed in at runtime via set_geo_reference(), not via YAML at
 *         load time). Covers three bugs fixed together (see
 *         mola_state_estimation's AGENTS.md "Relocalize mode" section):
 *
 *         1. set_geo_reference() being an unimplemented no-op (T_enu_to_map
 *            left effectively unconstrained, a GTSAM gauge freedom that
 *            eventually throws IndeterminantLinearSystemException once
 *            enough attitude factors accumulate) -- exercised here by fusing
 *            readings both before and after calling set_geo_reference(), and
 *            checking current_georeferencing() reflects the pushed value.
 *         2. estimated_navstate() not catching that exception (crashing the
 *            whole calling thread instead of returning std::nullopt like its
 *            signature promises) -- exercised by fusing enough
 *            pre-set_geo_reference() readings to reach the same
 *            under-constrained state and confirming no exception escapes.
 *         3. has_converged_localization() requiring estimate_geo_reference=
 *            true unconditionally (impossible to satisfy in relocalize mode
 *            by definition) -- exercised directly by asserting it returns
 *            false (not throws) before set_geo_reference(), matching the
 *            mode-aware logic's relocalize branch.
 *
 *         None of the other tests in this directory call set_geo_reference()
 *         at runtime, so none of them covered this path before. This test
 *         does not assert eventual GNSS+IMU convergence (has_converged_
 *         localization() -> true): with a synthetically *perfectly static*
 *         vehicle and independent per-reading IID noise (no correlated
 *         integrator signal, unlike a real sensor stream), the
 *         ConstantVelocity kinematic model's angular-velocity variable is
 *         numerically singular (confirmed via a *different* GTSAM
 *         IndeterminantLinearSystemException, on a W(k) symbol) -- a
 *         separate, pre-existing numerical-conditioning question from the
 *         three bugs above, out of scope here. End-to-end convergence
 *         accuracy for this exact code path is validated instead against
 *         real, correlated GNSS+IMU noise in MOLA + MVSIM simulation
 *         (mola-georef-init's Phase 5, 5/5 repeated runs + 2 adversarial
 *         variants, 5/5 each -- see that project's PLAN.md).
 */

#include <mola_kernel/interfaces/NavStateFilter.h>
#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/core/get_env.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/topography/conversions.h>

#include <iostream>
#include <tuple>

using namespace std::string_literals;
using namespace mrpt::literals;

// This whole test exercises StateEstimationSmoother::set_geo_reference(),
// which is only compiled in when the installed mola_kernel provides the
// (optional) NavStateFilter::set_geo_reference()/get_geo_reference() virtual
// methods -- see the same guard in StateEstimationSmoother.h/.cpp. Older
// mola_kernel releases (pre-dating that interface addition) lack it, so skip
// this test entirely rather than fail to compile against them.
#if defined(MOLA_KERNEL_NAVSTATE_FILTER_HAS_GEO_REFERENCE)

namespace
{

constexpr double GNSS_NOISE_XY_M = 0.10;
constexpr double GNSS_NOISE_Z_M  = 0.15;
constexpr double IMU_NOISE_QUAT  = 0.01;

const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

const size_t numReadingsBeforeGeoRef = 5;  // fused with NO geo-reference yet
const size_t numReadingsAfterGeoRef  = 30;
const double T                       = 0.2;  // sensor period (5 Hz)

auto& rng = mrpt::random::getRandomGenerator();

const char* navStateParams =
    R"###(# Config for Parameters
params:
    vehicle_frame_name: "base_link"
    reference_frame_name: "map"
    max_time_to_use_velocity_model: 2.0
    kinematic_model: KinematicModel::ConstantVelocity
    sliding_window_length: 8.0
    min_time_difference_to_create_new_frame: 0.05
    sigma_random_walk_acceleration_linear: 0.1
    sigma_random_walk_acceleration_angular: 0.1
    sigma_integrator_position: 0.02
    sigma_integrator_orientation: 0.02

    # No fixed_geo_reference here on purpose: it must arrive later, at
    # runtime, via set_geo_reference() -- as mola_lidar_odometry does once
    # it finishes loading a geo-referenced map.
    estimate_geo_reference: false
)###";

using Pose = mrpt::poses::CPose3D;

void fuse_one_gnss_imu_reading(
    mola::state_estimation_smoother::StateEstimationSmoother& stateEst, double t,
    const mrpt::topography::TGeodeticCoords& vehicleGeodetic, const Pose& actualImuPoseGlobal)
{
    // GNSS:
    {
        mrpt::obs::CObservationGPS obsGps;
        obsGps.timestamp   = mrpt::Clock::fromDouble(t);
        obsGps.sensorLabel = "gnss";

        constexpr double gnss_noise_deg = GNSS_NOISE_XY_M * mrpt::RAD2DEG(1.0 / 6300e3);

        mrpt::obs::gnss::Message_NMEA_GGA gga_msg;
        gga_msg.fields.latitude_degrees =
            vehicleGeodetic.lat + rng.drawGaussian1D(0, gnss_noise_deg);
        gga_msg.fields.longitude_degrees =
            vehicleGeodetic.lon + rng.drawGaussian1D(0, gnss_noise_deg);
        gga_msg.fields.altitude_meters =
            vehicleGeodetic.height + rng.drawGaussian1D(0, GNSS_NOISE_Z_M);
        gga_msg.fields.fix_quality = 4;  // RTK fixed
        obsGps.setMsg(gga_msg);

        auto& cov = obsGps.covariance_enu.emplace();
        cov.setIdentity();
        cov(0, 0) = cov(1, 1) = mrpt::square(GNSS_NOISE_XY_M);
        cov(2, 2)             = mrpt::square(GNSS_NOISE_Z_M);

        stateEst.fuse_gnss(obsGps);
    }

    // IMU:
    {
        mrpt::obs::CObservationIMU obsImu;
        obsImu.timestamp   = mrpt::Clock::fromDouble(t);
        obsImu.sensorLabel = "imu";

        mrpt::math::CQuaternionDouble gtQuatGlobal;
        actualImuPoseGlobal.getAsQuaternion(gtQuatGlobal);

        mrpt::math::CQuaternionDouble noisyQuat;
        for (int k = 0; k < 4; k++)
        {
            noisyQuat[k] = gtQuatGlobal[k] + rng.drawGaussian1D(0, IMU_NOISE_QUAT);
        }
        noisyQuat.normalize();

        obsImu.set(mrpt::obs::IMU_ORI_QUAT_W, noisyQuat.w());
        obsImu.set(mrpt::obs::IMU_ORI_QUAT_X, noisyQuat.x());
        obsImu.set(mrpt::obs::IMU_ORI_QUAT_Y, noisyQuat.y());
        obsImu.set(mrpt::obs::IMU_ORI_QUAT_Z, noisyQuat.z());

        const auto localUp = actualImuPoseGlobal.inverseRotateVector({0, 0, 9.81});
        obsImu.set(mrpt::obs::IMU_X_ACC, localUp.x + rng.drawGaussian1D(0, 0.1));
        obsImu.set(mrpt::obs::IMU_Y_ACC, localUp.y + rng.drawGaussian1D(0, 0.1));
        obsImu.set(mrpt::obs::IMU_Z_ACC, localUp.z + rng.drawGaussian1D(0, 0.1));

        obsImu.set(mrpt::obs::IMU_WX, rng.drawGaussian1D(0, 0.01));
        obsImu.set(mrpt::obs::IMU_WY, rng.drawGaussian1D(0, 0.01));
        obsImu.set(mrpt::obs::IMU_WZ, rng.drawGaussian1D(0, 0.01));

        stateEst.onNewObservation(std::make_shared<const mrpt::obs::CObservationIMU>(obsImu));
    }
}

void run_test()
{
    mola::state_estimation_smoother::StateEstimationSmoother stateEst;
    if (VERBOSE)
    {
        stateEst.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    }

    {
        auto cfgYaml = mrpt::containers::yaml::FromText(navStateParams);
        stateEst.initialize(cfgYaml);
    }

    const auto actualVehiclePose =
        Pose::FromXYZYawPitchRoll(5.0, -3.0, 0.2, 30.0_deg, 0.5_deg, -0.3_deg);
    const auto vehicleToAzimuth =
        mrpt::poses::CPose3D::FromYawPitchRoll(mrpt::DEG2RAD(-90.0), 0.0_deg, 0.0_deg);
    const auto actualImuPoseGlobal = actualVehiclePose + vehicleToAzimuth;

    mrpt::topography::TGeodeticCoords geoReference;
    geoReference.lat    = 40.1234;
    geoReference.lon    = -74.5678;
    geoReference.height = 100.0;

    mrpt::topography::TGeocentricCoords vehicleGeocentric;
    mrpt::topography::ENUToGeocentric(
        {actualVehiclePose.x(), actualVehiclePose.y(), actualVehiclePose.z()}, geoReference,
        vehicleGeocentric, mrpt::topography::TEllipsoid::Ellipsoid_WGS84());
    mrpt::topography::TGeodeticCoords vehicleGeodetic;
    mrpt::topography::geocentricToGeodetic(vehicleGeocentric, vehicleGeodetic);

    double t = 0;

    // --- Phase A: fuse a few readings with NO geo-reference set yet -----
    // Before the fix, this alone was enough to eventually leave
    // symbol_T_enu_to_map's rotation an unconstrained GTSAM gauge freedom;
    // has_converged_localization() must gracefully report "not converged",
    // never throw.
    for (size_t i = 0; i < numReadingsBeforeGeoRef; i++, t += T)
    {
        fuse_one_gnss_imu_reading(stateEst, t, vehicleGeodetic, actualImuPoseGlobal);

        // In production, mola_lidar_odometry's periodic spinOnce() (via
        // estimated_navstate()) flushes pending GTSAM factors/values ahead
        // of any has_converged_localization() poll; replicate that here so
        // this test exercises the same calling pattern. Two calls: ISAM2
        // needs the *following* update() to fold a brand-new variable into
        // a queryable BayesTree clique, so the very latest keyframe isn't
        // reliably queryable after only one flush.
        std::ignore = stateEst.estimated_navstate(mrpt::Clock::fromDouble(t), "map");
        std::ignore = stateEst.estimated_navstate(mrpt::Clock::fromDouble(t), "map");

        mrpt::poses::CPose3DPDFGaussian pose;
        const bool                      converged = stateEst.has_converged_localization(pose);
        ASSERT_(!converged);
    }

    // --- Phase B: push the geo-reference in at runtime, as a localization
    // front-end (mola_lidar_odometry) does once it finishes loading a
    // geo-referenced map -------------------------------------------------
    mola::Georeferencing georef;
    georef.geo_coord         = geoReference;
    georef.T_enu_to_map.mean = mrpt::poses::CPose3D::Identity();
    georef.T_enu_to_map.cov.setZero();
    stateEst.set_geo_reference(georef);

    // set_geo_reference() must have actually taken effect (bug 1's core
    // defect: it used to be a silent no-op).
    const auto storedGeoref = stateEst.current_georeferencing();
    ASSERT_(storedGeoref.has_value());
    ASSERT_(storedGeoref->geo_coord.lat == geoReference.lat);
    ASSERT_(storedGeoref->geo_coord.lon == geoReference.lon);

    // --- Phase C: fuse more readings past set_geo_reference() and confirm
    // no exception ever escapes (bug 2) and has_converged_localization()
    // stays well-defined (never throws) throughout, whatever it returns
    // (bug 3's mode-aware logic path; see the file header for why this test
    // doesn't additionally assert it eventually becomes true) -------------
    for (size_t i = 0; i < numReadingsAfterGeoRef; i++, t += T)
    {
        fuse_one_gnss_imu_reading(stateEst, t, vehicleGeodetic, actualImuPoseGlobal);

        std::ignore = stateEst.estimated_navstate(mrpt::Clock::fromDouble(t), "map");

        mrpt::poses::CPose3DPDFGaussian pose;
        std::ignore = stateEst.has_converged_localization(pose);
    }

    if (VERBOSE)
    {
        std::cout << "Ground truth: " << actualVehiclePose << "\n";
    }
}

}  // namespace

int main()
{
    try
    {
        run_test();
        std::cout << "\n✅ SUCCESS\n";
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "\n❌ FAILED:\n" << e.what() << std::endl;
        return 1;
    }
}

#else

int main()
{
    std::cout << "\nSKIPPED: installed mola_kernel lacks "
                 "NavStateFilter::set_geo_reference()/get_geo_reference()\n";
    return 0;
}

#endif
