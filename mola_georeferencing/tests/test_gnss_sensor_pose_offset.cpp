/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
 Closed-source licenses available upon request, for this package
 alone or in combination with the complete SLAM system.
*/

// Verifies that CObservationGPS::sensorPose (the antenna lever arm, in the
// vehicle frame) is actually taken into account by simplemap_georeference().
//
// The keyframe vehicle poses move along a (mildly climbing) straight path but
// their yaw varies at each one, so a fixed-in-vehicle antenna offset traces a
// KF-dependent arc in the map frame. This makes the offset impossible to
// absorb into a constant translation of T_enu_to_map: if the code ignored (or
// mishandled) `sensorPose`, the fitted rigid transform would be forced to
// explain the resulting noncolinear target points with a colinear,
// offset-free source point set, which would not recover the ground truth.

#include <mola_georeferencing/simplemap_georeference.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/topography/conversions.h>

#include <cmath>
#include <iostream>
#include <vector>

namespace
{
void expect(bool cond, const std::string& msg)
{
    if (!cond)
    {
        throw std::runtime_error("Test assertion failed: " + msg);
    }
}

mrpt::topography::TGeodeticCoords geodetic_from_enu(
    const mrpt::math::TPoint3D& enuPoint, const mrpt::topography::TGeodeticCoords& origin)
{
    mrpt::topography::TGeocentricCoords geocentric;
    mrpt::topography::ENUToGeocentric(
        enuPoint, origin, geocentric, mrpt::topography::TEllipsoid::Ellipsoid_WGS84());

    mrpt::topography::TGeodeticCoords geodetic;
    mrpt::topography::geocentricToGeodetic(
        geocentric, geodetic, mrpt::topography::TEllipsoid::Ellipsoid_WGS84());

    return geodetic;
}

// Builds a CSimpleMap with one GNSS observation per keyframe. Each keyframe's
// vehicle pose lies on the map X-axis (Y=Z=0) but has a distinct yaw, and each
// GNSS observation carries `antennaOnVehicle` as its `sensorPose` lever arm.
// The GGA lat/lon/height is computed so that the antenna's true ENU position
// (ground-truth T_enu_to_map, composed with the vehicle pose and the antenna
// offset) is reported exactly.
mrpt::maps::CSimpleMap make_sm_with_gnss_offset(
    const mrpt::poses::CPose3D& T_enu_to_map, const mrpt::math::TPoint3D& antennaOnVehicle,
    const mrpt::topography::TGeodeticCoords& origin)
{
    mrpt::maps::CSimpleMap sm;

    const mrpt::poses::CPose3D T_veh_to_antenna(
        antennaOnVehicle.x, antennaOnVehicle.y, antennaOnVehicle.z, 0, 0, 0);

    const int    numKFs  = 5;
    const double stepX   = 3.0;  // [m] between consecutive KFs, along map X-axis
    const double stepZ   = 0.7;  // [m] mild climb, to avoid a coplanar (rotation-ambiguous) path
    const double stepYaw = 30.0;  // [deg] yaw increment between consecutive KFs

    for (int i = 0; i < numKFs; i++)
    {
        const mrpt::poses::CPose3D T_map_to_veh(
            i * stepX, 0, i * stepZ, mrpt::DEG2RAD(i * stepYaw), 0, 0);

        auto pose_pdf  = mrpt::poses::CPose3DPDFGaussian::Create();
        pose_pdf->mean = T_map_to_veh;

        const mrpt::poses::CPose3D T_enu_to_antenna =
            T_enu_to_map + T_map_to_veh + T_veh_to_antenna;
        const mrpt::math::TPoint3D antennaENU = T_enu_to_antenna.translation();

        const auto coords = geodetic_from_enu(antennaENU, origin);

        auto sf  = mrpt::obs::CSensoryFrame::Create();
        auto obs = mrpt::obs::CObservationGPS::Create();

        mrpt::obs::gnss::Message_NMEA_GGA gga;
        gga.fields.latitude_degrees  = coords.lat.getDecimalValue();
        gga.fields.longitude_degrees = coords.lon.getDecimalValue();
        gga.fields.altitude_meters   = coords.height;
        gga.fields.fix_quality       = 4;  // RTK fixed
        gga.fields.thereis_HDOP      = true;
        gga.fields.HDOP              = 1.0f;
        obs->setMsg(gga);

        obs->sensorPose = T_veh_to_antenna;

        // Small, deterministic ENU uncertainty:
        mrpt::math::CMatrixDouble33 cov;
        cov.setDiagonal(0.05 * 0.05);
        obs->covariance_enu = cov;

        sf->insert(obs);
        sm.insert(pose_pdf, sf);
    }

    return sm;
}

void run_single_test(const mrpt::math::TPoint3D& antennaOnVehicle)
{
    std::cout << "==================================================\n";
    std::cout << "[Test] GNSS antenna (sensorPose) offset: " << antennaOnVehicle << "\n";

    // Ground truth ENU-to-map transformation (translation + yaw). Kept of the
    // same order of magnitude as the KF path and antenna offset so that the LM
    // optimizer (seeded at identity) converges reliably regardless of the
    // antenna offset direction:
    const mrpt::poses::CPose3D T_enu_to_map(10.0, -6.0, 2.0, mrpt::DEG2RAD(25.0), 0, 0);

    const mrpt::topography::TGeodeticCoords origin(37.0, -2.0, 100.0);

    const auto sm = make_sm_with_gnss_offset(T_enu_to_map, antennaOnVehicle, origin);

    mola::SMGeoReferencingParams params;
    // Anchor the ENU frame at the same datum used to build the synthetic
    // GGA data (otherwise it defaults to the first GNSS fix, which would
    // make T_est incomparable to the ground truth T_enu_to_map):
    params.geodeticReference = origin;
    // No IMU data in this synthetic map: rely on GNSS alone.
    const auto out = mola::simplemap_georeference(sm, params);

    expect(out.geo_ref.has_value(), "Georeferencing output should not be empty!");

    const auto& T_est = out.geo_ref->T_enu_to_map.mean;

    const double pos_error = (T_enu_to_map.translation() - T_est.translation()).norm();
    const double yaw_error_deg =
        std::abs(mrpt::RAD2DEG(mrpt::math::wrapToPi(T_est.yaw() - T_enu_to_map.yaw())));

    std::cout << "  Estimated T_enu_to_map: " << T_est << "\n";
    std::cout << "  Position error: " << pos_error << " m\n";
    std::cout << "  Yaw error:      " << yaw_error_deg << " deg\n";

    const double tol_pos_m   = 0.05;
    const double tol_yaw_deg = 0.1;

    expect(pos_error < tol_pos_m, "recovered T_enu_to_map translation must match ground truth");
    expect(yaw_error_deg < tol_yaw_deg, "recovered T_enu_to_map yaw must match ground truth");
}
}  // namespace

int main()
{
    try
    {
        // Antenna lever arms, all with a 2 m offset along X and/or Y:
        const std::vector<mrpt::math::TPoint3D> offsets = {
            {2.0, 0.0, 0.0},
            {0.0, 2.0, 0.0},
            {2.0, 2.0, 0.0},
            {-2.0, 2.0, 0.0},
        };

        std::size_t failed_count = 0;

        for (const auto& offset : offsets)
        {
            try
            {
                run_single_test(offset);
            }
            catch (const std::exception& e)
            {
                failed_count++;
                std::cerr << "\n[Test Failed] Exception in test case (offset: " << offset << "):\n"
                          << e.what() << std::endl;
            }
        }

        if (failed_count > 0)
        {
            std::cerr << "\n[Summary] " << failed_count << " out of " << offsets.size()
                      << " tests failed." << std::endl;
            return 1;
        }

        std::cout << "\n[Success] All GNSS sensorPose offset tests passed!" << std::endl;
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "\n[Test Failed] Exception caught:\n" << e.what() << std::endl;
        return 1;
    }
}
