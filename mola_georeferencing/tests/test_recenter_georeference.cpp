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

#include <mola_georeferencing/simplemap_georeference.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/topography/conversions.h>

#include <cmath>
#include <iostream>

namespace
{
void expect(bool cond, const std::string& msg)
{
    if (!cond)
    {
        throw std::runtime_error("Test assertion failed: " + msg);
    }
}

// Absolute geodetic coordinates of a given map-frame point, using a georef.
// This is the operation that must stay invariant under recenter_georeference().
//
// NOTE: we deliberately use the ellipsoid-normal ENU convention
// (ENU_axes_from_WGS84), which is the one used by geodeticToENU_WGS84 and hence
// by both the georeferencing pipeline and the state estimator's fuse_gnss().
// (mrpt::topography::ENUToGeocentric uses a *geocentric-radial* Up axis instead,
// which differs by ~0.2 deg and would introduce a spurious cm-level bias here.)
mrpt::topography::TGeodeticCoords geodetic_of_map_point(
    const mp2p_icp::metric_map_t::Georeferencing& g, const mrpt::math::TPoint3D& p_map)
{
    // p_enu = T_enu_to_map^{-1}(p_map), relative to the datum:
    mrpt::math::TPoint3D p_enu;
    g.T_enu_to_map.mean.inverseComposePoint(p_map.x, p_map.y, p_map.z, p_enu.x, p_enu.y, p_enu.z);

    // ENU -> geocentric via the datum's ENU frame (ellipsoid-normal Up):
    mrpt::math::TPose3D enuPose;
    mrpt::topography::ENU_axes_from_WGS84(g.geo_coord, enuPose, /*only_angles=*/false);
    mrpt::math::TPoint3D geocentric;
    mrpt::poses::CPose3D(enuPose).composePoint(p_enu.x, p_enu.y, p_enu.z, geocentric.x, geocentric.y, geocentric.z);

    // geocentric -> geodetic:
    mrpt::topography::TGeodeticCoords geo;
    mrpt::topography::geocentricToGeodetic(
        geocentric, geo, mrpt::topography::TEllipsoid::Ellipsoid_WGS84());
    return geo;
}

// Builds a non-trivial georef: datum somewhere real, T_enu_to_map with a
// non-identity rotation and a several-meters translation (mimicking the
// GNSS-derived output that this feature is meant to re-datum).
mp2p_icp::metric_map_t::Georeferencing make_georef()
{
    mp2p_icp::metric_map_t::Georeferencing g;
    g.geo_coord.lat    = 56.108686;  // deg
    g.geo_coord.lon    = 10.125739;  // deg
    g.geo_coord.height = 100.0;      // m

    // yaw=30deg, pitch=5deg, roll=-3deg; translation several meters:
    g.T_enu_to_map.mean = mrpt::poses::CPose3D(
        7.0, -4.0, 2.0, mrpt::DEG2RAD(30.0), mrpt::DEG2RAD(5.0), mrpt::DEG2RAD(-3.0));
    g.T_enu_to_map.cov.setIdentity();
    return g;
}

// Compares two geodetic coordinates within tolerances.
void expect_geodetic_near(
    const mrpt::topography::TGeodeticCoords& a, const mrpt::topography::TGeodeticCoords& b,
    const std::string& ctx)
{
    // 1e-7 deg ~ 1.1 cm; comfortably above WGS84 geodetic<->geocentric
    // round-trip precision (sub-mm), yet far tighter than any GNSS accuracy.
    expect(std::abs(a.lat - b.lat) < 1e-7, ctx + ": latitude mismatch");
    expect(std::abs(a.lon - b.lon) < 1e-7, ctx + ": longitude mismatch");
    expect(std::abs(a.height - b.height) < 1e-2, ctx + ": height mismatch");
}

// Core property: re-datuming preserves the map<->geodetic mapping exactly, and
// forces T_enu_to_map to the requested translation.
void test_recenter_preserves_mapping(const mrpt::math::TPoint3D& target)
{
    const auto g0 = make_georef();

    // Sample several map points and record their absolute geodetic coords:
    const std::vector<mrpt::math::TPoint3D> probes = {
        {0, 0, 0}, {10, 20, 3}, {-15, 8, -2}, {100, -50, 12}};

    std::vector<mrpt::topography::TGeodeticCoords> before;
    for (const auto& p : probes) before.push_back(geodetic_of_map_point(g0, p));

    const auto g1 = mola::recenter_georeference(g0, target);

    // (1) T_enu_to_map translation is the requested one (to WGS84 round-trip
    // precision, i.e. sub-mm):
    const auto t1 = g1.T_enu_to_map.mean.translation();
    expect(std::abs(t1.x - target.x) < 1e-3, "translation X set");
    expect(std::abs(t1.y - target.y) < 1e-3, "translation Y set");
    expect(std::abs(t1.z - target.z) < 1e-3, "translation Z set");

    // (2) rotation is NOT exactly preserved: the local ENU axes rotate as the
    // datum moves, so T_enu_to_map's rotation changes by that small amount. It
    // must stay close for a modest datum move, but need not be identical. Just
    // sanity-check it is a proper rotation close to the original (bounded).
    const auto R0 = g0.T_enu_to_map.mean.getRotationMatrix();
    const auto R1 = g1.T_enu_to_map.mean.getRotationMatrix();
    expect((R0 - R1).array().abs().maxCoeff() < 0.2, "rotation stays bounded near original");

    // (3) the map<->geodetic mapping is invariant for every probe (this is the
    // key exactness property, and it holds regardless of the datum move size):
    for (size_t i = 0; i < probes.size(); i++)
    {
        const auto after = geodetic_of_map_point(g1, probes[i]);
        expect_geodetic_near(before[i], after, "probe " + std::to_string(i));
    }

    // (4) the new datum equals the geodetic of the map point at `target`:
    expect_geodetic_near(
        g1.geo_coord, geodetic_of_map_point(g0, target), "datum == geodetic(target)");
}

// Placing the ENU origin at the map origin (0,0,0) makes the datum equal to the
// geodetic coordinates of the map origin under the original georef.
void test_recenter_to_map_origin()
{
    const auto g0 = make_georef();
    const auto g1 = mola::recenter_georeference(g0, {0, 0, 0});

    const auto t1 = g1.T_enu_to_map.mean.translation();
    expect(t1.norm() < 1e-3, "map-origin case: T translation is zero");

    expect_geodetic_near(
        g1.geo_coord, geodetic_of_map_point(g0, {0, 0, 0}), "datum == geodetic(map origin)");
}

}  // namespace

int main()
{
    try
    {
        test_recenter_preserves_mapping({0, 0, 0});
        test_recenter_preserves_mapping({100, -50, 1});
        test_recenter_preserves_mapping({-1234.5, 6789.0, -30.0});
        test_recenter_to_map_origin();

        std::cout << "All recenter_georeference tests passed!\n";
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return 1;
    }
}
