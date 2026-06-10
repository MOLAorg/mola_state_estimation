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
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>

#include <iostream>

namespace
{
// Builds a CSimpleMap with one GNSS observation per keyframe, at the given
// geodetic coordinates and with an isotropic ENU sigma (via covariance_enu).
mrpt::maps::CSimpleMap make_sm_with_gnss(
    const std::vector<mrpt::topography::TGeodeticCoords>& coords, double sigma_m)
{
    mrpt::maps::CSimpleMap sm;

    for (size_t i = 0; i < coords.size(); i++)
    {
        auto pose_pdf  = mrpt::poses::CPose3DPDFGaussian::Create();
        pose_pdf->mean = mrpt::poses::CPose3D(static_cast<double>(i), 0, 0, 0, 0, 0);

        auto sf  = mrpt::obs::CSensoryFrame::Create();
        auto obs = mrpt::obs::CObservationGPS::Create();

        mrpt::obs::gnss::Message_NMEA_GGA gga;
        gga.fields.latitude_degrees  = coords[i].lat;
        gga.fields.longitude_degrees = coords[i].lon;
        gga.fields.altitude_meters   = coords[i].height;
        gga.fields.fix_quality       = 4;  // RTK fixed
        gga.fields.thereis_HDOP      = true;
        gga.fields.HDOP              = 1.0f;
        obs->setMsg(gga);

        // Provide an explicit ENU covariance so the sigma is deterministic:
        mrpt::math::CMatrixDouble33 cov;
        cov.setDiagonal(sigma_m * sigma_m);
        obs->covariance_enu = cov;

        sf->insert(obs);
        sm.insert(pose_pdf, sf);
    }

    return sm;
}

void expect(bool cond, const std::string& msg)
{
    if (!cond)
    {
        throw std::runtime_error("Test assertion failed: " + msg);
    }
}

// A cluster of GNSS fixes spanning only ~0.2 m with a 0.5 m sigma -> degenerate.
void test_degenerate_cluster()
{
    std::cout << "[test] degenerate cluster...\n";

    const double                                   baseLat = 37.0;
    const double                                   baseLon = -2.0;
    const double                                   baseH   = 100.0;
    std::vector<mrpt::topography::TGeodeticCoords> coords;

    // ~1e-6 deg latitude ~= 0.11 m. Keep all points within a tiny box:
    for (int i = 0; i < 5; i++)
    {
        coords.emplace_back(baseLat + i * 1e-6, baseLon + i * 1e-6, baseH + i * 0.02);
    }

    const auto sm     = make_sm_with_gnss(coords, /*sigma_m=*/0.5);
    const auto frames = mola::extract_gnss_frames_from_sm(sm);

    expect(frames.frames.size() == 5, "all 5 GNSS frames should be extracted");
    expect(frames.possibly_degenerate, "tiny spatial spread must be flagged as degenerate");
}

// GNSS fixes spread over ~100 m with a 0.5 m sigma -> NOT degenerate.
void test_non_degenerate_spread()
{
    std::cout << "[test] non-degenerate spread...\n";

    const double                                   baseLat = 37.0;
    const double                                   baseLon = -2.0;
    const double                                   baseH   = 100.0;
    std::vector<mrpt::topography::TGeodeticCoords> coords;

    // ~1e-4 deg latitude ~= 11 m per step -> ~44 m diagonal over 5 frames:
    for (int i = 0; i < 5; i++)
    {
        coords.emplace_back(baseLat + i * 1e-4, baseLon + i * 1e-4, baseH);
    }

    const auto sm     = make_sm_with_gnss(coords, /*sigma_m=*/0.5);
    const auto frames = mola::extract_gnss_frames_from_sm(sm);

    expect(frames.frames.size() == 5, "all 5 GNSS frames should be extracted");
    expect(!frames.possibly_degenerate, "a wide spatial spread must NOT be flagged as degenerate");
}
}  // namespace

int main()
{
    try
    {
        test_degenerate_cluster();
        test_non_degenerate_spread();
        std::cout << "\n[Success] GNSS degeneracy detection tests passed!\n";
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "\n[Test Failed] " << e.what() << std::endl;
        return 1;
    }
}
