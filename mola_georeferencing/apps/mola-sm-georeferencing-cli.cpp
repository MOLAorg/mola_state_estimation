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
#include <mp2p_icp/metricmap.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>

#include <CLI/CLI.hpp>
#include <algorithm>
#include <fstream>
#include <sstream>

// ---------------------------------------------------------------------------
// CLI flags
// ---------------------------------------------------------------------------
namespace
{
CLI::App cmd{"mola-sm-georeferencing-cli"};

std::string argInput = "map.simplemap";

std::string argWriteMMInto = "map.mm";
std::string argOutput      = "map.georef";

double       argHorz                        = 1.0;
double       argIMUGravitySigmaDeg          = 3.0;
double       argIMUAttitudeSigmaDeg         = 3.0;
double       argIMUAttitudeYawSigmaDeg      = 15.0;
double       argIMUAttitudeAzimuthOffsetDeg = 0.0;
double       argMinUncertaintyXYZ           = 0.20;
double       argGnssHuberK                  = 1.5;
unsigned int argMinFixQuality               = 0;
std::string  argPlugins;
bool         argNoIMUGravity  = false;
bool         argNoIMUAttitude = false;
std::string  argSetEnuToMapXYZ;
std::string  arg_verbosity_level = "INFO";

CLI::Option* optWriteMMInto;
CLI::Option* optOutput;
CLI::Option* optHorz;
CLI::Option* optIMUGravitySigmaDeg;
CLI::Option* optIMUAttitudeSigmaDeg;
CLI::Option* optIMUAttitudeYawSigmaDeg;
CLI::Option* optIMUAttitudeAzimuthOffsetDeg;
CLI::Option* optMinUncertaintyXYZ;
CLI::Option* optGnssHuberK;
CLI::Option* optMinFixQuality;
CLI::Option* optPlugins;
CLI::Option* optSetEnuToMapXYZ;

}  // namespace

static bool is_binary_georef(const std::string& fil)
{
    return mrpt::system::extractFileExtension(fil) == "georef";
}

void run_sm_georef()
{
    if (optPlugins->count() > 0)
    {
        std::string sErrs;
        bool        ok = mrpt::system::loadPluginModules(argPlugins, sErrs);
        if (!ok)
        {
            std::cerr << "Errors loading plugins: " << argPlugins << std::endl;
            throw std::runtime_error(sErrs.c_str());
        }
    }

    mrpt::system::COutputLogger logger;
    logger.setLoggerName("mola-sm-georeferencing-cli");
    logger.setVerbosityLevel(
        mrpt::typemeta::str2enum<mrpt::system::VerbosityLevel>(arg_verbosity_level));

    const auto& filSM = argInput;

    mrpt::maps::CSimpleMap sm;

    logger.logFmt(mrpt::system::LVL_INFO, "Reading simplemap from: '%s'...", filSM.c_str());

    sm.loadFromFile(filSM);

    logger.logFmt(mrpt::system::LVL_INFO, "Done read simplemap with %zu keyframes.", sm.size());

    ASSERT_(!sm.empty());

    mola::SMGeoReferencingParams p;
    p.logger = &logger;

    if (optHorz->count() > 0)
    {
        p.fgParams.addHorizontalityConstraints = true;
        p.fgParams.horizontalitySigmaZ         = argHorz;
    }
    if (optMinUncertaintyXYZ->count() > 0)
    {
        p.fgParams.minimumUncertaintyXYZ = argMinUncertaintyXYZ;
    }
    if (optGnssHuberK->count() > 0)
    {
        p.fgParams.robustParamHuberK = argGnssHuberK;
    }

    if (optMinFixQuality->count() > 0)
    {
        p.minimumGNSSFixQuality = argMinFixQuality;
    }

    if (argNoIMUGravity)
    {
        p.useIMUGravityAlignment = false;
    }
    if (optIMUGravitySigmaDeg->count() > 0)
    {
        p.imuGravityParams.imuGravitySigmaDeg = argIMUGravitySigmaDeg;
    }

    if (argNoIMUAttitude)
    {
        p.useIMUAttitudeAlignment = false;
    }
    if (optIMUAttitudeSigmaDeg->count() > 0)
    {
        p.imuAttitudeParams.imuAttitudeSigmaDeg = argIMUAttitudeSigmaDeg;
    }
    if (optIMUAttitudeYawSigmaDeg->count() > 0)
    {
        p.imuAttitudeParams.imuAttitudeYawSigmaDeg = argIMUAttitudeYawSigmaDeg;
    }
    if (optIMUAttitudeAzimuthOffsetDeg->count() > 0)
    {
        p.imuAttitudeParams.azimuthOffsetDeg = argIMUAttitudeAzimuthOffsetDeg;
    }

    mola::SMGeoReferencingOutput smGeo = mola::simplemap_georeference(sm, p);

    if (!smGeo.geo_ref.has_value())
    {
        std::cerr << "Georeferencing failed. No output will be generated.\n";
        return;
    }

    // Optional re-datuming: force T_enu_to_map to a user-defined translation,
    // moving the geodetic datum accordingly (see mola::recenter_georeference).
    if (optSetEnuToMapXYZ->count() > 0)
    {
        if (!smGeo.has_geodetic_datum)
        {
            THROW_EXCEPTION(
                "--set-t-enu-to-map-xyz requires a GNSS-derived geodetic datum, but "
                "the input simplemap only provided IMU data (no real geo_coord to "
                "re-datum).");
        }

        std::string s = argSetEnuToMapXYZ;
        std::replace(s.begin(), s.end(), ',', ' ');
        std::istringstream   iss(s);
        mrpt::math::TPoint3D xyz;
        if (!(iss >> xyz.x >> xyz.y >> xyz.z))
        {
            THROW_EXCEPTION_FMT(
                "Cannot parse --set-t-enu-to-map-xyz value as \"X,Y,Z\": '%s'",
                argSetEnuToMapXYZ.c_str());
        }
        iss >> std::ws;
        if (!iss.eof())
        {
            THROW_EXCEPTION_FMT(
                "Cannot parse --set-t-enu-to-map-xyz value as \"X,Y,Z\": '%s'",
                argSetEnuToMapXYZ.c_str());
        }

        smGeo.geo_ref = mola::recenter_georeference(*smGeo.geo_ref, xyz);

        logger.logFmt(
            mrpt::system::LVL_INFO,
            "Re-centered T_enu_to_map translation to (%.3f, %.3f, %.3f) and moved datum "
            "accordingly.",
            xyz.x, xyz.y, xyz.z);
    }

    const auto& geo_ref = smGeo.geo_ref.value();

    std::cout << "Obtained georeferencing:\n"
              << "lat: " << geo_ref.geo_coord.lat.getAsString() << "\n"
              << "lon: " << geo_ref.geo_coord.lon.getAsString() << "\n"
              << mrpt::format(
                     "lat_lon: %.06f, %.06f\n", geo_ref.geo_coord.lat.decimal_value,
                     geo_ref.geo_coord.lon.decimal_value)
              << "h: " << geo_ref.geo_coord.height << "\n"
              << "T_enu_to_map: " << geo_ref.T_enu_to_map.asString() << "\n";

    // Warn if the user has not requested any output at all.
    if (optWriteMMInto->count() == 0 && optOutput->count() == 0)
    {
        std::cerr
            << "[mola-sm-georeferencing-cli] WARNING: Georeferencing was computed successfully "
               "but no output destination was specified.\n"
               "  Use '--write-into <map.mm>' to inject it into a metric map, or\n"
               "  '-o <map.georef|map.yaml>' to save it to a standalone file.\n"
               "  The result will be discarded.\n";
    }

    if (optWriteMMInto->count() > 0)
    {
        mp2p_icp::metric_map_t mm;

        std::cout << "[mola-sm-georeferencing-cli] Loading mm map: '" << argWriteMMInto << "'..."
                  << std::endl;

        const bool loadOk = mm.load_from_file(argWriteMMInto);
        if (!loadOk)
        {
            THROW_EXCEPTION_FMT("Error loading input map file: '%s'", argWriteMMInto.c_str());
        }

        // overwrite metadata:
        mm.georeferencing = smGeo.geo_ref;

        // and save:
        const auto saved_ok = mm.save_to_file(argWriteMMInto);
        if (!saved_ok)
        {
            std::cerr << "Error saving modified .mm file: '" << argWriteMMInto << "'\n";
            return;
        }

        std::cout << "[mola-sm-georeferencing-cli] Writing modified mm map: '" << argWriteMMInto
                  << "'..." << std::endl;
    }

    if (optOutput->count() > 0)
    {
        const std::string outFil = argOutput;

        std::cout << "[mola-sm-georeferencing-cli] Writing georef data file: '" << outFil << "'..."
                  << std::endl;

        std::optional<mp2p_icp::metric_map_t::Georeferencing> g = smGeo.geo_ref;

        if (is_binary_georef(outFil))
        {
            // Binary gzip format (*.georef)
            mrpt::io::CFileGZOutputStream f(outFil);
            auto                          arch = mrpt::serialization::archiveFrom(f);
            arch << g;
        }
        else
        {
            // YAML format (*.yaml, *.yml, or any other extension)
            const auto    yamlData = mp2p_icp::ToYAML(g);
            std::ofstream of(outFil);
            if (!of.is_open())
            {
                THROW_EXCEPTION_FMT("Cannot open output file for writing: '%s'", outFil.c_str());
            }
            of << yamlData;
        }
    }
}

int main(int argc, char** argv)
{
    cmd.add_option("-i,--input", argInput, "Input .simplemap file")->required();

    optWriteMMInto = cmd.add_option(
        "--write-into", argWriteMMInto,
        "An existing .mm file in which to write the georeferencing metadata");

    optOutput = cmd.add_option(
        "-o,--output", argOutput,
        "Write the obtained georeferencing metadata to a file. The format is "
        "determined by the file extension: binary gzip (`*.georef`) or YAML "
        "(`*.yaml`, `*.yml`).");

    optHorz = cmd.add_option(
        "--horizontality-sigma", argHorz,
        "For short trajectories (not >10x the GPS uncertainty), this helps to "
        "avoid degeneracy.");

    optIMUGravitySigmaDeg = cmd.add_option(
        "--imu-gravity-sigma-deg", argIMUGravitySigmaDeg,
        "IMU gravity alignment uncertainty (degrees).");

    optIMUAttitudeSigmaDeg = cmd.add_option(
        "--imu-attitude-sigma-deg", argIMUAttitudeSigmaDeg,
        "IMU absolute-attitude roll/pitch uncertainty (degrees).");

    optIMUAttitudeYawSigmaDeg = cmd.add_option(
        "--imu-attitude-yaw-sigma-deg", argIMUAttitudeYawSigmaDeg,
        "IMU absolute-attitude yaw (azimuth) uncertainty (degrees). Defaults higher "
        "than the roll/pitch sigma since fused yaw (often magnetometer-based) is "
        "typically noisier.");

    optIMUAttitudeAzimuthOffsetDeg = cmd.add_option(
        "--imu-attitude-azimuth-offset-deg", argIMUAttitudeAzimuthOffsetDeg,
        "Fixed calibration offset (degrees) between the IMU's zero-yaw reading and "
        "true azimuth, e.g. due to sensor mounting or magnetic declination.");

    optMinUncertaintyXYZ = cmd.add_option(
        "--min-gnss-sigma", argMinUncertaintyXYZ,
        "Minimum per-axis GNSS uncertainty (meters) used as a floor for the "
        "ENU noise model. Default: 0.20");

    optGnssHuberK = cmd.add_option(
        "--gnss-huber-k", argGnssHuberK,
        "Parameter 'k' of the Huber robust kernel applied to GNSS position "
        "factors (whitened units). Default: 1.5");

    optMinFixQuality = cmd.add_option(
        "--min-gnss-fix-quality", argMinFixQuality,
        "If non-zero, discard GNSS frames whose NMEA GGA fix quality is below "
        "this value (1=GPS, 2=DGPS, 4=RTK fixed, 5=RTK float). Default: 0 "
        "(filter disabled).");

    optPlugins = cmd.add_option(
        "-l,--load-plugins", argPlugins,
        "One or more (comma separated) *.so files to load as plugins, e.g. "
        "defining new CMetricMap classes");

    cmd.add_flag(
        "--no-imu-gravity", argNoIMUGravity,
        "Disable using IMU acceleration data for gravity alignment "
        "(enabled by default).");

    cmd.add_flag(
        "--no-imu-attitude", argNoIMUAttitude,
        "Disable using IMU absolute-attitude (orientation) data for full attitude "
        "alignment, including azimuth (enabled by default).");

    optSetEnuToMapXYZ = cmd.add_option(
        "--set-t-enu-to-map-xyz", argSetEnuToMapXYZ,
        "Instead of keeping the GNSS-derived translation of T_enu_to_map (which "
        "may be several meters from the map origin), force it to the given "
        "\"X,Y,Z\" (meters, map frame) and move the geodetic datum accordingly so "
        "the map<->geodetic mapping is preserved. Use \"0,0,0\" to place the ENU "
        "origin at the map origin. The estimated rotation is kept.");

    cmd.add_option(
        "-v,--verbosity", arg_verbosity_level,
        "Verbosity level: ERROR|WARN|INFO|DEBUG (Default: INFO)");

    CLI11_PARSE(cmd, argc, argv);

    try
    {
        run_sm_georef();
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what();
        return 1;
    }
    return 0;
}
