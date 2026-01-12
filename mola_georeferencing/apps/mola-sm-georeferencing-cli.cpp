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
#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>

// CLI flags:

struct Cli
{
    TCLAP::CmdLine cmd{"mola-sm-georeferencing-cli"};

    TCLAP::ValueArg<std::string> argInput{
        "i", "input", "Input .simplemap file", true, "map.simplemap", "map.simplemap", cmd};

    TCLAP::ValueArg<std::string> argWriteMMInto{
        "",    "write-into", "An existing .mm file in which to write the georeferencing metadata",
        false, "map.mm",     "map.mm",
        cmd};

    TCLAP::ValueArg<std::string> argOutput{
        "o",   "output",     "Write the obtained georeferencing metadata to a .georef file",
        false, "map.georef", "map.georef",
        cmd};

    TCLAP::ValueArg<double> argHorz{
        "",
        "horizontality-sigma",
        "For short trajectories (not >10x the GPS uncertainty), this helps to "
        "avoid degeneracy.",
        false,
        1.0,
        "1.0",
        cmd};

    TCLAP::ValueArg<std::string> argPlugins{
        "l",
        "load-plugins",
        "One or more (comma separated) *.so files to load as plugins, e.g. "
        "defining new CMetricMap classes",
        false,
        "foobar.so",
        "foobar.so",
        cmd};

    TCLAP::ValueArg<std::string> arg_verbosity_level{
        "v",   "verbosity", "Verbosity level: ERROR|WARN|INFO|DEBUG (Default: INFO)",
        false, "INFO",      "INFO",
        cmd};
};

void run_sm_georef(Cli& cli)
{
    if (cli.argPlugins.isSet())
    {
        std::string sErrs;
        bool        ok = mrpt::system::loadPluginModules(cli.argPlugins.getValue(), sErrs);
        if (!ok)
        {
            std::cerr << "Errors loading plugins: " << cli.argPlugins.getValue() << std::endl;
            throw std::runtime_error(sErrs.c_str());
        }
    }

    mrpt::system::COutputLogger logger;
    logger.setLoggerName("mola-sm-georeferencing-cli");
    logger.setVerbosityLevel(
        mrpt::typemeta::str2enum<mrpt::system::VerbosityLevel>(cli.arg_verbosity_level.getValue()));

    const auto& filSM = cli.argInput.getValue();

    mrpt::maps::CSimpleMap sm;

    logger.logFmt(mrpt::system::LVL_INFO, "Reading simplemap from: '%s'...", filSM.c_str());

    sm.loadFromFile(filSM);

    logger.logFmt(mrpt::system::LVL_INFO, "Done read simplemap with %zu keyframes.", sm.size());

    ASSERT_(!sm.empty());

    mola::SMGeoReferencingParams p;
    p.logger = &logger;

    if (cli.argHorz.isSet())
    {
        p.fgParams.addHorizontalityConstraints = true;
        p.fgParams.horizontalitySigmaZ         = cli.argHorz.getValue();
    }
    // TODO: p.fgParams.minimumUncertaintyXYZ = xxx;

    const mola::SMGeoReferencingOutput smGeo = mola::simplemap_georeference(sm, p);

    std::cout << "Obtained georeferencing:\n"
              << "lat: " << smGeo.geo_ref.geo_coord.lat.getAsString() << "\n"
              << "lon: " << smGeo.geo_ref.geo_coord.lon.getAsString() << "\n"
              << mrpt::format(
                     "lat_lon: %.06f, %.06f\n", smGeo.geo_ref.geo_coord.lat.decimal_value,
                     smGeo.geo_ref.geo_coord.lon.decimal_value)
              << "h: " << smGeo.geo_ref.geo_coord.height << "\n"
              << "T_enu_to_map: " << smGeo.geo_ref.T_enu_to_map.asString() << "\n";

    if (cli.argWriteMMInto.isSet())
    {
        mp2p_icp::metric_map_t mm;

        std::cout << "[mola-sm-georeferencing-cli] Loading mm map: '"
                  << cli.argWriteMMInto.getValue() << "'..." << std::endl;

        mm.load_from_file(cli.argWriteMMInto.getValue());

        // overwrite metadata:
        mm.georeferencing = smGeo.geo_ref;

        // and save:
        mm.save_to_file(cli.argWriteMMInto.getValue());

        std::cout << "[mola-sm-georeferencing-cli] Writing modified mm map: '"
                  << cli.argWriteMMInto.getValue() << "'..." << std::endl;
    }

    if (cli.argOutput.isSet())
    {
        const std::string outFil = cli.argOutput.getValue();

        std::cout << "[mola-sm-georeferencing-cli] Writing georef data file: '" << outFil << "'..."
                  << std::endl;

        mrpt::io::CFileGZOutputStream                         f(outFil);
        std::optional<mp2p_icp::metric_map_t::Georeferencing> g;
        g = smGeo.geo_ref;

        auto arch = mrpt::serialization::archiveFrom(f);
        arch << g;
    }
}

int main(int argc, char** argv)
{
    try
    {
        Cli cli;

        // Parse arguments:
        if (!cli.cmd.parse(argc, argv))
        {
            return 1;  // should exit.
        }

        run_sm_georef(cli);
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what();
        return 1;
    }
    return 0;
}
