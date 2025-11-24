# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                              (MOLA)
#
# Copyright (C) 2018-2025, Jose Luis Blanco-Claraco
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file
# ------------------------------------------------------------------------------

# Minimum CMake version: limited by CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS
cmake_minimum_required(VERSION 3.5)

if("$ENV{ROS_VERSION}" STREQUAL "2")
  set(DETECTED_ROS2 TRUE)
endif()

# Tell CMake we'll use C++ for use in its tests/flags
project(mola_gtsam_factors LANGUAGES CXX)

# MOLA CMake scripts: "mola_xxx()"
find_package(mola_common REQUIRED)
find_package(GTSAM REQUIRED)
find_package(mrpt-poses REQUIRED)

# -----------------------
# define lib:
# -----------------------
mola_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES
    src/register.cpp
    include/mola_gtsam_factors/gtsam_detect_version.h
    include/mola_gtsam_factors/FactorTrapezoidalIntegrator.h
    include/mola_gtsam_factors/FactorConstLocalVelocity.h
    include/mola_gtsam_factors/FactorConstVelKinematics.h
    include/mola_gtsam_factors/FactorAngularVelocityIntegration.h
    include/mola_gtsam_factors/FactorTricycleKinematics.h
    include/mola_gtsam_factors/id.h
    include/mola_gtsam_factors/FactorGNSS2ENU.h
  PUBLIC_LINK_LIBRARIES
    gtsam
    mrpt::poses
  #PRIVATE_LINK_LIBRARIES
  CMAKE_DEPENDENCIES
    mrpt-poses
)

# -----------------------
# define tests:
#enable_testing()
#add_subdirectory(module/tests)

# -----------------------------------------------------------------------------
#  ROS2
# -----------------------------------------------------------------------------
if (DETECTED_ROS2)
  # find dependencies
  find_package(ament_cmake REQUIRED)

  ament_package()
endif()
