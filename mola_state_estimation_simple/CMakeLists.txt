# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
#
# Copyright (C) 2018-2025, Jose Luis Blanco-Claraco, contributors (AUTHORS.md)
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file
# ------------------------------------------------------------------------------

# Minimum CMake vesion: limited by CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS
cmake_minimum_required(VERSION 3.5)

# Tell CMake we'll use C++ for use in its tests/flags
project(mola_state_estimation_simple LANGUAGES CXX)

# MOLA CMake scripts: "mola_xxx()"
find_package(mola_common REQUIRED)

# find CMake dependencies:
find_package(mrpt-obs)

# Find MOLA packages:
find_package(mola_imu_preintegration REQUIRED)
find_package(mola_kernel REQUIRED)

# -----------------------
# define lib:
file(GLOB_RECURSE LIB_SRCS src/*.cpp src/*.h)
file(GLOB_RECURSE LIB_PUBLIC_HDRS include/*.h)

mola_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
  PUBLIC_LINK_LIBRARIES
    mrpt::obs
    mola::mola_imu_preintegration
    mola::mola_kernel
  #PRIVATE_LINK_LIBRARIES
  #  mrpt::obs
  CMAKE_DEPENDENCIES
    mola_common
    mola_kernel
    mola_imu_preintegration
)

# -----------------------
# define tests:
#enable_testing()
#add_subdirectory(tests)
