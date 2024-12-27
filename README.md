[![CI Check clang-format](https://github.com/MOLAorg/mola_state_estimation/actions/workflows/check-clang-format.yml/badge.svg)](https://github.com/MOLAorg/mola_state_estimation/actions/workflows/check-clang-format.yml)
[![CI ROS](https://github.com/MOLAorg/mola_state_estimation/actions/workflows/build-ros.yml/badge.svg)](https://github.com/MOLAorg/mola_state_estimation/actions/workflows/build-ros.yml)
[![Docs](https://img.shields.io/badge/docs-latest-brightgreen.svg)](https://docs.mola-slam.org/latest/)

| Distro | Build dev | Release |
| --- | --- | --- |
| ROS 2 Humble (u22.04) | [![Build Status](https://build.ros2.org/job/Hdev__mola_state_estimation__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Hdev__mola_state_estimation__ubuntu_jammy_amd64/) | [![Version](https://img.shields.io/ros/v/humble/mola_state_estimation)](https://index.ros.org/search/?term=mola_state_estimation) |
| ROS 2 Jazzy (u24.04) | [![Build Status](https://build.ros2.org/job/Jdev__mola_state_estimation__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Jdev__mola_state_estimation__ubuntu_noble_amd64/) | [![Version](https://img.shields.io/ros/v/jazzy/mola_state_estimation)](https://index.ros.org/search/?term=mola_state_estimation) |
| ROS 2 Rolling (u24.04) | [![Build Status](https://build.ros2.org/job/Rdev__mola_state_estimation__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Rdev__mola_state_estimation__ubuntu_noble_amd64/) | [![Version](https://img.shields.io/ros/v/rolling/mola_state_estimation)](https://index.ros.org/search/?term=mola_state_estimation) |


# mola_state_estimation
Implementations of the MOLA virtual state estimation API for robots / vehicles.

Two packages are provided here:


Refer to the [official documentation](https://docs.mola-slam.org/latest/) for build instructions, demos, API reference, etc.

## Individual package build status

Note: Rows within each cell are for ``amd64`` and ``arm64`` architectures.

| Package | ROS 2 Humble <br/> BinBuild |  ROS 2 Jazzy <br/> BinBuild |  ROS 2 Rolling <br/> BinBuild |
| --- | --- | --- | --- |
| mola_state_estimation_simple | [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mola_state_estimation_simple__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mola_state_estimation_simple__ubuntu_jammy_amd64__binary/) <br> [![Build Status](https://build.ros2.org/job/Hbin_ujv8_uJv8__mola_state_estimation_simple__ubuntu_jammy_arm64__binary/badge/icon)](https://build.ros2.org/job/Hbin_ujv8_uJv8__mola_state_estimation_simple__ubuntu_jammy_arm64__binary/)  | [![Build Status](https://build.ros2.org/job/Jbin_uN64__mola_state_estimation_simple__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_uN64__mola_state_estimation_simple__ubuntu_noble_amd64__binary/) <br> [![Build Status](https://build.ros2.org/job/Jbin_unv8_uNv8__mola_state_estimation_simple__ubuntu_noble_arm64__binary/badge/icon)](https://build.ros2.org/job/Jbin_unv8_uNv8__mola_state_estimation_simple__ubuntu_noble_arm64__binary/)  | [![Build Status](https://build.ros2.org/job/Rbin_uN64__mola_state_estimation_simple__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uN64__mola_state_estimation_simple__ubuntu_noble_amd64__binary/)<br> [![Build Status](https://build.ros2.org/job/Rbin_unv8_uNv8__mola_state_estimation_simple__ubuntu_noble_arm64__binary/badge/icon)](https://build.ros2.org/job/Rbin_unv8_uNv8__mola_state_estimation_simple__ubuntu_noble_arm64__binary/) | 
| mola_state_estimation_smoother | [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mola_state_estimation_smoother__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mola_state_estimation_smoother__ubuntu_jammy_amd64__binary/) <br> [![Build Status](https://build.ros2.org/job/Hbin_ujv8_uJv8__mola_state_estimation_smoother__ubuntu_jammy_arm64__binary/badge/icon)](https://build.ros2.org/job/Hbin_ujv8_uJv8__mola_state_estimation_smoother__ubuntu_jammy_arm64__binary/)  | [![Build Status](https://build.ros2.org/job/Jbin_uN64__mola_state_estimation_smoother__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_uN64__mola_state_estimation_smoother__ubuntu_noble_amd64__binary/) <br> [![Build Status](https://build.ros2.org/job/Jbin_unv8_uNv8__mola_state_estimation_smoother__ubuntu_noble_arm64__binary/badge/icon)](https://build.ros2.org/job/Jbin_unv8_uNv8__mola_state_estimation_smoother__ubuntu_noble_arm64__binary/)  | [![Build Status](https://build.ros2.org/job/Rbin_uN64__mola_state_estimation_smoother__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uN64__mola_state_estimation_smoother__ubuntu_noble_amd64__binary/)<br> [![Build Status](https://build.ros2.org/job/Rbin_unv8_uNv8__mola_state_estimation_smoother__ubuntu_noble_arm64__binary/badge/icon)](https://build.ros2.org/job/Rbin_unv8_uNv8__mola_state_estimation_smoother__ubuntu_noble_arm64__binary/) | 

## License
MOLA is released under the GNU GPL v3 license, except noted otherwise in each individual module. Other options available upon request.
Some modules are released under BSD-3. See the [official documentation](https://docs.mola-slam.org/latest/).
