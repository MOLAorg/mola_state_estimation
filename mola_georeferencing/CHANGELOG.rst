^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mola_georeferencing
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.1 (2026-01-15)
------------------
* Fix potential bug in sm_georeferencing
* Merge pull request `#11 <https://github.com/MOLAorg/mola_state_estimation/issues/11>`_ from MOLAorg/feat/improve-docs
  Improve docs for mola_gtsam_factors
* Add missing #include for gtsam
* Contributors: Jose Luis Blanco-Claraco

2.0.0 (2026-01-14)
------------------
* Merge pull request `#7 <https://github.com/MOLAorg/mola_state_estimation/issues/7>`_ from MOLAorg/feature/tricycle-kinematic
  Add tricycle kinematics
* lint clean ups
* Copyright year bump
* Merge pull request `#10 <https://github.com/MOLAorg/mola_state_estimation/issues/10>`_ from MOLAorg/feature/add-georef-tests
  Add georeference unit tests
* Add georef unit test
* Merge pull request `#5 <https://github.com/MOLAorg/mola_state_estimation/issues/5>`_ from MOLAorg/feature/fuse-gnss-imu-odom
  Refactor: new packages mola_georeferencing, mola_gtsam_factors, functional smoother state estimator
* Make mola_gtsam_factors non headers-only and rename GNSS2ENU as GnssEnu for consistency
* process CObservationRobotPose
* integrate code coverage in cmake
* package.xml: add FILE to license tags
* Refactor to expose all gtsam factors into a new library 'mola_gtsam_factors'
* new package mola_georeferencing, imported and refactored from mola_sm_loop_closure
* Contributors: Jose Luis Blanco-Claraco
