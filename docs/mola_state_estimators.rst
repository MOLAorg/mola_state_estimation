.. _mola_sta_est_index:

===================
State estimators
===================

.. contents::
   :depth: 1
   :local:
   :backlinks: none

____________________________________________

|

1. Theory
---------------------------------
State Estimation (SE) comprises finding the **vehicle kinematic state(s)**
that **best explain** the imperfect, noisy **sensor readings**.

**What?** In MOLA, the kinematic state of the vehicle at time :math:`t` is

.. math::

   \mathbf{x}(t) = \left( \mathbf{T}(t),\ \mathbf{v}(t),\ \boldsymbol{\omega}(t) \right)

where :math:`\mathbf{T}(t) \in SE(3)` is the pose of the vehicle (``base_link``) in a
reference frame (typically ``map``), and :math:`\mathbf{v}(t), \boldsymbol{\omega}(t)`
are its linear and angular velocity expressed in the vehicle's own frame (the *body twist*).
A state estimator answers *"what is* :math:`\mathbf{x}(t)` *and how uncertain is it?"*
for any time :math:`t` close to the latest measurements, in any of the frames it knows about.

**Why?**

- No single sensor is enough: wheel, visual and LiDAR odometry drift over time; GNSS is
  absolute but noisy, slow, and sometimes unavailable; an IMU gives attitude and rates,
  but positions integrated from it drift within seconds.
- Sensors run at different rates and with different latencies, while consumers need the
  state at *their own* timestamps: LiDAR odometry needs a motion prior at each scan time
  (ICP initial guess, scan de-skewing), a controller needs a smooth, high-rate pose.
- Each estimate comes with a covariance, so it can be used as a properly weighted prior
  by other modules.

**How?** By default, both estimators share a *constant velocity* motion model between
measurements (``StateEstimationSimple`` can optionally integrate the IMU instead, see
``imu_propagation`` in :ref:`section 4 <mola_sta_est_simple>`):

.. math::

   \mathbf{T}(t + \Delta t) = \mathbf{T}(t) \oplus \exp\!\left(
   \begin{bmatrix} \mathbf{v}(t) \\ \boldsymbol{\omega}(t) \end{bmatrix} \Delta t \right)

where unmodeled accelerations are treated as zero-mean white noise
(parameters ``sigma_random_walk_acceleration_linear`` and ``_angular``), so uncertainty grows
with :math:`\Delta t`. This model is used to interpolate or extrapolate the state to any requested
time. Requests farther than ``max_time_to_use_velocity_model`` from the latest data return no
estimate.

The two implementations differ in how measurements are combined:

- ``StateEstimationSimple`` (:ref:`section 4 <mola_sta_est_simple>`) keeps only the latest
  state: each new pose replaces the previous one, and velocities are low-pass filtered.
  Cheap and robust, but it does not really *fuse* redundant sources.
- ``StateEstimationSmoother`` (:ref:`section 5 <mola_sta_est_smoother>`) solves a
  maximum a posteriori (MAP) problem over a sliding window of *keyframes*:

  .. math::

     \mathcal{X}^\star = \arg\min_{\mathcal{X}} \sum_j \rho_j\!\left(
     \left\| \mathbf{r}_j(\mathcal{X}_j) \right\|^2_{\boldsymbol{\Sigma}_j} \right),
     \qquad
     \|\mathbf{r}\|^2_{\boldsymbol{\Sigma}} = \mathbf{r}^\top \boldsymbol{\Sigma}^{-1} \mathbf{r}

  The variables :math:`\mathcal{X}` are :math:`\mathbf{T}_k, \mathbf{v}_k, \boldsymbol{\omega}_k`
  for every keyframe :math:`k` in the last ``sliding_window_length`` seconds, plus the pose of
  each odometry frame in ``map`` (``T_map_to_odom_i``) and the geo-reference (``T_enu_to_map``).
  Each factor :math:`\mathbf{r}_j` is either a sensor measurement or a kinematic constraint between
  consecutive keyframes, with covariance :math:`\boldsymbol{\Sigma}_j`; :math:`\rho_j` is the
  identity (Gaussian noise) or a Huber kernel for robust factors.
  The problem is solved incrementally with iSAM2 (GTSAM), and keyframes leaving the window are
  marginalized out, so the cost per update stays bounded.
  Since all states within the window are re-estimated as new evidence arrives, the smoother can
  also estimate quantities only observable over time, such as the drift of each odometry frame
  with respect to ``map``, or the geo-reference.

|

2. Selecting the S.E. method in launch files
------------------------------------------------

.. _mola_sta_est_try_it:

2.1. Try it yourself: simulated sensors + RViz
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
No robot or dataset is needed for a first look at the smoother. This demo runs a simulated
robot driving a circle of 5 m radius at 1 m/s, publishing noisy wheel odometry, drifting
visual odometry, IMU and GNSS, plus its ground truth. The smoother fuses the sensor
combination selected with ``mode``, and RViz shows the result:

.. code-block:: bash

   ros2 launch mola_state_estimation_smoother ros2-demo-simulated-sensors.launch.py \
     mode:=wheels_imu

.. figure:: imgs/state_estimation_demo_rviz.webp
   :width: 500
   :align: center

   ``mode:=wheels_imu`` after one lap: ground truth (white), raw wheel odometry (red),
   raw visual odometry (orange, not fused in this mode), fused trail (green), and the
   current fused pose with its position covariance (blue).

.. list-table::
   :header-rows: 1
   :widths: 25 75

   * - ``mode``
     - Fused sensors, and what to look at
   * - ``wheels_imu`` (default)
     - Wheel odometry + IMU. The wheel odometry heading drifts; the IMU attitude keeps the
       fused heading, so the fused trail stays on the ground truth while the raw odometry
       spirals away. Position uncertainty grows, since nothing observes absolute position.
   * - ``wheels_imu_gnss``
     - Wheel odometry + IMU + GNSS, estimating the geo-reference (``enu -> map``) online.
       The ground truth, given in ``enu``, is only drawn once the geo-reference converges,
       and through that estimate, so it also shows its error.
   * - ``two_odometries``
     - Wheel odometry + drifting visual odometry + IMU: two odometry chains, each with its
       own ``T_map_to_odom_i``.
   * - ``imu_gnss``
     - IMU + GNSS only. Without odometry, the trajectory between GNSS fixes relies on the
       IMU and the constant velocity model alone; compare it with the other modes.

Other launch arguments: ``use_rviz`` (default ``True``), ``use_mola_gui`` (MolaViz console,
default ``False``), and the simulated noise ``wheel_odom_ang_sigma`` (yaw rate noise,
default ``0.15`` rad/s) and ``visual_odom_drift`` (lateral drift, default ``0.08`` m/s).
Smoother parameters can be overridden as explained in the next section, e.g. prefix the
command with ``MOLA_REL_POSE_INCR_SIGMA_LIN=0.05`` to see the effect of trusting each
odometry increment less.

Under the hood, this launch file runs the synthetic sensor publisher from ``mola_demos``
and includes ``ros2-state-estimator.launch.py``, the same one used with real sensors below.

2.2. Launching the state estimator standalone
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Next we show different possible use cases.
All of them run ``StateEstimationSmoother`` inside a ``mola-cli`` process, with
``mola::BridgeROS2`` subscribing to the sensor topics and publishing the fused result back
to ROS 2. See :ref:`section 3 <mola_sta_est_api>` for how each message type is fused.

Smoother parameters are read from
`params/state-estimation-smoother.yaml <https://github.com/MOLAorg/mola_state_estimation/blob/develop/mola_state_estimation_smoother/params/state-estimation-smoother.yaml>`_.
Most of them can be overridden via the environment variables named in that file, or a whole
custom copy of the file can be used instead:

.. code-block:: bash

   MOLA_STATE_ESTIMATOR_YAML=/path/to/my-smoother-params.yaml \
     ros2 launch mola_state_estimation_smoother ros2-state-estimator.launch.py [...]

.. dropdown:: Merging wheel odometry + GNSS + IMU
   :icon: code-review


    .. code-block:: bash

      # MOLA_VERBOSITY_BRIDGE_ROS2=DEBUG \
      # MOLA_VERBOSITY_MOLA_STATE_ESTIMATOR=DEBUG \

      ros2 launch mola_state_estimation_smoother ros2-state-estimator.launch.py \
        estimate_geo_reference:=True \
        odom1_topic:=/wheel_odom \
        imu_topic_name:=/imu \
        gnss_topic_name:=/gps1

   Up to three ``nav_msgs/Odometry`` sources can be fused simultaneously via
   ``odom1_topic``, ``odom2_topic``, and ``odom3_topic`` (empty string disables
   each one). Set ``odomN_as_robot_pose:=False`` for wheel odometry (planar, with the
   wheel motion model; the default ``True`` is for LiDAR or visual odometry).
   A second IMU goes in ``imu2_topic_name``: both gyroscopes are preintegrated with
   ``imu_preintegration_gyro_only:=True imu_preintegration_max_imus:=2`` (otherwise only the
   first IMU heard is). For bag replay, add ``use_sim_time:=True``. Without at least one odometry source the smoother relies solely on
   the constant-velocity kinematic model between GNSS fixes, which degrades for
   non-smooth motion.

   The launch arguments ``navstate_kinematic_model``, ``navstate_sliding_window_sec``,
   ``navstate_sigma_random_walk_linacc`` and ``navstate_sigma_random_walk_angacc``
   are empty by default, meaning the value from the parameters YAML file is used.
   Set them to override it.


.. dropdown:: Fusing two ``nav_msgs/Odometry`` sources (e.g. wheel + visual odometry)
   :icon: code-review

   This demo subscribes to two ``nav_msgs/Odometry`` topics from ROS 2, fuses them
   in the sliding-window factor graph smoother alongside an optional IMU, and publishes
   the fused result back as ``nav_msgs/Odometry`` + ``/tf``.

   Each odometry topic is assigned a distinct ``output_sensor_label``; the smoother
   treats them as independent odometry frames and estimates the optimal
   ``T_map_to_odom_X`` transform for each one.

   **Step 1: Start the fake sensor publisher (for testing without a real robot):**

   .. code-block:: bash

      # Wheel odom (50 Hz) + visual odom (30 Hz, Y drift) + IMU (100 Hz),
      # all from a single script, circular motion at vx=1 m/s, wz=0.2 rad/s:
      python3 $(ros2 pkg prefix mola_demos)/share/mola_demos/demos/fake_sensor_publisher.py \
        --ros-args \
        -p scenario:=circle \
        -p odom2_topic:=/visual_odom \
        -p imu_topic:=/imu

   The two odometry streams share the same ground-truth circular motion but have
   different noise and drift characteristics so the smoother can demonstrate
   visible fusion benefit.  The script supports three scenarios via ``scenario:=``
   (``circle``, ``moving``, ``static``) and can also publish GNSS by setting
   ``gnss_topic:=/gps``.

   **Step 2: Launch the smoother:**

   .. code-block:: bash

      ros2 launch mola_state_estimation_smoother ros2-fuse-two-odometries.launch.py \
        odom1_topic:=/wheel_odom \
        odom2_topic:=/visual_odom \
        imu_topic_name:=/imu

   Or using ``mola-cli`` directly (set topic names via environment variables):

   .. code-block:: bash

      ODOM1_TOPIC=/wheel_odom \
      ODOM2_TOPIC=/visual_odom \
      IMU_TOPIC=/imu \
        mola-cli $(ros2 pkg prefix mola_state_estimation_smoother)/share/mola_state_estimation_smoother/mola-cli-launchs/state_estimator_ros2.yaml

   .. tip::

      Odometry sources are fused as chains of increments, whose uncertainty is set by
      ``relative_pose_increment_sigma_lin`` / ``_ang`` (env vars ``MOLA_REL_POSE_INCR_SIGMA_LIN``
      / ``_ANG``), not by the covariance in the messages. The defaults suit LiDAR odometry
      (``_lin`` 0 = automatic: 0.1 mm once an IMU is being preintegrated, 2 cm otherwise), and
      are shared by all such sources. For wheel odometry use instead
      ``odomN_as_robot_pose:=False``: it is then fused as planar odometry with its own
      motion model (``odom_motion_model_a1`` .. ``a4``, env vars ``MOLA_ODOM_MOTION_MODEL_A*``).
      See :ref:`section 3 <mola_sta_est_api>`.

   Key launch arguments:

   .. list-table::
      :header-rows: 1
      :widths: 30 15 55

      * - Argument
        - Default
        - Description
      * - ``odom1_topic``
        - ``/wheel_odom``
        - First ``nav_msgs/Odometry`` topic (e.g. wheel encoders)
      * - ``odom1_label``
        - ``odom_wheels``
        - Sensor label (and odometry frame name) of the first source
      * - ``odom2_topic``
        - ``/visual_odom``
        - Second ``nav_msgs/Odometry`` topic (e.g. visual/LiDAR odometry)
      * - ``odom2_label``
        - ``visual_odom``
        - Sensor label (and odometry frame name) of the second source
      * - ``imu_topic_name``
        - ``/imu``
        - IMU topic (empty to disable)
      * - ``gnss_topic_name``
        - (empty)
        - Optional GNSS ``NavSatFix`` topic (empty to disable)
      * - ``enforce_planar_motion``
        - ``False``
        - Constrain z=0, pitch=0, roll=0 for ground vehicles
      * - ``use_mola_gui``
        - ``True``
        - Show MolaViz visualization
      * - ``use_rviz``
        - ``False``
        - Also launch RViz2

   **Step 3: Verify fused output:**

   .. code-block:: bash

      # Fused pose as nav_msgs/Odometry:
      ros2 topic echo /state_estimation/pose

      # Inspect all published topics:
      ros2 topic list | grep state_estimation

      # View /tf tree:
      ros2 run tf2_tools view_frames


.. dropdown:: LiDAR odometry + wheel odometry fused in the smoother
   :icon: code-review

   This demo runs ``mola::LidarOdometry`` from a live ``PointCloud2`` topic alongside
   an external wheel odometry source from ROS 2. Both are fused by
   ``StateEstimationSmoother``, and the fused result is published back to ROS 2.

   Data flow::

     ROS2 /lidar_points  --> BridgeROS2 --> LidarOdometry ---+
     ROS2 /wheel_odom    --> BridgeROS2 ----+                |
     ROS2 /imu           --> BridgeROS2 ----+--> StateEstimationSmoother
                                                      |
                                             advertiseUpdatedLocalization()
                                                      |
                                               BridgeROS2 --> /state_estimation/pose
                                                          --> /tf (map -> base_link)

   .. code-block:: bash

      ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
        lidar_topic_name:=/ouster/points \
        use_state_estimator:=True \
        forward_ros_tf_odom_to_mola:=False

   To additionally subscribe to a wheel odometry topic, use the ``mola-cli`` YAML directly
   (``GNSS_TOPIC`` is optional, and smoother parameters come from the same YAML file as above):

   .. code-block:: bash

      WHEEL_ODOM_TOPIC=/wheel_odom \
      MOLA_LIDAR_TOPIC=/ouster/points \
        mola-cli $(ros2 pkg prefix mola_state_estimation_smoother)/share/mola_state_estimation_smoother/mola-cli-launchs/demo_lidar_odom_plus_wheel_odom_fusion.yaml


|

2.3. Launching the state estimator + LO/LIO
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
In the context of launching LiDAR odometry (LO) mapping or localization
as explained :ref:`here <launching_mola_lo>`, note that default configurations
include ``StateEstimationSimple`` as the method of choice, but it can be
changed as follows:

.. dropdown:: MOLA-LO with a custom State Estimation configuration
   :icon: code-review

   Both, all MOLA-LO GUI applications, and the ROS node, rely on MOLA system :ref:`configuration files <yaml_slam_cfg_file>`
   to know what MOLA modules to launch and what parameters to pass to them.

   - `Read through those files <https://github.com/MOLAorg/mola_lidar_odometry/tree/develop/mola-cli-launchs>`_
     to fully understand what is under the hood.
   - Default parameter files for estimators:
     `state-estimation-simple.yaml <https://github.com/MOLAorg/mola_lidar_odometry/tree/develop/state-estimator-params>`_
     (in ``mola_lidar_odometry``) and
     `state-estimation-smoother.yaml <https://github.com/MOLAorg/mola_state_estimation/blob/develop/mola_state_estimation_smoother/params/state-estimation-smoother.yaml>`_
     (in ``mola_state_estimation_smoother``).

   So, what follows are just examples that should be considered starting points for user customizations by using custom S.E. parameter files:

   .. tab-set::

      .. tab-item:: Defaults
         :selected:

         .. code-block:: bash

            # Launch LO-GUI on the KITTI dataset, using the default state estimator:
            mola-lo-gui-kitti 04

            # Launch MOLA-LO (CLI version) on KITTI, using default state estimator:
            mola-lidar-odometry-cli \
              -c $(ros2 pkg prefix mola_lidar_odometry)/share/mola_lidar_odometry/pipelines/lidar3d-default.yaml \
              --state-estimator-param-file $(ros2 pkg prefix mola_lidar_odometry)/share/mola_lidar_odometry/state-estimator-params/state-estimation-simple.yaml \
              --input-kitti-seq 04

      .. tab-item:: Custom state estimator configuration

         .. code-block:: bash

            # Launch LO-GUI on the KITTI dataset, using the smoother state estimator:
            MOLA_STATE_ESTIMATOR="mola::state_estimation_smoother::StateEstimationSmoother" \
            MOLA_STATE_ESTIMATOR_YAML="$(ros2 pkg prefix mola_state_estimation_smoother)/share/mola_state_estimation_smoother/params/state-estimation-smoother.yaml" \
              mola-lo-gui-kitti 04

            # Launch MOLA-LO (CLI version) on KITTI, using the smoother state estimator.
            # MOLA_ASYNC_BACKEND=false selects the deterministic (synchronous) solver,
            # best for offline runs:
            MOLA_ASYNC_BACKEND=false \
            mola-lidar-odometry-cli \
              -c $(ros2 pkg prefix mola_lidar_odometry)/share/mola_lidar_odometry/pipelines/lidar3d-default.yaml \
              --state-estimator "mola::state_estimation_smoother::StateEstimationSmoother" \
              --state-estimator-param-file $(ros2 pkg prefix mola_state_estimation_smoother)/share/mola_state_estimation_smoother/params/state-estimation-smoother.yaml \
              --load-plugins libmola_state_estimation_smoother.so \
              --input-kitti-seq 04

|

.. _mola_sta_est_api:

3. API and supported inputs
---------------------------------

.. image:: imgs/mola_state_estimation_api_overview.webp

3.1. C++ API
~~~~~~~~~~~~~~~~
Both estimators implement the virtual interface
:ref:`mola::NavStateFilter <doxid-classmola_1_1_nav_state_filter>`:

.. list-table::
   :header-rows: 1
   :widths: 35 65

   * - Method
     - Purpose
   * - ``fuse_pose(t, pose, frame_id)``
     - SE(3) pose with covariance of the vehicle at time ``t``, relative to frame ``frame_id``
       (``map``, or a source's own odometry frame).
   * - ``fuse_odometry(obs, name)``
     - Planar wheel odometry (``CObservationOdometry``); increment uncertainty comes from a
       motion model.
   * - ``fuse_imu(obs)``
     - IMU reading (``CObservationIMU``): orientation, acceleration and/or angular velocity.
   * - ``fuse_gnss(obs)``
     - GNSS fix (``CObservationGPS``).
   * - ``fuse_twist(t, twist, cov)``
     - Body velocity measurement.
   * - ``estimated_navstate(t, frame_id)``
     - Returns a :ref:`mola::NavState <doxid-structmola_1_1_nav_state>`: pose with its
       information matrix in ``frame_id``, plus body twist with its information matrix.
       Empty if there is not enough data yet, or ``t`` is too far from the latest data.

Optional methods: ``estimated_trajectory()``, ``has_converged_localization()``,
``set_geo_reference()`` / ``get_geo_reference()``.

When running as a MOLA module (e.g. inside ``mola-cli``), an estimator:

- receives raw observations from the module(s) named in its ``raw_data_source`` (e.g. ``BridgeROS2``,
  or a dataset source), and dispatches them by type as in the table below. Sensor labels are
  filtered with the regular expressions ``do_process_imu_labels_re``,
  ``do_process_odometry_labels_re`` and ``do_process_gnss_labels_re``. Observations labeled
  ``ground_truth`` are ignored unless ``fuse_ground_truth_label: true``.
- receives ``fuse_pose()`` calls and ``estimated_navstate()`` queries from other modules.
  For example, ``mola::LidarOdometry`` finds the estimator module in the system, feeds it
  its ICP poses, and uses its predictions as motion prior.

3.2. Inputs from ROS 2
~~~~~~~~~~~~~~~~~~~~~~~~~~
``mola::BridgeROS2`` converts subscribed topics into MOLA observations, labeled with each
topic's ``output_sensor_label``. IMU and GNSS observations carry the sensor pose on the vehicle,
taken from ``/tf`` (or from ``fixed_sensor_pose`` if ``use_fixed_sensor_pose: true``).

.. list-table::
   :header-rows: 1
   :widths: 18 41 41

   * - ROS 2 message
     - ``StateEstimationSmoother``
     - ``StateEstimationSimple``
   * - ``nav_msgs/Odometry``
       (default: becomes a ``CObservationRobotPose``)
     - ``fuse_pose()`` in the source's own frame (named after its label). Odometry drifts,
       so only the increments between consecutive keyframes are fused, with the uncertainty
       set by the parameters ``relative_pose_increment_sigma_lin`` / ``_ang``
       (plus optional growth with the increment size, ``..._per_sqrt_meter`` / ``_rad``).
       Their defaults suit LiDAR odometry; for wheel odometry use the planar mode below.
       The first reading adds one absolute factor, with the message covariance, to resolve
       ``T_map_to_odom_<label>``. The twist part is not used.
     - Increments between consecutive readings dead-reckon the pose between primary pose
       updates.
   * - ``nav_msgs/Odometry``
       (with ``BridgeROS2`` param ``odometry_as_robot_pose_observation: false``, global or
       per ``subscribe`` entry, i.e. launch argument ``odomN_as_robot_pose:=False``:
       becomes a planar ``CObservationOdometry``)
     - ``fuse_odometry()``: relative increments, with covariance from the motion model
       ``odom_motion_model_a1`` .. ``a4``.
     - Increments as above, plus wheel velocities (``sigma_wheel_odom_*``).
   * - ``sensor_msgs/Imu``
     - Orientation as an absolute attitude factor (``imu_attitude_sigma_deg``, 0 disables;
       ``imu_attitude_azimuth_offset_deg``; at most one per ``imu_attitude_min_sample_period``;
       optional azimuth bias state, see below); acceleration as gravity direction
       (``imu_normalized_gravity_alignment_sigma``, 0 disables); angular velocity as a prior
       on :math:`\boldsymbol{\omega}_k` (``imu_angular_velocity_sigma``, 0 disables).
       Readings are averaged over ``imu_min_sample_period``.
     - Angular velocity (``sigma_imu_angular_velocity``). Optionally, inertial propagation
       between pose updates (``imu_propagation``).
   * - ``sensor_msgs/NavSatFix``, ``gps_msgs/GPSFix``
     - ENU position factor with a robust kernel (``gnss_robust_kernel``: Huber by default, or
       Cauchy, Geman-McClure, DCS; scale ``gnss_huber_threshold``), on the closest keyframe
       moved to the fix time with its twist. Sigmas are floored at ``gnss_min_sigma_floor_xy``
       and ``gnss_min_sigma_floor_z`` (``MOLA_NAVSTATE_GNSS_MIN_SIGMA_XY`` / ``_Z``, 0.10 m and
       0.30 m by default, as in the simple estimator). Each fix must first pass admission tests
       (accepted fix qualities, valid covariance, maximum horizontal sigma, optional height band
       around the datum, and optionally ``gnss_max_sigma_ratio``: a horizontal sigma at most
       that many times the lowest one in the last ``gnss_sigma_ratio_window`` seconds, for
       receivers that keep reporting a solution while it degrades; enable it only when the
       odometry is better than those degraded fixes) and an innovation gate against the predicted antenna position
       (``gnss_gate_chi2``, 0 disables it); rejected fixes are logged and counted, and a run of
       fixes that keep moving like the vehicle is reacquired after a drift. Precise (RTK
       fixed) fixes are reacquired at once when their residual grew smoothly since a fix within
       the nominal gate, over ``gnss_gate_reacquire_after`` fixes and
       ``gnss_gate_reacquire_min_path`` of motion, and their height passes the gate; a jump of
       the residual (e.g. a wrong RTK fix) starts over both that chain and the run of rejected
       fixes. With
       ``estimate_geo_reference``, the datum is only set after a few mutually consistent fixes.
       Requires a geo-reference:
       either estimated (``estimate_geo_reference: true``) or fixed (``fixed_geo_reference``,
       or given by a geo-referenced map). The final one can be written to a YAML file with
       ``save_final_georef_to_file`` (``MOLA_SE_SAVE_GEOREF_FILE``), and poses can be queried
       in the ``enu`` frame: each one through the ``T_enu_to_map`` it was estimated with, since
       ``map`` itself moves with respect to ``enu`` while that transform is refined.
     - Ignored unless ``gnss_enabled: true`` and a geo-reference is set; then low-sigma fixes
       nudge the position.
   * - (none: C++ or other MOLA modules, e.g. LiDAR odometry)
     - ``fuse_pose()`` with ``frame_id`` equal to ``reference_frame_name`` (``map``): a prior
       factor on the keyframe pose. Other frames: as for ``nav_msgs/Odometry`` above.
     - Each pose becomes the new state; velocity is estimated from consecutive poses.

.. note::

   **IMU orientation.** ``sensor_msgs/Imu`` orientation is used unless
   ``orientation_covariance[0]`` is negative (the ROS convention for "no orientation").
   An IMU without magnetometer, whose yaw is arbitrary, must publish ``-1`` there, otherwise
   that yaw is taken as the absolute heading. By default (offset 0), yaw zero is assumed to point
   North; IMUs whose yaw zero points East (ENU, as in REP-103) need
   ``imu_attitude_azimuth_offset_deg: -90``.

   **IMU azimuth bias.** Magnetometer-based headings carry an error shared by all readings
   (hard/soft-iron residuals, mounting offsets). Fused as independent readings, their average
   looks ever more precise while it keeps that error. With ``imu_attitude_yaw_bias_sigma_deg``
   > 0, the smoother estimates it as a state :math:`b_k` (one per keyframe with attitude), so
   that each reading observes :math:`R_z(b_k) R_k`, with a first-order Gauss-Markov model:

   .. math::

      b_k = \phi b_{k-1} + w_k, \quad \phi = e^{-\Delta t / \tau}, \quad
      \sigma_{w_k}^2 = \sigma_b^2 (1 - \phi^2)

   with :math:`\sigma_b` = ``imu_attitude_yaw_bias_sigma_deg`` (stationary sigma) and
   :math:`\tau` = ``imu_attitude_yaw_bias_time_constant``. While only the IMU observes the
   heading, its sigma stays near :math:`\sigma_b`; once other sources (e.g. LiDAR
   localization in a map) observe it, the bias is estimated and keeps correcting the IMU
   heading for a time of the order of :math:`\tau`. ``publish_imu_yaw_bias: true`` publishes
   it (ROS 2: ``std_msgs/Float32`` ``mola_diagnostics/<module>/imu_yaw_bias_deg`` and
   ``imu_yaw_bias_sigma_deg``).

   **GNSS receiver bias.** Most of the error of a low-cost (non-RTK) receiver is a slowly
   varying offset of one or two meters, not white noise: fused as independent fixes, their
   average looks ever more precise while it keeps that offset, the innovation gate cannot
   tell it apart, and when a better source appears (RTK, map-based localization) the estimate
   jumps. With ``gnss_bias_sigma_xy`` > 0, each receiver whose label matches
   ``gnss_bias_labels_re`` (empty by default, so both must be set, e.g. only the low-cost
   receiver when an RTK one is fused too) gets a bias state :math:`\mathbf{b}_k` (ENU, one per fused fix),
   so that each fix observes the antenna position plus :math:`\mathbf{b}_k`, with the same
   first-order Gauss-Markov model as above (:math:`\sigma_b` = ``gnss_bias_sigma_xy``, and
   ``gnss_bias_sigma_z`` for the vertical, 0 = horizontal only; :math:`\tau` =
   ``gnss_bias_time_constant``). The receiver sigmas must then describe only the white part
   of its error (``gnss_bias_white_sigma_xy`` / ``_z`` replace the reported ones, which
   usually include the bias). With a single receiver and an estimated geo-reference a
   constant offset is not observable: the position sigma stays near :math:`\sigma_b`
   (``convergence_max_position_sigma`` must be above it). With a second absolute reference,
   e.g. RTK fixes from another receiver for a while, the bias is estimated and the low-cost
   receiver keeps the estimate close to that reference for a time of the order of
   :math:`\tau`. The innovation gate predicts those fixes with the bias, and does not lock
   out the fixes of other receivers while the estimate is uncertain. The bias state helps
   only while the odometry drifts less than the bias can: a drift of :math:`d` m/s ends up
   about :math:`d\,\tau` meters inside the bias instead of being corrected, so a bias
   estimate beyond ``gnss_bias_restart_sigmas`` (3 by default) restarts its chain.
   ``publish_gnss_bias: true`` publishes the estimates
   (``mola_diagnostics/<module>/gnss_bias_<label>_{e,n,u}_m``, ``..._sigma_m``). Model, factor
   graph, observability and validation: :ref:`mola_sta_est_gnss_bias`.

3.3. Outputs
~~~~~~~~~~~~~~
- ``estimated_navstate()`` from C++, for any time and frame (see above).
- ``StateEstimationSmoother`` also publishes, at its module ``execution_rate``, the fused pose
  of ``vehicle_frame_name`` (``base_link``) in ``reference_frame_name`` (``map``).
  ``BridgeROS2`` forwards it as ``nav_msgs/Odometry`` on ``<module name>/pose``
  (e.g. ``/state_estimation/pose``) and as ``/tf``, if its
  ``publish_*_from_slam_source`` parameters select this module.
  Optional extra outputs: ``map -> odom`` (``publish_map_to_odom_tf``) and the fused pose in a
  separate child frame (``publish_fused_vehicle_tf``). An estimated geo-reference is published once
  converged (``publish_estimated_georef_on_convergence``).
- ``StateEstimationSimple`` does not publish anything by itself: it serves predictions to
  other modules, e.g. LiDAR odometry, which publishes its own pose.

|

.. _mola_sta_est_simple:

4. Implementation: Simple estimator
---------------------------------------
The package ``mola_state_estimation_simple`` implements
:ref:`StateEstimationSimple <doxid-classmola_1_1state__estimation__simple_1_1_state_estimation_simple>`,
a lightweight constant-velocity estimator. It is the default in MOLA LiDAR odometry, since
for LO/LIO the LiDAR poses dominate and all it needs is a good motion prior for the next scan.

Algorithm:

- **State:** the latest pose (with covariance) at its timestamp, plus the body twist
  :math:`(\mathbf{v}, \boldsymbol{\omega})`.
- **Pose updates** (``fuse_pose()``, e.g. LiDAR odometry) replace the pose. The twist is
  derived from consecutive poses of the same source (pose increment divided by :math:`\Delta t`).
- **Odometry** readings dead-reckon the pose forward between pose updates. Planar wheel odometry
  is applied in the yaw-only frame, and its velocities (if present) update the twist.
- **IMU** angular velocity updates :math:`\boldsymbol{\omega}`. IMU and odometry readings are
  buffered and applied in timestamp order, so results do not depend on delivery order.
- **Velocity filter** (``velocity_filter_enabled``, default on): each twist component is
  smoothed by a scalar Kalman filter, with process noise from
  ``sigma_random_walk_acceleration_*`` and measurement noise from each input.
- **Prediction:** ``estimated_navstate(t)`` applies the constant velocity model from the last pose
  (or integrates the IMU, if ``imu_propagation`` is enabled; the twist is then the inertial velocity
  at ``t``, from the stamp of the last pose on, holding the newest IMU reading until the next one
  arrives), with a diagonal covariance growing
  with :math:`\Delta t` since the last pose update:

  .. math::

     \sigma^2_{xyz} = \sigma^2_{v}\,\Delta t^2 + \left(\tfrac{1}{2}\sigma_{a,\text{lin}}\,\Delta t^2\right)^2
     + \sigma^2_{\text{rel,lin}},
     \qquad
     \sigma^2_{rot} = \sigma^2_{\omega}\,\Delta t^2 + \left(\tfrac{1}{2}\sigma_{a,\text{ang}}\,\Delta t^2\right)^2
     + \sigma^2_{\text{rel,ang}}

  (added to the last pose covariance), where :math:`\sigma_v, \sigma_\omega` are the
  uncertainties of the filtered twist, ``sigma_random_walk_acceleration_linear/angular``
  (:math:`\sigma_a`) model unmodeled accelerations, and ``sigma_relative_pose_linear/angular``
  (:math:`\sigma_{\text{rel}}`) is a constant floor. The linear velocity covariance is rotated
  from the vehicle frame into the map frame.
- The ``frame_id`` argument is ignored: all poses are assumed to be in the same frame.

Main parameters (default values from
`state-estimation-simple.yaml <https://github.com/MOLAorg/mola_lidar_odometry/tree/develop/state-estimator-params>`_;
the full list is in :ref:`Parameters <doxid-classmola_1_1state__estimation__simple_1_1_parameters>`):

.. list-table::
   :header-rows: 1
   :widths: 40 15 45

   * - Parameter
     - Default
     - Meaning
   * - ``max_time_to_use_velocity_model``
     - 0.75 s
     - Maximum extrapolation time since the last observation.
   * - ``sigma_random_walk_acceleration_linear`` / ``_angular``
     - 1.0 m/s², 10.0 rad/s²
     - Unmodeled acceleration; velocity filter process noise.
   * - ``sigma_relative_pose_linear`` / ``_angular``
     - 0.5 m, 0.1 rad
     - Constant floor of the predicted pose uncertainty (tightness of the ICP prior).
   * - ``sigma_imu_angular_velocity``
     - 0.05 rad/s
     - Gyroscope noise.
   * - ``sigma_wheel_odom_linear_vel`` / ``_angular_vel``
     - 0.1 m/s, 0.05 rad/s
     - Wheel odometry velocity noise.
   * - ``initial_twist`` (+ ``initial_twist_sigma_lin/ang``)
     - zero (20 m/s, 3 rad/s)
     - Initial velocity guess, e.g. for datasets starting in motion.
   * - ``imu_propagation``
     - false
     - Propagate the pose between updates by integrating the IMU (gyro + accelerometer) instead
       of a constant twist.
   * - ``imu_accel_scale``
     - 0
     - Multiplier taking the accelerometer readings to m/s². 0: automatic (units of g if the mean
       norm of the first 0.5 s of valid readings is below 3).
   * - ``gnss_enabled``
     - false
     - Use low-sigma GNSS fixes (e.g. RTK) to correct position drift, given a geo-reference.
   * - ``enforce_planar_motion``
     - false
     - Force z, pitch and roll to zero.

Use the smoother instead when several sources must be truly fused, or when GNSS-based
geo-referencing must be estimated.

|

.. _mola_sta_est_smoother:

5. Implementation: Factor graph smoother
------------------------------------------
The package ``mola_state_estimation_smoother`` implements a sliding window optimization over the
last few keyframes and sensor observations (odometry sources, IMU, GNSS) to solve
for the optimal kinematic state (pose + velocity) at any desired time point, interpolating or extrapolating
into the past or future.

When run as a MOLA module (e.g. within a ROS 2 node), it also publishes the estimated fused pose information
in a timely manner, for use as the high-quality, robust localization source.

This package follows this frame convention (see :ref:`other /tf configurations <mola_ros2_tf_frames>` when using
MOLA LiDAR-odometry without state estimation):

.. figure:: https://mrpt.github.io/imgs/mola_mrpt_ros_frames_fusion.png
    :width: 500
    :align: center

These are the publishers of each transformation:

- ``map → base_link``: Published by **this state estimation package** (``mola_state_estimation_smoother``).
  The odometry sources (``odom_{i}``) are fused from their ``nav_msgs/Odometry`` topics: their drivers must
  not also publish ``odom → base_link`` on ``/tf``, or ``base_link`` would have two parents.
- Alternatively, ``map → odom``, to keep the REP-105 ``map → odom → base_link`` chain of one odometry source
  whose driver publishes ``odom → base_link``: enable ``publish_map_to_odom_tf``
  (``MOLA_PUBLISH_MAP_TO_ODOM_TF=true``) and select it as the published ``/tf`` source
  (``localization_publish_tf_source:=state_estimation/map_odom`` in the MOLA-LO launch file).
  See :ref:`this complete example <localization-lo_georef_gnss_imu_kinematics>`.
- ``enu → {map, utm}``: Published by either:

  - ``mola_lidar_odometry`` :ref:`map loading service <map_loading_saving>` if fed with a geo-referenced metric map (``.mm``) file; or
  - ``mola_state_estimation_smoother`` (this package) if geo-referencing is to be estimated at run-time; or
  - ``mrpt_map_server`` (`github <https://github.com/mrpt-ros-pkg/mrpt_navigation/tree/ros2/mrpt_map_server/>`_) if set to publish a geo-referenced
    map.


The factor graph of a typical configuration (wheel odometry, IMUs and GNSS) is illustrated in
:ref:`this tutorial <tutorial-wheels-imu-gnss-fusion>`.

**Queries at arbitrary times.** ``estimated_navstate(t)`` takes the keyframe closest to ``t``
(before or after it) and extrapolates its pose with its estimated body twist, with the constant
velocity model of :ref:`section 1 <mola_sta_est_index>`, so its uncertainty grows with the time
difference. A request farther than ``max_time_to_use_velocity_model`` from every keyframe returns
no estimate.


5.1. Kinematic factors
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Between two consecutive keyframes close enough in time, a "kinematic factor" is added.
Two options are implemented:

A. Free motion kinematic factor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
This is actually implemented as the combination of distinct GTSAM factors:

- ``mola::factors::FactorConstLocalVelocityPose``: between linear and the angular velocity components of both keyframes to
  favor smooth velocities. See line 3 of eq (4) in the MOLA RSS2019 paper.
- ``mola::factors::FactorTrapezoidalIntegratorPose``: enforces fulfillment of numerical integration on the translational
  part of SE(3). See line 2 of eq (1) in the MOLA RSS2019 paper.
- ``mola::factors::FactorAngularVelocityIntegrationPose``: enforces the fulfillment of numerical integration on the rotational
  part of SE(3). See line 1 of eq (4) in the MOLA RSS2019 paper.


B. Tricycle model kinematic factor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
This is actually implemented as the combination of distinct GTSAM factors:

- ``mola::factors::FactorConstLocalVelocityPose``: between linear and the angular velocity components of both keyframes to
  favor smooth velocities. See line 3 of eq (4) in the MOLA RSS2019 paper.
- ``mola::factors::FactorTricycleKinematic``: integrates the pose along a circular arc with the body
  velocities of the first keyframe (Euler integration), :math:`\mathbf{T}_j = \mathbf{T}_i \oplus \Delta\mathbf{T}`,
  where, with the forward velocity :math:`v = v_x` and the yaw rate :math:`\omega = \omega_z`, the increment
  :math:`\Delta\mathbf{T}` is a rotation of :math:`\omega\,\Delta t` around :math:`z` and a translation

  .. math::

     \Delta\mathbf{p} = \frac{v}{\omega} \begin{bmatrix} \sin(\omega \Delta t) \\ 1-\cos(\omega \Delta t) \\ 0 \end{bmatrix}
     \qquad \left( \Delta\mathbf{p} = \begin{bmatrix} v\,\Delta t & 0 & 0 \end{bmatrix}^\top \text{ for } \omega \approx 0 \right)
- ``gtsam::PriorFactor``: to (gently) favor null components of the local velocity components ``vy``, ``vz``, ``wx``, ``wy``. Parameters can be
  used to tune how much these soft constraints are allowed to be broken, i.e. depending on how much wheel slippage exists.


|

.. _mola_sta_est_gnss_bias:

5.2. GNSS receiver bias states
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Most of the error of a low-cost (non-RTK) receiver is a slowly varying offset, not white noise.
On a ground robot with a u-blox 7 and an RTK ZED-F9P as reference, the u-blox error had a
fix-to-fix (white) part of 0.06-0.13 m, and a colored part with a spread of 1-3 m per axis that
decorrelates in 30-80 s, plus a mean offset of up to 2 m that persists across runs (part of it is
the RTK correction frame against the receiver's WGS84). The sigma the receiver reports (4-5 m)
is a total-error figure and is not correlated with its actual error. Fused as independent fixes,
such a receiver makes the estimate look ever more precise while it keeps the offset, the
innovation gate cannot reject it, and a better source (RTK, map-based localization) makes the
estimate jump when it appears. This section describes the optional bias state that addresses it
(``gnss_bias_sigma_xy`` > 0 and the receiver's label in ``gnss_bias_labels_re``; default off).

**Measurement and process model.** For receiver :math:`r` with a bias state, a fix at time
:math:`t_k`, attached to keyframe :math:`k` (stamp :math:`t_k - \Delta t_k`), observes:

.. math::

   \mathbf{z}_k = {}^{enu}\mathbf{T}_{map} \; \mathbf{T}_k \; \mathbf{l}(\Delta t_k)
                  + \mathbf{b}_{r}(t_k) + \mathbf{n}_k, \qquad
   \mathbf{n}_k \sim \mathcal{N}(\mathbf{0}, \mathbf{R}_k)

with :math:`\mathbf{l}(\Delta t) = \mathbf{l} + \Delta t\,(\mathbf{v}_k + \boldsymbol{\omega}_k \times \mathbf{l})`
the antenna (lever arm :math:`\mathbf{l}`) moved to the fix time with the keyframe body twist, and
:math:`\mathbf{R}_k` only the white part of the receiver error (``gnss_bias_white_sigma_xy`` /
``_z`` replace the reported sigmas; the floors ``gnss_min_sigma_floor_*`` still apply). The bias
is a first-order Gauss-Markov process in ENU axes, one variable per fused fix:

.. math::

   \mathbf{b}_{r,k} = \boldsymbol{\Phi}_k \, \mathbf{b}_{r,k-1} + \mathbf{w}_k, \quad
   \boldsymbol{\Phi}_k = \mathrm{diag}(\phi_k, \phi_k, \phi_k^{z}), \quad
   \phi_k = e^{-\delta_k/\tau}, \quad
   \mathbf{w}_k \sim \mathcal{N}\left(\mathbf{0}, \boldsymbol{\Sigma}_b (1-\phi_k^2)\right)

.. math::

   \mathbf{b}_{r,0} \sim \mathcal{N}(\mathbf{0}, \boldsymbol{\Sigma}_b), \qquad
   \boldsymbol{\Sigma}_b = \mathrm{diag}(\sigma_{xy}^2, \sigma_{xy}^2, \sigma_{z}^2)

where :math:`\delta_k` is the time between the two fixes, :math:`\sigma_{xy}` =
``gnss_bias_sigma_xy``, :math:`\sigma_z` = ``gnss_bias_sigma_z`` and :math:`\tau` =
``gnss_bias_time_constant``. It is the sampled form of
:math:`\dot{\mathbf{b}} = -\mathbf{b}/\tau + \mathbf{w}(t)`, with
:math:`E[\mathbf{w}\mathbf{w}^\top] = (2/\tau)\,\boldsymbol{\Sigma}_b\,\delta(t)`: its marginal
variance never exceeds :math:`\boldsymbol{\Sigma}_b`, and the random walk is the limit
:math:`\tau \to \infty` with :math:`\sigma\sqrt{2/\tau}` per :math:`\sqrt{s}` fixed. If
:math:`\sigma_z = 0`, the vertical component is not a process: :math:`\phi_k^z = 0` and each variable
holds it at zero with a 1 mm prior. Links between (almost) simultaneous fixes are floored at
:math:`\sigma/1000`, so the chain stays factorable.

**Factor graph.** Variables (GTSAM keys): ``T(k)`` keyframe poses in ``{map}``, ``V(k)`` and
``W(k)`` their body linear and angular velocities, ``F(0)`` = :math:`{}^{enu}\mathbf{T}_{map}`
(never marginalized), and ``G(k)``, ``H(k)``, ``J(k)``, ``K(k)``: the bias (Point3, ENU) of up to
four receivers, at the keyframe of each of their fused fixes. Factors added by this feature:

- ``mola::factors::FactorGnssMapEnuBias`` on ``F(0), T(k), G(k)`` for a fix at its keyframe stamp,
  or ``FactorGnssMapEnuConstVelBias`` on ``F(0), T(k), V(k), W(k), G(k)`` otherwise; the Jacobian
  with respect to the bias is the identity. Robust kernel as for any GNSS fix.
- ``mola::factors::GaussMarkovPoint3::link()`` between consecutive bias variables of a receiver
  (``ExpressionFactor`` on :math:`\mathbf{b}_k - \boldsymbol{\Phi}\,\mathbf{b}_{k-1}`), and
  ``GaussMarkovPoint3::prior_noise()`` for the prior of the first one. Both are in
  ``mola_gtsam_factors``, for reuse by other graphs (e.g. a batch mapper).

The rest of the graph (odometry increments, kinematic, IMU and gravity factors) is unchanged. In
the fixed-lag smoother, the tail of each chain gets its key stamp refreshed at every solve, like
``F(0)`` and the odometry frames: it is never marginalized while it is the tail, so the chain, and
what it learned, survives GNSS outages of any length, and the next link uses the right
:math:`\delta_k`. The earlier bias variables are marginalized with the window. The graph can be
drawn from this Graphviz source (e.g. ``dot -Tsvg``), shown here as text only:

.. code-block:: text

   graph gnss_bias_state {
     rankdir=LR;
     node [shape=circle];
     Tk [label="T(k)"]; Tk1 [label="T(k+1)"]; Tk2 [label="T(k+2)"];
     F0 [label="T_enu_to_map\nF(0)"];
     Gk [label="G(k)"]; Gk2 [label="G(k+2)"];
     node [shape=box, width=0.15, height=0.15, label="", style=filled];
     odo1; odo2;      // odometry / kinematic factors
     gnss1; gnss2;    // FactorGnssMapEnuBias (fixes at k and k+2)
     prior; gm;       // GaussMarkovPoint3: prior on the first bias, link
     Tk -- odo1 -- Tk1;  Tk1 -- odo2 -- Tk2;
     F0 -- gnss1;  Tk -- gnss1;  Gk -- gnss1;
     F0 -- gnss2;  Tk2 -- gnss2; Gk2 -- gnss2;
     prior -- Gk;  Gk -- gm -- Gk2;
   }

**Observability.**

- *One receiver, estimated geo-reference.* The fixes depend on the translation of
  :math:`{}^{enu}\mathbf{T}_{map}` and on the bias only through their sum, so a constant offset can
  move from one to the other without changing any residual: the constant part of the bias is not
  observable, only regularized by the zero-mean prior (and by the prior that the first GNSS
  azimuth baseline puts on :math:`{}^{enu}\mathbf{T}_{map}`). Consequences: the absolute error
  cannot drop below the receiver's own offset, and the solved position sigma in ``{enu}`` stays
  near :math:`\sigma_{xy}` instead of shrinking like :math:`1/\sqrt{N}`, i.e. it is honest.
  ``convergence_max_position_sigma`` must then be above :math:`\sigma_{xy}`.
- *The time-varying part* is shared between the bias and the odometry drift according to their
  noise models, as in a complementary filter: the odometry carries the short time scales and GNSS
  the long ones. The cost is that odometry drifting at :math:`d` m/s leaves the bias about
  :math:`d\,\tau` away from zero instead of being corrected; the state only pays off while the
  odometry drifts well below :math:`\sigma_{xy}/\tau`. A bias estimate beyond
  ``gnss_bias_restart_sigmas`` stationary sigmas (horizontal Mahalanobis distance, plus vertical if
  estimated) is taken as odometry drift: the chain restarts from its prior at the next fix.
- *A second absolute reference* (RTK fixes from another receiver, or a fixed geo-reference with
  map-based localization) pins the translation, and the bias becomes observable: during the
  reference it is estimated to about the white sigma over :math:`\sqrt{N}`, and afterwards its
  variance relaxes as :math:`\phi^2(t)\,\mathrm{Var}_0 + \boldsymbol{\Sigma}_b\,(1-\phi^2(t))`, so the
  low-cost receiver keeps the estimate on the reference for a time of the order of :math:`\tau`.

**Innovation gate.** A fix of a receiver with a bias state is predicted as
:math:`h(\hat{\mathbf{x}}) + \phi\,\hat{\mathbf{b}}`, with :math:`\phi` from the time since the bias
estimate. While its fixes are in the window they pin the sum of position and bias much better
than either part, so only the bias change is added to the usual prediction variance,
:math:`(1-\phi)^2\,\mathrm{Var}(\hat{\mathbf{b}}) + (1-\phi^2)\,\boldsymbol{\Sigma}_b`; after a longer
gap, :math:`\phi^2\,\mathrm{Var}(\hat{\mathbf{b}}) + (1-\phi^2)\,\boldsymbol{\Sigma}_b`. Fixes of
other receivers use, per axis, at least the variance of the vehicle position in ``{enu}`` of the
last solve (joint marginal of :math:`{}^{enu}\mathbf{T}_{map}` and the pose, refreshed once per
second), so a better source is not locked out while the estimate is uncertain. Motion measures
of the gate (re-acquisition) use the prediction without the bias, and the fix-to-fix tests before
convergence add :math:`\sigma_{xy}` to the tolerance.

**Settings and validation.** For a u-blox 7 class receiver: ``gnss_bias_sigma_xy`` 2 m,
``gnss_bias_time_constant`` 60 s, ``gnss_bias_white_sigma_xy`` 0.3 m, ``gnss_bias_restart_sigmas``
3, the receiver's label in ``gnss_bias_labels_re`` (not the RTK one), and
``convergence_max_position_sigma`` above 2 m. Measured on three Husky A200 runs (9-38 min,
wheel odometry + 2 gyroscopes, LiDAR odometry, or both; the u-blox 7 fused, the RTK F9P as ground
truth), compared with the same configuration without the bias state:

- the reported position covered the actual error better in all 18 cases (fraction inside the
  95% ellipse, e.g. 0.55 to 0.98 with wheels and gyros on the longest run);
- with two 15 s windows of RTK fixes, the median error in the 3 minutes after each window fell
  from 1.37 to 0.45 m (longest run, wheels and gyros) and improved wherever the odometry was
  sound during that time; worst-case errors shrank in most cases;
- with the u-blox alone, the median error was 15-50% worse in every case: its offset is not
  observable and odometry drift leaks into the bias. It is not an accuracy improvement for a
  single low-cost receiver.

A known limitation: if the first GNSS azimuth baseline happens far from the ``{map}`` origin (e.g.
the odometry started far from where GNSS is first received, or after a ``reset()``) and only
receivers with a bias state are fused, the azimuth error of that baseline times the distance goes
into the bias. The first of the runs above is part of the public
`Husky UAL dataset <https://w3.ual.es/~jlblanco/datasets/husky-ual-lidar-inertial-dataset/>`_
(sequence 2026-11-16, with both receivers).
