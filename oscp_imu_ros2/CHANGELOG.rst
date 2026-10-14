^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package oscp_imu_ros2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.0.0 (2026-10-05)
------------------
* Merge pull request `#2 <https://github.com/OSCPS/oscp_ros2_driver/issues/2>`_ from OSCPS/ros-distro-support
  Specify supported ROS 2 distros (Humble, Jazzy)
* chores: removed invalid libstdcxx rosdep key from package.xml
* chores: humble and jazzy support - working on CI
* fix: separating cmd calls for explicit compiler order.
* typo: Update default.yaml to have LOW Operating Mode
* fix: added GNSS startup wait time and config
* cleanup: GetConfig response format matches config yaml file
* feature: Testing new magnetometer calibration + split calibration and writing functions
* chores: OSCP and MagCal Libraries + README Changes + ROS Message Timing Optimization
* feat: ROS Node IMU verification + Parameter Yaml Config Optimizations
* feat: MagCal Action Implementation + Services
* feat: Incorporated new oscp_library files
* Contributors: BlueMazer, thibautgravey
