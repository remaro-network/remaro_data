# Ping360 sonar data recored at TU DELFT 3me pond [ROS2]

[Data link](https://figshare.com/s/d945f27cb6b9d66bc23d)

Sonar and MAVROS data recored in ros2 bag format using ROS.

Recordings were performed using a BlueROV2 robot equipped with Ping360 sonar.
ROS setup:
- ROS2 Foxy
- Ping360 sonar ROS2 package: https://github.com/CentraleNantesRobotics/ping360_sonar/tree/3ade26c7a6f22cc4f50488ae578282b36e150a04
- MAVROS: https://github.com/mavlink/mavros/commit/227b3accf7d16c7f0c468da25125e164de3ff292

Data was recorded at TU Delft 3me pond 25-05-2022.
This data will be used in the activities of REMARO (https://remaro.eu/) project.

Topics recorded:

```
Topic information: Topic: /mavros/wind_estimation | Type: geometry_msgs/msg/TwistWithCovarianceStamped | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/mission/reached | Type: mavros_msgs/msg/WaypointReached | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/vision_speed/speed_twist_cov | Type: geometry_msgs/msg/TwistWithCovarianceStamped | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/vision_pose/pose | Type: geometry_msgs/msg/PoseStamped | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/vision_pose/pose_cov | Type: geometry_msgs/msg/PoseWithCovarianceStamped | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/vfr_hud | Type: mavros_msgs/msg/VfrHud | Count: 1757 | Serialization Format: cdr
                   Topic: /mavros/vibration/raw/vibration | Type: mavros_msgs/msg/Vibration | Count: 528 | Serialization Format: cdr
                   Topic: /mavros/terrain/report | Type: mavros_msgs/msg/TerrainReport | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/debug_value/debug | Type: mavros_msgs/msg/DebugValue | Count: 0 | Serialization Format: cdr
                   Topic: /diagnostics | Type: diagnostic_msgs/msg/DiagnosticArray | Count: 371 | Serialization Format: cdr
                   Topic: /mavros/global_position/gp_origin | Type: geographic_msgs/msg/GeoPointStamped | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/global_position/compass_hdg | Type: std_msgs/msg/Float64 | Count: 528 | Serialization Format: cdr
                   Topic: /mavros/log_transfer/raw/log_data | Type: mavros_msgs/msg/LogData | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/camera/image_captured | Type: mavros_msgs/msg/CameraImageCaptured | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/global_position/raw/fix | Type: sensor_msgs/msg/NavSatFix | Count: 162 | Serialization Format: cdr
                   Topic: /mavros/mocap/tf | Type: geometry_msgs/msg/TransformStamped | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/global_position/global | Type: sensor_msgs/msg/NavSatFix | Count: 528 | Serialization Format: cdr
                   Topic: /mavros/obstacle/send | Type: sensor_msgs/msg/LaserScan | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/log_transfer/raw/log_entry | Type: mavros_msgs/msg/LogEntry | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/imu/temperature_imu | Type: sensor_msgs/msg/Temperature | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/setpoint_raw/global | Type: mavros_msgs/msg/GlobalPositionTarget | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/hil/controls | Type: mavros_msgs/msg/HilControls | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/gps_rtk/send_rtcm | Type: mavros_msgs/msg/RTCM | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/setpoint_position/global_to_local | Type: geographic_msgs/msg/GeoPoseStamped | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/global_position/raw/gps_vel | Type: geometry_msgs/msg/TwistStamped | Count: 162 | Serialization Format: cdr
                   Topic: /mavros/local_position/pose_cov | Type: geometry_msgs/msg/PoseWithCovarianceStamped | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/manual_control/control | Type: mavros_msgs/msg/ManualControl | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/play_tune | Type: mavros_msgs/msg/PlayTuneV2 | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/landing_target/pose | Type: geometry_msgs/msg/PoseStamped | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/esc_telemetry/telemetry | Type: mavros_msgs/msg/ESCTelemetry | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/global_position/rel_alt | Type: std_msgs/msg/Float64 | Count: 528 | Serialization Format: cdr
                   Topic: /mavros/esc_status/info | Type: mavros_msgs/msg/ESCInfo | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/global_position/raw/satellites | Type: std_msgs/msg/UInt32 | Count: 162 | Serialization Format: cdr
                   Topic: /mavros/odometry/out | Type: nav_msgs/msg/Odometry | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/actuator_control | Type: mavros_msgs/msg/ActuatorControl | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/time_reference | Type: sensor_msgs/msg/TimeReference | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/state | Type: mavros_msgs/msg/State | Count: 186 | Serialization Format: cdr
                   Topic: /mavros/adsb/vehicle | Type: mavros_msgs/msg/ADSBVehicle | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/altitude | Type: mavros_msgs/msg/Altitude | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/geofence/fences | Type: mavros_msgs/msg/WaypointList | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/companion_process/status | Type: mavros_msgs/msg/CompanionProcessStatus | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/debug_value/send | Type: mavros_msgs/msg/DebugValue | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/mount_control/orientation | Type: geometry_msgs/msg/Quaternion | Count: 528 | Serialization Format: cdr
                   Topic: /mavros/imu/temperature_baro | Type: sensor_msgs/msg/Temperature | Count: 353 | Serialization Format: cdr
                   Topic: /mavros/cam_imu_sync/cam_imu_stamp | Type: mavros_msgs/msg/CamIMUStamp | Count: 0 | Serialization Format: cdr
                   Topic: /uas1/mavlink_source | Type: mavros_msgs/msg/Mavlink | Count: 25259 | Serialization Format: cdr
                   Topic: /mavros/setpoint_position/local | Type: geometry_msgs/msg/PoseStamped | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/local_position/velocity_local | Type: geometry_msgs/msg/TwistStamped | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/gps_rtk/rtk_baseline | Type: mavros_msgs/msg/RTKBaseline | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/home_position/set | Type: mavros_msgs/msg/HomePosition | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/imu/data | Type: sensor_msgs/msg/Imu | Count: 3513 | Serialization Format: cdr
                   Topic: /mavros/landing_target/pose_in | Type: geometry_msgs/msg/PoseStamped | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/local_position/pose | Type: geometry_msgs/msg/PoseStamped | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/imu/mag | Type: sensor_msgs/msg/MagneticField | Count: 353 | Serialization Format: cdr
                   Topic: /mavros/battery | Type: sensor_msgs/msg/BatteryState | Count: 528 | Serialization Format: cdr
                   Topic: /uas1/mavlink_sink | Type: mavros_msgs/msg/Mavlink | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/trajectory/path | Type: nav_msgs/msg/Path | Count: 0 | Serialization Format: cdr
                   Topic: /rosout | Type: rcl_interfaces/msg/Log | Count: 140 | Serialization Format: cdr
                   Topic: /mavros/debug_value/named_value_float | Type: mavros_msgs/msg/DebugValue | Count: 2471 | Serialization Format: cdr
                   Topic: /mavros/wheel_odometry/odom | Type: nav_msgs/msg/Odometry | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/hil/optical_flow | Type: mavros_msgs/msg/OpticalFlowRad | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/home_position/home | Type: mavros_msgs/msg/HomePosition | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/setpoint_trajectory/desired | Type: nav_msgs/msg/Path | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/mag_calibration/status | Type: std_msgs/msg/UInt8 | Count: 0 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 1 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/gps_input/gps_input | Type: mavros_msgs/msg/GPSINPUT | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/vision_speed/speed_vector | Type: geometry_msgs/msg/Vector3Stamped | Count: 0 | Serialization Format: cdr
                   Topic: /scan_image | Type: sensor_msgs/msg/Image | Count: 1833 | Serialization Format: cdr
                   Topic: /mavros/px4flow/ground_distance | Type: sensor_msgs/msg/Range | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/hil/gps | Type: mavros_msgs/msg/HilGPS | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/landing_target/lt_marker | Type: geometry_msgs/msg/Vector3Stamped | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/setpoint_position/global | Type: geographic_msgs/msg/GeoPoseStamped | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/extended_state | Type: mavros_msgs/msg/ExtendedState | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/hil/rc_inputs | Type: mavros_msgs/msg/RCIn | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/px4flow/temperature | Type: sensor_msgs/msg/Temperature | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/target_actuator_control | Type: mavros_msgs/msg/ActuatorControl | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/hil/actuator_controls | Type: mavros_msgs/msg/HilActuatorControls | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/setpoint_accel/accel | Type: geometry_msgs/msg/Vector3Stamped | Count: 0 | Serialization Format: cdr
                   Topic: /scan | Type: sensor_msgs/msg/LaserScan | Count: 22 | Serialization Format: cdr
                   Topic: /mavros/hil/imu_ned | Type: mavros_msgs/msg/HilSensor | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/setpoint_raw/local | Type: mavros_msgs/msg/PositionTarget | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/global_position/set_gp_origin | Type: geographic_msgs/msg/GeoPointStamped | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/vision_speed/speed_twist | Type: geometry_msgs/msg/TwistStamped | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/local_position/velocity_body | Type: geometry_msgs/msg/TwistStamped | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/esc_status/status | Type: mavros_msgs/msg/ESCStatus | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/px4flow/raw/optical_flow_rad | Type: mavros_msgs/msg/OpticalFlowRad | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/debug_value/named_value_int | Type: mavros_msgs/msg/DebugValue | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/mag_calibration/report | Type: mavros_msgs/msg/MagnetometerReporter | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/onboard_computer/status | Type: mavros_msgs/msg/OnboardComputerStatus | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/rangefinder/rangefinder | Type: sensor_msgs/msg/Range | Count: 528 | Serialization Format: cdr
                   Topic: /mavros/gpsstatus/gps1/rtk | Type: mavros_msgs/msg/GPSRTK | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/imu/data_raw | Type: sensor_msgs/msg/Imu | Count: 353 | Serialization Format: cdr
                   Topic: /mavros/gpsstatus/gps2/rtk | Type: mavros_msgs/msg/GPSRTK | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/global_position/local | Type: nav_msgs/msg/Odometry | Count: 528 | Serialization Format: cdr
                   Topic: /mavros/fake_gps/mocap/tf | Type: geometry_msgs/msg/TransformStamped | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/local_position/accel | Type: geometry_msgs/msg/AccelWithCovarianceStamped | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/setpoint_raw/attitude | Type: mavros_msgs/msg/AttitudeTarget | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/manual_control/send | Type: mavros_msgs/msg/ManualControl | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/mocap/pose | Type: geometry_msgs/msg/PoseStamped | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/mount_control/command | Type: mavros_msgs/msg/MountControl | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/mount_control/status | Type: geometry_msgs/msg/Vector3Stamped | Count: 528 | Serialization Format: cdr
                   Topic: /mavros/imu/diff_pressure | Type: sensor_msgs/msg/FluidPressure | Count: 353 | Serialization Format: cdr
                   Topic: /mavros/rc/override | Type: mavros_msgs/msg/OverrideRCIn | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/adsb/send | Type: mavros_msgs/msg/ADSBVehicle | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/gpsstatus/gps1/raw | Type: mavros_msgs/msg/GPSRAW | Count: 162 | Serialization Format: cdr
                   Topic: /mavros/odometry/in | Type: nav_msgs/msg/Odometry | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/nav_controller_output/output | Type: mavros_msgs/msg/NavControllerOutput | Count: 353 | Serialization Format: cdr
                   Topic: /mavros/param/event | Type: mavros_msgs/msg/ParamEvent | Count: 918 | Serialization Format: cdr
                   Topic: /mavros/imu/static_pressure | Type: sensor_msgs/msg/FluidPressure | Count: 353 | Serialization Format: cdr
                   Topic: /mavros/px4flow/raw/send | Type: mavros_msgs/msg/OpticalFlowRad | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/global_position/gp_lp_offset | Type: geometry_msgs/msg/PoseStamped | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/rallypoint/rallypoints | Type: mavros_msgs/msg/WaypointList | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/rc/in | Type: mavros_msgs/msg/RCIn | Count: 353 | Serialization Format: cdr
                   Topic: /mavros/debug_value/debug_vector | Type: mavros_msgs/msg/DebugValue | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/rc/out | Type: mavros_msgs/msg/RCOut | Count: 353 | Serialization Format: cdr
                   Topic: /parameter_events | Type: rcl_interfaces/msg/ParameterEvent | Count: 1033 | Serialization Format: cdr
                   Topic: /mavros/estimator_status | Type: mavros_msgs/msg/EstimatorStatus | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/setpoint_attitude/thrust | Type: mavros_msgs/msg/Thrust | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/setpoint_attitude/cmd_vel | Type: geometry_msgs/msg/TwistStamped | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/mission/waypoints | Type: mavros_msgs/msg/WaypointList | Count: 1 | Serialization Format: cdr
                   Topic: /mavros/setpoint_raw/target_attitude | Type: mavros_msgs/msg/AttitudeTarget | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/setpoint_raw/target_global | Type: mavros_msgs/msg/GlobalPositionTarget | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/setpoint_raw/target_local | Type: mavros_msgs/msg/PositionTarget | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/setpoint_trajectory/local | Type: trajectory_msgs/msg/MultiDOFJointTrajectory | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/gpsstatus/gps2/raw | Type: mavros_msgs/msg/GPSRAW | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/setpoint_velocity/cmd_vel | Type: geometry_msgs/msg/TwistStamped | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/setpoint_velocity/cmd_vel_unstamped | Type: geometry_msgs/msg/Twist | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/statustext/recv | Type: mavros_msgs/msg/StatusText | Count: 3 | Serialization Format: cdr
                   Topic: /mavros/local_position/velocity_body_cov | Type: geometry_msgs/msg/TwistWithCovarianceStamped | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/radio_status | Type: mavros_msgs/msg/RadioStatus | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/statustext/send | Type: mavros_msgs/msg/StatusText | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/local_position/odom | Type: nav_msgs/msg/Odometry | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/timesync_status | Type: mavros_msgs/msg/TimesyncStatus | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/hil/state | Type: mavros_msgs/msg/HilStateQuaternion | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/tunnel/out | Type: mavros_msgs/msg/Tunnel | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/tunnel/in | Type: mavros_msgs/msg/Tunnel | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/trajectory/generated | Type: mavros_msgs/msg/Trajectory | Count: 0 | Serialization Format: cdr
                   Topic: /mavros/trajectory/desired | Type: mavros_msgs/msg/Trajectory | Count: 0 | Serialization Format: cdr

```
