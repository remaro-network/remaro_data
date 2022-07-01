# Ping360 sonar data recored at De Salamander windmill
[Data link](https://figshare.com/s/d3cde14a83a61c3c442b)


Underwater sonar data and camera images recored in rosbag format using ROS. Recordings were performed using a BlueROV2 robot equipped with PIng360 sonar and standard camera.

ROS setup:
- ROS Kinetic
- Ping360 sonar ROS packaged: ttps://github.com/CentraleNantesRobotics/ping360_sonar/tree/aa48eb6e5ae2650e5cf1d270e0fe4ce8fb3c8b0f

Morning data: default BlueROV2 configuration
Afternoon data: BlueROV2 customized as represented in figure 'bluerov_customized.jpg' in afternoon zip file

Topics recorded:

```
Published topics:
 * /rosout_agg [rosgraph_msgs/Log] 1 publisher
 * /rosout [rosgraph_msgs/Log] 1 publisher
 * /clock [rosgraph_msgs/Clock] 1 publisher
 * /ping360_node/parameter_descriptions [dynamic_reconfigure/ConfigDescription] 1 publisher
 * /imu/data [sensor_msgs/Imu] 1 publisher
 * /ping360_node/sonar/data [ping360_sonar/SonarEcho] 1 publisher
 * /velocity [geometry_msgs/TwistStamped] 1 publisher
 * /bluerov_camera/camera_info [sensor_msgs/CameraInfo] 1 publisher
 * /ping360_node/sonar/scan [sensor_msgs/LaserScan] 1 publisher
 * /tf [tf2_msgs/TFMessage] 1 publisher
 * /imu/mag [sensor_msgs/MagneticField] 1 publisher
 * /ping360_node/parameter_updates [dynamic_reconfigure/Config] 1 publisher
 * /imu_data_str [std_msgs/String] 1 publisher
 * /ping360_node/sonar/images [sensor_msgs/Image] 1 publisher
 * /bluerov_camera/image_raw [sensor_msgs/Image] 1 publisher
 * /bluerov_camera/bluerov_camera_stream/parameter_descriptions [dynamic_reconfigure/ConfigDescription] 1 publisher
 * /bluerov_camera/bluerov_camera_stream/parameter_updates [dynamic_reconfigure/Config] 1 publisher
```


Data was recorded at  De Salamander windmill on 28-03-2022.
This data will be used in the activities of REMARO (https://remaro.eu/) project.

[Youtube videos](https://www.youtube.com/watch?v=KYdHnL1VCK8&list=PLV9uZDMC62kCDQPCyR0CRxLXrnigyYUhN&index=1&t=1s)
![Screenshot from 2022-05-26 11-29-22](https://user-images.githubusercontent.com/20564040/170460443-cafb0ec3-8c96-44eb-af0b-37e32c85b746.png)

