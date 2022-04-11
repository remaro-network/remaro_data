# Ping360 sonar data recored at TU DELFT 3me pond

[Data link](https://figshare.com/s/e61264438d5ee481829b)

Sonar data recored in rosbag format using ROS. Recordings were performed using a BlueROV2 robot equipped with PIng360 sonar.

ROS setup:
- ROS Kinetic
- Ping360 sonar ROS packaged: ttps://github.com/CentraleNantesRobotics/ping360_sonar/tree/aa48eb6e5ae2650e5cf1d270e0fe4ce8fb3c8b0f

Topics recorded:

```
Published topics:
 * /rosout_agg [rosgraph_msgs/Log] 2 publishers
 * /rosout [rosgraph_msgs/Log] 1 publisher
 * /clock [rosgraph_msgs/Clock] 1 publisher
 * /ping360_node/parameter_descriptions [dynamic_reconfigure/ConfigDescription] 1 publisher
 * /tf [tf2_msgs/TFMessage] 1 publisher
 * /ping360_node/sonar/data [ping360_sonar/SonarEcho] 1 publisher
 * /ping360_node/sonar/scan [sensor_msgs/LaserScan] 1 publisher
 * /ping360_node/parameter_updates [dynamic_reconfigure/Config] 1 publisher
 * /ping360_node/sonar/images [sensor_msgs/Image] 1 publisher
```


Data was recorded at TU Delft 3me pond 25-03-2022.
This data will be used in the activities of REMARO (https://remaro.eu/) project.
