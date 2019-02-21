## Sensor Fusion Sequence

### General ROS setup:

​	 `roscore`

Start RQT GUI (This is a GUI for visualization of ROS Topics, networks, Structures and other useful things.): 

​	`rosrun rqt_gui rqt_gui`

(Optional) Start Rviz for 3D visualization of poses: 

​	`rosrun rviz rviz`

### Camera:

Start Camera(s) using cv_camera package or marker_based_localization package's launch file: `video_capture.launch` (dependent on cv_camera package). 

Resolution, path to camera calibration file , camera index and other settings can be changed within the launch file.

The output image is on the topic `/camera/image_raw`. 

### ORB SLAM

Start ORB SLAM : 

​	`rosrun ORB_SLAM2 Mono ../Vocabulary/ORBvoc.txt ../Examples/Monocular/GoPro-3-Drone-1720p.yaml`

ORB SLAM is configured to read the topic  `/camera/image_raw` and publish a odometry topic called `/MonoPose`. 

Monopose currently publishes only pose info within the odometry topic, but can also be configured to include twist and pose covariances in order to support sensor fusion.

### IMU

To get the IMU data, connect to the Solo's Wifi Network and start the mavros package with apm.launch

`roslaunch mavros apm.launch`

Data from the drone can be configured using the `apm_config.yaml` file located in the mavros package's `/mavros/launch` directory

The IMU's measurement variances can be set here in this file. The set variances are what will be published in the IMU data topic. It is recommended to measure the actual variance of the IMU topic.

### Sensor Fusion

Start sensor fusion node using : 

​	`roslaunch robot_localization ekf_template_orbslam.launch`

The various settings for sensor fusion can be changed in `/params/ekf_orbslam.yaml`.

The yaml file has general documentation on how to change the settings.

## Package Descriptions

### `marker_based_localization` package:

This package contains various tools I wrote for visualization and ease of testing such as :

- Displaying paths from various topics in rviz: `pathpublisherclassall.py`
- Transforming pose/tf data from one form to another: `posetransformer.py`, `posetoodom.py`
- Dynamic pose adjustment within RQT_GUI. Useful for manually zeroing poses after initialization : `poseadjuster.py`
- Converting any topic type with quaternion rotation to euler angles: `/launch/eulerangles.launch , quattoeuler.py `
- Removing Constant starting biases from IMU: `remove_IMU_bias.py`

### `vrpn_client_ros` package:

Use the launch file to set the correct IP address to Optitrack computer and topic names

### `take_off test` package

Contains some old code I wrote while working on the drone control

### Fiducials package:

Contains the Aruco detection and Marker SLAM package. Use if needed.