# Depthai ROS Repository
I have done the following things so far. 

* Use depthai_ros_driver to publish stereo images, imu data, and yolo BBox detection on RGB
* Stored rosbag (`.bag`) (can be found [here](https://drive.google.com/drive/u/1/folders/1PGnxorSoP_wxfEeHzICimydF3Ft5Acr4))

## Sensor parameter
See `depthai_ros_driver/config/stereo_inertial_hd.yaml` for detail. <br>
Please refer to `depthai_ros_driver/cfg/parameters.cfg` for further info about each rosparameter.

* Stereo Images
  * 400p (640x400)
  * 720p
  * shutter speed 10ms
* IMU
  * gyro: 200Hz
  * accel: 200Hz
  * unite_mode: LINEAR_INTERPOLATE_ACCEL
* Yolo BBox
  * in depthai custom message
  * BBox is collected by inference on RGB 1080p(1920x1080)image
  * BBox must be properly scaled&translated to align with left image

To get rosbag, execute the following commands

```
#1st terminal
cd ~/catkin_ws/
source devel/setup.bash
roslaunch depthai_ros_driver stereo_inertial_vga.launch
```

```
#2nd terminal 
cd ~/catkin_ws/
source devel/setup.bash
roslaunch depthai_ros_driver record_stereo_inertial.launch
```

## Coordinate
TF tree
<img src="images/oakD_frame.png">

Note that IMU optical frame to left camera optical frame is not identity rotation! (Different from RealSense D435i).

```
rosrun tf tf_echo oak_left_camera_optical_frame oak_imu_frame
```
And here is the output
```
At time 0.000
- Translation: [0.052, 0.014, -0.000]
- Rotation: in Quaternion [-0.000, 0.000, 0.707, 0.707]
            in RPY (radian) [0.000, 0.000, 1.571]
            in RPY (degree) [0.000, 0.000, 90.000]
```