# Dartmouth Robotics lab procedure

This repo explains how to calibrate OS1-64 or Velodyne LiDAR with monocular camera at Dartmouth Robotics lab.
## Author
* Mingi Jeong


## Requirements
1. Ubuntu 16.04, 18.04
2. ROS Kinetic, Melodic on host

### Camera intrinsic calibration
http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration
http://wiki.ros.org/camera_calibration_parsers

```
rosparam set autoexposure true
rosparam set auto_focus false
rosparam set usb_cam/video_device /dev/video1
rosparam set usb_cam/image_height 480
rosparam set usb_cam/image_width 640
rosparam set usb_cam/pixel_format mjpeg
rosparam set /usb_cam/camera_frame_id usb_cam
```

```
rosrun usb_cam usb_cam_node

rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108 image:=/camera/image_raw camera:=/camerarosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108 image:=/usb_cam/image_raw camera:=/usb_cam
```
* move the board in front of the camera and you will see the gauge go up
* once it shows a calibration button, we can click. Then click save --> goes to /tmp/calibrationfile.tar.gz
* extract .tar.gz file 
* mv ost.txt ost.ini
* rosrun  camera_calibration_parsers convert  ost.ini cam_with_ouster.yml


## Directory Tree
```
.
├── cfg
│   ├── Lidar.cfg
│   ├── Monocular.cfg
│   ├── Plane.cfg
│   └── Stereo.cfg
├── CITATION.cff
├── CMakeLists.txt
├── HOWTO.md
├── include
│   └── velo2cam_utils.h
├── launch
│   ├── lidar_pattern.launch
│   ├── mono_pattern.launch
│   ├── ouster
│   ├── registration.launch
│   ├── remap.launch
│   ├── stereo_pattern.launch
│   └── velodyne
├── LICENSE.md
├── msg
│   └── ClusterCentroids.msg
├── package.xml
├── README.md
├── RLAB.md
├── screenshots
│   ├── calibration_target_real_scheme_journal.png
│   ├── lidar_filters_1.png
│   ├── lidar_filters_2.png
│   ├── real_results.png
│   ├── stereo_filters_1.png
│   └── stereo_filters_2.png
└── src
    ├── disp_masker.cpp
    ├── lidar_pattern.cpp
    ├── mono_qr_pattern.cpp
    ├── plane.cpp
    ├── stereo_pattern.cpp
    └── velo2cam_calibration.cpp
```

## Build / Setup
Please refer to the original [README.md](README.md).
* Ouster driver: https://github.com/ouster-lidar/ouster-ros\
The driver was origianlly contained and built by `ouster_example` repo but not they are separated.
* Make sure you conducted `intrinsic` calibration first and the camera node is executed based on that __yaml__ file.

## Run
The following example shows `OS1-64` for lidar.\
Please look at the example images which I did for `VLP-16`. https://www.dropbox.com/scl/fo/k5ypzuyx7zki7jfx0jfj8/h?dl=0&rlkey=21714k1m6ui5he4h7zburidt9

### 1. LiDAR driver running
Note that `udp_dest` is our host computer IP address which can be found in the network setting menu or by __wireshark__.\
Note that the `os_cloud_node/points` use point clouds wrt `os_sensor` frame where __x__ axis faces forward.
old driver. should be replaced.
```
roslaunch ouster_ros ouster.launch sensor_hostname:=os-992121000445.local udp_dest:=169.254.10.231 metadata:=/home/minkbrook/Desktop/test.json lidar_mode:=1024x10 viz:=false
```
If you are not using a fixed IP, you can find it via `wireshark` for `udp_dest`.

### 2. Camera driver running

```
rosparam set autoexposure true
rosparam set auto_focus false
rosparam set usb_cam/video_device /dev/video1
rosparam set usb_cam/image_height 480
rosparam set usb_cam/image_width 640
rosparam set usb_cam/pixel_format mjpeg
rosparam set /usb_cam/camera_frame_id usb_cam
```

Note that I falied to make it operate well by using my custom `usb_cam` launch. 
```
rosrun usb_cam usb_cam_node _pixel_format:=yuyv _video_device:=/dev/video0 _camera_frame_id:=usb_cam _camera_name:=usb_cam
```
* Make sure you set up a proper parameter for camera (frame_id, video_device, format, autoexposure..).
e.g., rosparam set /usb_cam/video_device /dev/video1 (depending on `v4cl device list`)
* Make sure that you run correct `.yaml` file for intrinsic calibration

###  3. Setup the environment
Install properly the calibration board. Use clamps and have a sufficient distance off from the wall.

### 4. Camera calibration
You should see `open-cv` popup with 4 Aruco Markers and 4 center circles detected. Aruco Marker numbers are [[1,2],[3,4]] from the top.
```
roslaunch velo2cam_calibration mono_pattern.launch camera_name:=/usb_cam image_topic:=image_raw frame_name:=usb_cam
```
```
rviz
```
```
rosrun rqt_reconfigure rqt_reconfigure
```
* adjust the marker size and qr width, height distance.


### 5. LiDAR calibration


```
roslaunch velo2cam_calibration lidar_pattern.launch cloud_topic:=/ouster/points
```
* The lidar_pattern `lidar_pattern_/zyx_filtered` cloud should represent only the calibration pattern and the points corresponding to those lidar rays that go through the holes. 
Thecloud can be filtered through the parameters filter_limit_min and filter_limit_max of the `pass_through_x_velo_, pass_through_y_velo_, and pass_through_z_velo`.
* On the other hand, `lidar_pattern_/range_filtered_cloud` must contain only the calibration pattern. To that end, a radial passthrough filter is available in the lidar_pattern_ node, tunable through the `passthrough_radius_min` and `passthrough_radius_max` parameters. Basically, this changes the ring part generated by __red__ while excluding __white__

![running result](screenshots/dart-20221219_1.png)
Note, however, that the filters do not need an exact tuning as long as the areas of interest (i.e., the pattern and the surface behind it) are well defined in the LiDAR clouds.

### 6. pattern matching

Once you pass the camera and LiDAR matching pstep, 

```
roslaunch velo2cam_calibration registration.launch sensor1_type:=mono sensor2_type:=lidar
```
Make sure to use correct __camera_name__ and __topic__.

When the registration is finished with warm-up, it will ask you like warm-up finish`[Y/N]` and if Y typed, it will conduct calibration. Once done, it will ask whether to conduct another location. If N, it will terminate the calibration and you will get a new launch file with TF defined between the camera and lidar.


### 7. Record a bag file

```
rosbag record /mono_pattern_0/centers_cloud /mono_pattern_0/centers_pts_cloud /mono_pattern_0/cumulative_cloud /mono_pattern_0/parameter_descriptions /mono_pattern_0/parameter_updates /mono_pattern_0/qr_cloud /move_base_simple/goal /os_node/imu_packets /os_node/lidar_packets /ouster/imu /ouster/imu_packets /ouster/lidar_packets /ouster/nearir_image /ouster/os_nodelet_mgr/bond /ouster/points /ouster/range_image /ouster/reflec_image /ouster/signal_image /rosout /rosout_agg /tf /tf_static /usb_cam/camera_info /usb_cam/image_raw/compressed /usb_cam/image_raw/compressed/parameter_descriptions /usb_cam/image_raw/compressed/parameter_updates /warmup_switch 

```

![running result_gif](screenshots/dart-20221219_result.gif)
The result will be saved in `launch` folder with time marked.

### 8. Run the tf launch
* Run the tf launch and get the extrinsic by `tf_echo`


## TODO 
1. custom `usb_cam` with correct parameters including parameter
2. 