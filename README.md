# DeepWater Explorer - ROS2 Parser


## Overview

Simple ROS2 package that reads in a DeepWater Explorer camera stream using V4L and publishes it to a ros topic. Camera settings and topic names can be set in the configuration file. 

Start with the following command:
```
ros2 launch dwe_ros2_parser dwe_ros2_parser.launch.py
```

## Dependencies



## How to find device number
Run to list available devices:

```
v4l2-ctl --list-devices 
```

Example output:
```
stellarHD Leader: stellarHD Lea (usb-0000:00:14.0-5.1):
	/dev/video0
	/dev/video1
	/dev/media0

stellarHD Leader: stellarHD Lea (usb-0000:00:14.0-5.3):
	/dev/video2
	/dev/video3
	/dev/media1

stellarHD Leader: stellarHD Lea (usb-0000:00:14.0-5.4):
	/dev/video4
	/dev/video5
	/dev/media2

```

The index of the first /dev/videoX of each camera can be used directly as the device parameter in the code. For example for the topmost camera in the example setting device index = 0 in the configuration will createa ros2 stream for that camera. Alternativley multiple camera streams can be added into one virtual streaming device using modprobe, and the stiched image can be published.

Command to create virtual camera:

```
sudo modprobe v4l2loopback video_nr=9 \
card_label=stellarHD_stitched exclusive_caps=1
```

Example stiching of three cameras:
```
gst-launch-1.0 -v \
compositor name=mix \
    sink_0::xpos=0    sink_0::ypos=0   sink_0::alpha=1 \
    sink_1::xpos=1600 sink_1::ypos=0   sink_1::alpha=1 \
    sink_2::xpos=3200 sink_2::ypos=0   sink_2::alpha=1 \
! jpegenc ! jpegdec ! videoconvert ! v4l2sink device=/dev/video9 \
v4l2src device=/dev/video0 ! image/jpeg,width=1600,framerate=60/1 ! jpegdec ! mix.sink_0 \
v4l2src device=/dev/video2 ! image/jpeg,width=1600,framerate=60/1 ! jpegdec ! mix.sink_1 \
v4l2src device=/dev/video4 ! image/jpeg,width=1600,framerate=60/1 ! jpegdec ! mix.sink_2
```