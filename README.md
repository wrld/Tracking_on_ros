# Tracking_on_ros
## introduction
- Use KCF/GOTURN/TLD with YOLOv3 to multiple track and data association(not complete), in case of losing targets.
- Use darknet_ros node for YOLOv3 to detect objects, and publish information of result boxes to tracking node.
- Use usb_cam node to drive the camera to detect online, and publish images to darknet node and tracking node.
- Use video_pub node to read the video and publish frames to darknet node and tracking node.
## platform
- Ubuntu 16.04
- ros kinetic
- opencv 3.4.1+
- if using yolov3 tracking, you need to install Nvidia driver, CUDA, CUDNN
## use
- First publish images
``` shell
roslaunch video_pub video_pub.launch
```
or
``` shell
roslaunch usb_cam usb_cam-test.launch
```
- First run darknet_ros node
``` shell
roslaunch darknet_ros darknet_ros.launch
```
- Then run tracking node
``` shell
roslaunch tracking tracking.launch 
```
## result
