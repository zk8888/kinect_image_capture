# kinect_image_capture
Capture and save color, depth and point cloud images from Kinect V1.
## Preparation
* Ubuntu14.04
* [install ros](http://wiki.ros.org/indigo/Installation/Ubuntu)
* install OpenCV2.4.9
* install PCL1.8

Install with ros
```
$ cd ~/catkin_ws/src 
$ git clone https://github.com/zk8888/kinect_image_capture.git
$ cd ..
$ catkin_make
```
## Example
Open terminal 1
```
$ roslaunch openni_launch openni.launch
```
Open terminal 2
```
$ rosrun kinect_img_capture kinect_img_capture your_save_path
```


