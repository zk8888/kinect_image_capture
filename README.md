# kinect_image_capture
[![License](https://img.shields.io/packagist/l/doctrine/orm.svg)](LICENSE)

Capture and save color, depth and point cloud images from Kinect V1.
## Preparation
* Ubuntu 14.04
* [Install ROS](http://wiki.ros.org/indigo/Installation/Ubuntu)
* Install OpenCV 2.4.9
* Install PCL 1.8

Install in ROS
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
Click on the opened window and press any key to pause or continue recording images.

![Demo](https://github.com/zk8888/kinect_image_capture/raw/master/example.gif)
## Authors
* **Kai Zhang** - *Initial work* - Kai Zhang


