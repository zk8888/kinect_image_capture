# kinect_image_capture
kinect v1 序列彩色、深度、点云图像采集与保存
## Preparation
* Ubuntu14.04及以上
* [安装ros](http://wiki.ros.org/indigo/Installation/Ubuntu)
* 安装OpenCV2.4.9及以上
* 安装PCL1.8

使用ros安装
```
$ cd ~/catkin_ws/src 
$ git clone https://github.com/zk8888/kinect_image_capture.git
$ cd ..
$ catkin_make
```
## 使用示例
打开终端1
```
$ roslaunch openni_launch openni.launch
```
打开终端2
```
$ rosrun kinect_img_capture kinect_img_capture your_save_path
```

