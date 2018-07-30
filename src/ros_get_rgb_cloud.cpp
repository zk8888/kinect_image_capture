//
// Created by zk8888 on 18-4-28.
//

// ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/PCLPointCloud2.h>

// OpenCV
#include <opencv2/opencv.hpp>

// C++
#include <iostream>

using namespace std;

ros::Publisher pub;

void callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::PointCloud2ConstPtr& input)
{
    cout << 222 << endl;
    time_t time1;
    time(&time1);
    std::stringstream ss;
    ss << time1;
    std::string name = ss.str();
    pcl::PointCloud<pcl::PointXYZ> temp;
    pcl::fromROSMsg(*input, temp);
    pcl::io::savePCDFile(name.substr(name.size()-6)+".pcd", temp);

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

//    cv::imshow("image", cv_ptr->image);
//    cv::waitKey(10);
    cv::imwrite(name.substr(name.size()-6)+".jpg", cv_ptr->image);
}

int
main(int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "my_pcl_tutorial");
    ros::NodeHandle nh;


//    ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);
    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/camera/rgb/image_rect_color", 1);  // topic1 输入
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, "/camera/depth_registered/points", 1); // topic2 输入
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> sync;
    message_filters::Synchronizer<sync> sync1(sync(10),image_sub,cloud_sub); //同步
    sync1.registerCallback(boost::bind(&callback, _1, _2));  //回调
//    pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);


    // Spin
    while (1){
        char s;
        cin >> s;
        ros::spinOnce();
    }

    return 0;
}