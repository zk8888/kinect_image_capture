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

std::string NAME = "";
static int NUM = -1;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> CloudT;
typedef CloudT::Ptr CloudPtr;

CloudPtr transImgToCloud(const cv::Mat &depth, const cv::Mat &rgb){
    // kinect serial: rgb_A00362A11150141A
    float fx = 535.05; // focal length x
    float fy = 536.34; // focal length y
    float cx = 322.04; // optical center x
    float cy = 266.97; // optical center y

    float factor;
    float unit_m_to_mm = 1000;
    if(depth.type()==CV_16UC1){
        factor = 4095;
    }else if(depth.type()==CV_8UC1){
        factor = 256*1.0/4096;
    }else if(depth.type()==CV_32FC1){
        factor = 1;
    }

    CloudPtr cloud(new CloudT());
    cloud->width = depth.cols;
    cloud->height = depth.rows;
    cloud->points.resize(cloud->width*cloud->height);
    cloud->is_dense = false;

    if(depth.type()==CV_16UC1){
        for(size_t i = 0; i < depth.rows; i++)
        {
            const ushort *tmp = depth.ptr<ushort>(i);
            for(size_t j = 0; j < depth.cols; j++){
                PointT p;
                p.z = float(tmp[j])/factor;
                p.x = (j-cx)*p.z/fx;
                p.y = (i-cy)*p.z/fy;
                cloud->points[i*depth.cols+j] = p;
            }
        }
    }else if(depth.type()==CV_32FC1){
        for(size_t i = 0; i < depth.rows; i++)
        {
            const float *tmp = depth.ptr<float>(i);
            for(size_t j = 0; j < depth.cols; j++){
                PointT p;
                p.z = float(tmp[j])/factor;
                p.x = (j-cx)*p.z/fx;
                p.y = (i-cy)*p.z/fy;
                cloud->points[i*depth.cols+j] = p;
            }
        }
    }else if(depth.type()==CV_8UC1){
        for(size_t i = 0; i < depth.rows; i++)
        {
            const uchar *tmp = depth.ptr<uchar>(i);
            for(size_t j = 0; j < depth.cols; j++){
                PointT p;
                p.z = float(tmp[j])/factor;
                p.x = (j-cx)*p.z/fx;
                p.y = (i-cy)*p.z/fy;
                cloud->points[i*depth.cols+j] = p;
            }
        }
    }
    else{
        throw;
    }

    if(rgb.size||rgb.type()==CV_8UC3){
        for(size_t i = 0; i < rgb.rows; ++i){
            const uchar *tmp = rgb.ptr<uchar>(i);
            for(size_t j = 0; j < rgb.cols; ++j){
                size_t index = i*640+j;
                cloud->points[index].b = tmp[j*3+0];
                cloud->points[index].g = tmp[j*3+1];
                cloud->points[index].r = tmp[j*3+2];
            }
        }
    }
    return cloud;
}

void CloudMtoMM(CloudPtr io_cloud){
    for(size_t ii = 0; ii < io_cloud->points.size(); ++ii){
        if(pcl::isFinite(io_cloud->points[ii])){
            io_cloud->points[ii].x *= 1000;
            io_cloud->points[ii].y *= 1000;
            io_cloud->points[ii].z *= 1000;
        }
    }
}

void callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::ImageConstPtr& depth)
{
    NUM++;
    std::stringstream ss;
    ss << NUM;
    std::string name = ss.str();
    name = "000000000000" + name;

    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(image);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::imwrite(NAME + '/' + name.substr(name.size()-6)+"_rgb.png", cv_ptr->image);

    cv_bridge::CvImageConstPtr cv_dpt_ptr;
    try
    {
        /// kinect topic查看
        cv_dpt_ptr = cv_bridge::toCvShare(depth);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat dpt32;
    cv_dpt_ptr->image.copyTo(dpt32);
    cv::imshow("depth32", dpt32);
    cv::imwrite(NAME + '/' + name.substr(name.size()-6)+"_dpt.exr", dpt32);

    CloudPtr cloud(new CloudT());
    cloud = transImgToCloud(cv_dpt_ptr->image, cv_ptr->image);
    CloudMtoMM(cloud);
    pcl::io::savePCDFileBinary(NAME + '/' + name.substr(name.size()-6)+".pcd", *cloud);
    cv::waitKey(1);
}

int
main(int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "my_pcl_tutorial");
    ros::NodeHandle nh;

    if(argc != 2){
        cout << "argc must be 2!" << endl;
        cout << "example: rosrun kinect_img_capture kinect_img_capture your_path" << endl;
        return -1;
    }

    NAME = argv[1];
    std::cout << "Image Save Path: " << NAME << std::endl;

    cv::namedWindow("depth32", cv::WINDOW_AUTOSIZE);

    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/camera/rgb/image_rect_color", 1);  // topic1 输入
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth_registered/sw_registered/image_rect", 1); // topic2 输入
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync;
    message_filters::Synchronizer<sync> sync1(sync(20),image_sub,depth_sub); //同步
    sync1.registerCallback(boost::bind(&callback, _1, _2));  //回调
//    pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
//    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/camera/rgb/image_rect_color", 1);  // topic1 输入
//    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth_registered/image", 1);  // topic1 输入
//    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(image_sub, depth_sub, 10);       // 同步
//    sync.registerCallback(boost::bind(&callback, _1, _2));      //回调

    // Spin
    while (true){
        cv::waitKey();
        cout << "continue" << endl;
        while (true){
            ros::spinOnce();
            // pause if enter any key in 30ms
            if(cv::waitKey(30)>=0)
            {
                cout << "pause" << endl;
                break;
            }
        }

    }

    return 0;
}