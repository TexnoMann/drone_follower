#pragma once

// include OpenCV header file
#include <opencv2/opencv.hpp>

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

// Boost
#include <boost/thread/thread.hpp>

struct CameraWorkingInfo
{
    // 1 - from image;
    // 2 - using onboard web camera
    // 3 - work with topic
    int workMode;

    // If selected mode 1
    std::string imagePath;

    // If selected mode 2
    int deviceNumber;

    // If selected mode 3
    std::string topicName;
    std::string messageType;
};

class SimpleCV {

public:
	SimpleCV(ros::NodeHandle & nh, CameraWorkingInfo info, int frequency);
	~SimpleCV();

	cv::Mat getImage() const
	{return image;}

private:
    // Image updaters
    // The frequency of working updater is equal to freq
    void updateImageByOpenCV();
    void imageCallback(const sensor_msgs::Image::ConstPtr & msg);

    // Pub/Sub
    ros::Subscriber cameraSubscriber;

    // Global Node handle
    ros::NodeHandle n;

    std::string packagePath;

    // Main circle frequency in hz
    int freq;

    // Same as CameraWorkingInfo work mode
    int workMode;

    // Actual image from camera
    cv::Mat image;

    // Actual device number of camera
    int devNum;

    // thread
    boost::thread thr;

    // CvBridge image
    cv_bridge::CvImagePtr cv_ptr;

    // Camera resolution
    double camWidth, camHeight;
    // Camera m/px coefficients
    double kx, ky;

};