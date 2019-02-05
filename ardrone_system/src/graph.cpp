#include <ardrone_system/ardrone.h>
#include <ar_cv/SimpleCV.hpp>
#include <ar_cv/algorithms.hpp>
#include <ardrone_control/DroneController.h>
#include <cmath>

int DILATION_SIZE = 0;
int BLUR_SIZE = 0;

cv::Scalar circleColor(255, 255, 255), contourColor(0, 255, 0);

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "graph");
    ros::NodeHandle nh;
    drone ar(nh);
    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::Publisher circlePublisher = nh.advertise<ar_cv::CircleInfo>("/ar_cv/circle_info", 8);

    CameraWorkingInfo info;
    int freq = 20;          // in hz (image updating frequency)
    double accurancy = 0.1; // accurancy of circle detection

    // Read ROS parameters
    std::vector<int> mincolor(3), maxcolor(3);
    ros::param::get("/ar_cv/dilation_size", DILATION_SIZE);
    ros::param::get("/ar_cv/blur_size", BLUR_SIZE);
    ros::param::get("/ar_cv/sigma", accurancy);
    ros::param::get("/ar_cv/min_color", mincolor);
    ros::param::get("/ar_cv/max_color", maxcolor);
    ros::param::get("/ar_cv/topic_name", info.topicName);
    ros::param::get("/ar_cv/camera_number", info.deviceNumber);
    ros::param::get("/ar_cv/camera_frequency", freq);

    cv::Scalar minColor(mincolor[0], mincolor[1], mincolor[2]),
               maxColor(maxcolor[0], maxcolor[1], maxcolor[2]);

    if (info.topicName == "")
        info.workMode = 2;      // Config for web camera
    else info.workMode = 3;     // Config for topic

    std::vector<ar_cv::CircleInfo> circles;
    SimpleCV cv(nh, info, freq);
    cv::Mat image;
    std::vector<double> v;
    double alpha0 = M_PI/2504;
    double x0,y0,z0,R = 0.032 , n, l,x,y,z;                
    while(ros::ok()) {
        image = cv.getImage().clone();
        if (!image.empty()) {

            // Recognize circle with some accurancy
            circles = findCircles(image, minColor, maxColor, accurancy,
                DILATION_SIZE, BLUR_SIZE, circleColor, contourColor);

            if (circles.size() > 0){
                circlePublisher.publish(circles[0]);

                v=getCircleInfoForControl(circles[0]);
                x0 = v[0];
                y0 = v[1];
                z0 = v[2];
                n  = v[3];
                l = R/(sin(alpha0*n/2));
                x = l*cos(alpha0*z0)*cos(alpha0*y0);
                y = l*sin(alpha0*y0)*cos(alpha0*z0);
                z = l*sin(alpha0*z0);
                std::cout << "===================================\n";
                std::cout << "x: "<<x - 1 << "  y: "<<y << "  z: "<<z <<"  l: " <<l <<"\n";
                std::cout << "===================================\n";

            }

            #ifdef DEBUG
            cv::imshow("Image", image);
            #endif
        }
        cv::waitKey(1);
    }

    ros::waitForShutdown();
}
