#include <ardrone_system/ardrone.h>
#include <ar_cv/SimpleCV.hpp>
#include <ar_cv/algorithms.hpp>
#include <ardrone_control/DroneController.h>

int DILATION_SIZE = 0;
int BLUR_SIZE = 0;

cv::Scalar circleColor(255, 255, 255), contourColor(0, 255, 0);
//cv::Scalar minColor(98, 140, 140), maxColor(218, 255, 255);
DroneController ctrl(0.802, 0.1, 0.1, 0.1, 1280.0, 720.0);

int main(int argc, char **argv){
	float x,y,z,zz,k = 0.5;
	ros::init(argc, argv, "ardrone");
	// drone ar;
	// ros::NodeHandle nh;
 //    ros::AsyncSpinner spinner(2);
 //    spinner.start();

 //    ros::Publisher circlePublisher = nh.advertise<ar_cv::CircleInfo>("/ar_cv/circle_info", 8);
	// // Configure camera work
 //    // Work modes::
 //    // 1 |  from Image      (set imagePath)
 //    // 2 |  from Web Camera (set deviceNumber)
 //    // 3 |  from Topic      (set topicName)

 //    CameraWorkingInfo info;
 //    int freq = 20;          // in hz (image updating frequency)
 //    double accurancy = 0.1; // accurancy of circle detection

 //    // Read ROS parameters
 //    std::vector<int> mincolor(3), maxcolor(3);
 //    ros::param::get("/ar_cv/dilation_size", DILATION_SIZE);
 //    ros::param::get("/ar_cv/blur_size", BLUR_SIZE);
 //    ros::param::get("/ar_cv/sigma", accurancy);
 //    ros::param::get("/ar_cv/min_color", mincolor);
 //    ros::param::get("/ar_cv/max_color", maxcolor);
 //    ros::param::get("/ar_cv/topic_name", info.topicName);
 //    ros::param::get("/ar_cv/camera_number", info.deviceNumber);
 //    ros::param::get("/ar_cv/camera_frequency", freq);

 //    cv::Scalar minColor(mincolor[0], mincolor[1], mincolor[2]),
 //               maxColor(maxcolor[0], maxcolor[1], maxcolor[2]);

 //    if (info.topicName == "")
 //        info.workMode = 2;      // Config for web camera
 //    else info.workMode = 3;     // Config for topic

 //    std::vector<ar_cv::CircleInfo> circles;   
 //    SimpleCV cv(nh, info, freq);
 //    cv::Mat image; 
	// while (ros::ok()) {
	// image = cv.getImage().clone();
 //        if (!image.empty()) {
        
 //            // Recognize circle with some accurancy
 //            circles = findCircles(image, minColor, maxColor, accurancy,
 //                DILATION_SIZE, BLUR_SIZE, circleColor, contourColor);

 //            if (circles.size() > 0){
 //                circlePublisher.publish(circles[0]);
 //                ar.drone_move(x,y,z,zz);
 //            } else {
 //            	ar.drone_move(0,0,0,0);
 //            }

 //            #ifdef DEBUG
 //            cv::imshow("Image", image);
 //            #endif
 //        }
 //        cv::waitKey(1);
	// }

	ros::spin();
	return 0;
}