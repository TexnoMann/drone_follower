#include <ardrone_system/ardrone.h>
#include <ar_cv/SimpleCV.hpp>
#include <ar_cv/algorithms.hpp>
#include <ardrone_control/DroneController.h>


int DILATION_SIZE = 0;
int BLUR_SIZE = 0;

cv::Scalar circleColor(255, 255, 255), contourColor(0, 255, 0);
//cv::Scalar minColor(98, 140, 140), maxColor(218, 255, 255);
DroneController ctrl(92,51.75, 1, 2, 2, 1280.0, 720.0);

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "test_drone_cv");
    ros::NodeHandle nh;
    drone ar(nh);
    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::Publisher circlePublisher = nh.advertise<ar_cv::CircleInfo>("/ar_cv/circle_info", 8);
    //ros::Publisher circlePublisher = nh.advertise<ar_cv::CircleInfo>("/ar_cv/circle_info", 8);

    // Configure camera work
    // Work modes::
    // 1 |  from Image      (set imagePath)
    // 2 |  from Web Camera (set deviceNumber)
    // 3 |  from Topic      (set topicName)

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
    while(ros::ok()) {
        image = cv.getImage().clone();
        if (!image.empty()) {

            // Recognize circle with some accurancy
            circles = findCircles(image, minColor, maxColor, accurancy,
                DILATION_SIZE, BLUR_SIZE, circleColor, contourColor);

            if (circles.size() > 0){
                circlePublisher.publish(circles[0]);

                std::vector<double> v=getCircleInfoForControl(circles[0]);
                std::cout <<"------------------------\n";
                std::cout <<"\nTest Lesha's function\n";
                std::cout<<"x: " << v[0]<<" y: "<<v[1]<<" z: "<<v[2]<<" rad: "<<v[3]<<"\n";
                std::vector<double> desVector={1,0,0};
                std::cout <<"\nTest Vlad's function\n";
                std::vector<float> s = ctrl.getVectorControl(v,ar.rotxyz()[0],ar.rotxyz()[1],desVector);
                std::cout<<"x: " << s[0]<<" y: "<<s[1]<<" z: "<<s[2]<<" zz: "<<s[3]<<"\n";
                std::cout <<"------------------------\n";
                 // ar.drone_move(s[0],s[1],s[2],0);
            }

            #ifdef DEBUG
            cv::imshow("Image", image);
            #endif
        }
        cv::waitKey(1);
    }

    ros::waitForShutdown();
}
