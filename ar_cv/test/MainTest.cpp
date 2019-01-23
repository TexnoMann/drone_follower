#include <ar_cv/SimpleCV.hpp>
#include <ar_cv/CircleInfo.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

int DILATION_SIZE = 0;
int BLUR_SIZE = 0;

cv::Scalar circleColor(255, 255, 255), contourColor(0, 255, 0);
//cv::Scalar minColor(98, 140, 140), maxColor(218, 255, 255);
std::vector<ar_cv::CircleInfo> findCircles(cv::Mat & frame, const cv::Scalar & lowColor, const cv::Scalar & highColor, double sigma);

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "main_test");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::Publisher circlePublisher = nh.advertise<ar_cv::CircleInfo>("/ar_cv/circle_info", 8);

    // Configure camera work
    // Work modes::
    // 1 |  from Image      (set imagePath)
    // 2 |  from Web Camera (set deviceNumber)
    // 3 |  from Topic      (set topicName)

    CameraWorkingInfo info;
    int freq = 10;          // in hz (image updating frequency)
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
            circles = findCircles(image, minColor, maxColor, accurancy);
            if (circles.size() > 0)
                circlePublisher.publish(circles[0]);

            #ifdef DEBUG
            cv::imshow("Image", image);
            #endif
        }
        cv::waitKey(1);
    }

    ros::waitForShutdown();
}

std::vector<ar_cv::CircleInfo> findCircles(cv::Mat & frame, const cv::Scalar & lowColor, const cv::Scalar & highColor, double sigma) {
    cv::Mat hsv, mask, dilat, blur;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    double S = 0, r = 0, err = 0;
    cv::Moments m;
    cv::Point2i center;

    double ang = 0;
    cv::Point2d vec(0, 0);

    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                   cv::Size(2*DILATION_SIZE + 1, 2*DILATION_SIZE + 1),
                   cv::Point(DILATION_SIZE, DILATION_SIZE));

    std::vector<ar_cv::CircleInfo> info;
    ar_cv::CircleInfo cInfo;

    // Filters / thresholds
    cv::bilateralFilter(frame, blur, BLUR_SIZE, BLUR_SIZE*2, BLUR_SIZE/2);
    cv::cvtColor(blur, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, lowColor, highColor, mask);
    cv::dilate(mask, dilat, element);

    cv::findContours(dilat, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    for (size_t i = 0; i < contours.size(); ++i) {
        S = cv::contourArea(contours[i]);
        r = sqrt(S/M_PI);

        if (r > 13 and r < 51) {
            m = cv::moments(contours[i]);
            center = cv::Point2i(m.m10/m.m00, m.m01/m.m00);

            size_t p = 0;
            for (; p < contours[i].size(); ++p) {
                vec = contours[i][p] - center;
                ang = atan2(vec.y, vec.x);

                err += pow(vec.x - r*cos(ang), 2) + pow(vec.y - r*sin(ang), 2);
            }
            err = sqrt(err/p)/r;

            #ifdef DEBUG
            std::cout << "Circle s: " << err << std::endl;
            #endif

            if (err < 0.1) {
                cv::drawContours(frame, contours, i, contourColor, 3, 8, hierarchy);
                cv::circle(frame, center, r, circleColor, 3);

                // Define result message
                cInfo.center.z = -center.y + frame.rows/2;
                cInfo.center.y = -center.x + frame.cols/2;
                cInfo.diameter = 2*r;
                info.push_back(cInfo);
            }
        }
    }
    #ifdef DEBUG
    cv::imshow("mask", dilat);
    cv::imshow("blur", blur);
    #endif
    return info;
}
