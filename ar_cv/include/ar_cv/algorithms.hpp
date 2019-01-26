#pragma once

#include <opencv2/opencv.hpp>

// ROS
#include <ar_cv/CircleInfo.h>

std::vector<ar_cv::CircleInfo> findCircles(cv::Mat & frame, const cv::Scalar & lowColor, const cv::Scalar & highColor,
    double sigma, int dilationSize, int blurSize, cv::Scalar circleColor, cv::Scalar contourColor)
{
    cv::Mat hsv, mask, dilat, blur;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    double S = 0, r = 0, err = 0;
    cv::Moments m;
    cv::Point2i center;

    double ang = 0;
    cv::Point2d vec(0, 0);

    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                   cv::Size(2*dilationSize + 1, 2*dilationSize + 1),
                   cv::Point(dilationSize, dilationSize));

    std::vector<ar_cv::CircleInfo> info;
    ar_cv::CircleInfo cInfo;

    // Filters / thresholds
    cv::bilateralFilter(frame, blur, blurSize, blurSize*2, blurSize/2);
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

            if (err < sigma) {
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

std::vector<double> getCircleInfoForControl(ar_cv::CircleInfo info){
    std::vector<double> vectorForControl(4);
    vectorForControl[0]=(double)info.center.x;
    vectorForControl[1]=(double)info.center.y;
    vectorForControl[2]=(double)info.center.z;
    vectorForControl[3]=(double)info.diameter;
    return vectorForControl;
}
