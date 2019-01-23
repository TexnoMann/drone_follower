#include <ar_cv/SimpleCV.hpp>

SimpleCV::SimpleCV(ros::NodeHandle & nh, CameraWorkingInfo info, int frequency)
	: n(nh)
{
    workMode = info.workMode;
    camWidth = 0; camHeight = 0;
    packagePath = ros::package::getPath("ar_cv");

    switch(info.workMode) {
        case 1: // From image
            image = cv::imread(info.imagePath, CV_LOAD_IMAGE_COLOR);
            // TODO do image processing
            cv::imshow("image", image);
            cv::waitKey(0);
            break;
        case 2: // Onboard web camera
            devNum = info.deviceNumber;
            std::cout << "[Simple CV] Start Web camera " << devNum << std::endl;
            thr = boost::thread(&SimpleCV::updateImageByOpenCV, this);
            break;
        case 3: // Topic
            std::cout << "[Simple CV] Start topic Subscriber: " << info.topicName << std::endl;
            cameraSubscriber = nh.subscribe(info.topicName, 10, &SimpleCV::imageCallback, this);
            ros::Duration(1).sleep();
            break;
        default:
            std::cout << "[Simple CV] Work Mode is not correct!" << std::endl;
    }
}
SimpleCV::~SimpleCV()
{}

void SimpleCV::updateImageByOpenCV()
{
    cv::VideoCapture videoCapture;
    videoCapture.open(devNum);
    if (!videoCapture.isOpened()) {
        std::cout << "Could not open reference " << devNum << std::endl;
        return;
    }

    videoCapture >> image;
    // if (camWidth == 0 || camHeight == 0) {
    //     camWidth = image.cols; camHeight = image.rows;
    // }

    ros::Rate rate(freq);
    while(ros::ok()) {
        videoCapture >> image;
        rate.sleep();
    }
}

void SimpleCV::imageCallback(const sensor_msgs::Image::ConstPtr & msg)
{

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        ros::shutdown();
    }

    image = cv_ptr->image;
    // if (camWidth == 0 || camHeight == 0) {
    //     camWidth = image.cols; camHeight = image.rows;
    // }
    // ros::Duration(round(1/freq)).sleep();
}