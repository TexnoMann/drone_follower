#include <ardrone_system/ardrone.h>
#include <ncurses.h>
#include <ctime> 

// #include <DroneController.h>
// #include <SimpleCV.hpp>
// #include <algorithms.hpp>

// #include <opencv2/imgproc.hpp>
// #include <opencv2/highgui.hpp>

int DILATION_SIZE = 0;
int BLUR_SIZE = 0;

// cv::Scalar circleColor(255, 255, 255), contourColor(0, 255, 0);

int main(int argc, char **argv){
	double ch = 0 ;
	initscr();
	float x,y,z,zz,k = 0.5;
	ros::init(argc, argv, "ardrone_move");
	drone ar;
	noecho();
	halfdelay(0.1); 
	// DroneController dronectrl(0.802, 0.1, 0.1, 0.1, 1280.0, 720.0);
	// Eigen::Vector3d zyrPos;
	// Eigen::Vector3d desVector={1, 0, -1};
	// Eigen::Vector3f ctrl;
	// std::vector<ar_cv::CircleInfo> circles;
	// cv::Mat image;
	// std::vector<int> mincolor(3)=(18,102,123), maxcolor(3)=(36,223,255);
	// cv::Scalar minColor(mincolor[0], mincolor[1], mincolor[2]),
 //               maxColor(maxcolor[0], maxcolor[1], maxcolor[2]);
	
	while (ar.ok()) {
		// image = cv.getImage().clone();
  //       if (!image.empty()) {
  //       	circles = findCircles(image, minColor, maxColor, accurancy, DILATION_SIZE, BLUR_SIZE, circleColor, contourColor);

  //           if (circles.size() > 0)
  //               zyrPos={circles[0][2],circles[0][1],circles[1]}

  //           #ifdef DEBUG
  //           cv::imshow("Image", image);
  //           #endif
  //       }
  //       cv::waitKey(1);
		switch (getch()){
			case 't':
				ar.takeoff();	
			break;
			case 'l':
				ar.land();
				endwin();
				return 0;
			break;
		}
		if (ar.inflight()){
			printw("I'm fly!");
		} else {
			printw("I'm on the graund!");
		}
		// ctrl=(Eigen::Vector3f)dronectrl.getVectorControl(1, zyrPos, 0, 0, desVector);
		// ar.drone_move(ctrl[0],ctrl[1],ctrl[2],0.00);
	
	}

	ros::spin();
	return 0;
}
