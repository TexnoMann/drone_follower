#include <ardrone_system/ardrone.h>
#include <DroneController.h>

int main(int argc, char **argv){
	int ch ;
	initscr();
	float x,y,z,xx,yy,zz, k = 0.5;
	ros::init(argc, argv, "ardrone_move");
	drone ar;
	ar.takeoff();
	while (1) {
		ch = getch();
		if (ch == 'w'){
			x=k;
			}
		if (ch == 'd'){
			y=-k;
			}
		if (ch == 's'){
			x=-k;
		}
		if (ch == 'a'){
			y=k;
		}
		if (ch == 'e'){
			zz=k;
		}
		if (ch == 'q'){
			zz=-k;
		}
		if ((ch == 'r') && k < 1) {
			k+=0.1;
		}
		if ((ch == 'f') && k > 0) {
			k-=0.1;
		}
		if (ch == 't'){
			z=k;
		}
		if (ch == 'g'){
			z=-k;
		}
		if (ch == 'x'){
			ar.land();
			endwin();
		}
		std::cout<<ch;
		ar.drone_move(x,y,z,zz);
		x = 0;
		y = 0;
		z = 0;
		zz = 0;
	}

	ros::spin();
	return 0;
}