#include <ardrone_system/ardrone.h>
#include <ardrone_autonomy/Navdata.h>
#include <ncurses.h>
#include <std_msgs/Char.h>

int state;
float batteryPercent;

void navdata_callback(const ardrone_autonomy::Navdata::ConstPtr& msg){
  		state = msg -> state;
  		batteryPercent = msg -> batteryPercent;
	}


	void printText(){
		printw("Parrot AR.Drone 2.0\n");
		printw("\nInformation:\n\n");
		switch(state){
			case 1:
				printw("State: inited\n");
			break;
			case 2:
				printw("State: landed\n");
			break;
			case 3:
			case 7:
				printw("State: flying\n");
			break;
			case 4:
				printw("State: hovering\n");
			break;
			case 6:
				printw("State: taking off\n");
			break;
			case 8:
				printw("State: landing\n");
			break;
			default:
				printw("State: unknown\n");
			break;
		}
		printw("Battery(%): %f",batteryPercent);
		printw("\n\nControl:\n\n");
		printw("takeoff - \'t\'\n");
		printw("reset - \'r\'\n");
		printw("land - \'l\'\n");
		printw("close programm - \'x\'\n\n");
		printw("Good luck!");

		 
	}
int main(int argc, char **argv){
	char ch = 0;
	ros::init(argc, argv, "convenience");
	ros::NodeHandle n;
	ros::Rate mainRate(100);
	ros::Subscriber navdata_sub = n.subscribe("ardrone/navdata", 1, navdata_callback);
	ros::Publisher talk = n.advertise<std_msgs::Char>("talk",1);
	initscr();
	noecho();
	drone ar(n);
	halfdelay(1);
	std_msgs::Char msg;
	
	while (ch != 'x') {
		printText();
		ch = getch();
		if (ch == 't' || ch == 'l' || ch == 'r'){
			msg.data = ch;
			talk.publish(msg);
		}
		clear();
		if (ch == 'x' && (state == 3 || state == 7)){
		msg.data = 'l';
		talk.publish(msg);}
		ros::spinOnce();
	}
	endwin();
	return 0;
}
