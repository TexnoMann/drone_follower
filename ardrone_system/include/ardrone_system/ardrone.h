#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Char.h>
#include <ardrone_autonomy/Navdata.h>
#include <cmath>

class drone{
private:
	ros::NodeHandle n;
	ros::Publisher move_publisher;
	ros::Publisher takeoff_publisher;
	ros::Publisher land_publisher;
	ros::Publisher reset_publisher;
	ros::Subscriber navdata_sub;
	ros::Subscriber talker_sub;
	int state,altd;
	double batteryPercent;
	std::vector<double> rotXYZ;

	geometry_msgs::Twist move;
	std_msgs::Empty empty;
	double start_time_sleeping;

	void navdata_callback(const ardrone_autonomy::Navdata::ConstPtr& msg){
  		state = msg -> state;
  		altd = msg -> altd;
  		batteryPercent = msg -> batteryPercent;
  		rotXYZ[0] = (msg -> rotX)*M_PI/180;
  		rotXYZ[1] = (msg -> rotY)*M_PI/180;
  		rotXYZ[2] = (msg -> rotZ)*M_PI/180;
	}

	void talk_callback(const std_msgs::Char::ConstPtr& msg){
  		switch (msg->data){
  		case 't':
  			takeoff();
  		break;
  		case 'l':
  			land();
  		break;
  		case 'r':
  			reset();
  		break;
  		}
  	}
public:

	

	drone(ros::NodeHandle & nh){
		n = nh;
		batteryPercent = 0;
		rotXYZ.resize(3);
		move_publisher = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
		takeoff_publisher = n.advertise<std_msgs::Empty>("ardrone/takeoff",1);
		land_publisher = n.advertise<std_msgs::Empty>("ardrone/land",1);
		reset_publisher = n.advertise<std_msgs::Empty>("ardrone/reset",1);
		navdata_sub = n.subscribe("ardrone/navdata", 1, &drone::navdata_callback, this);
		talker_sub = n.subscribe("talk", 1, &drone::talk_callback, this);
		start_time_sleeping = 1.5;

	}

	void drone_move(double x, double y, double z, double zz){
		if(ros::ok()){
			move.linear.x = x;
			move.linear.y = y;
			move.linear.z = z;
			move.angular.z= zz;
			move_publisher.publish(move);
		}
	}

	void land(){
		land_publisher.publish(empty);
	}

	void takeoff(){
		takeoff_publisher.publish(empty);
		ros::Duration(start_time_sleeping).sleep();
	}

	void reset(){
		reset_publisher.publish(empty);
	}

	void set_start_time_sleeping(double t){
		start_time_sleeping = t;
	}

	double battery(){
		return batteryPercent;
	}

	std::vector<double> rotxyz(){
		return rotXYZ;
	}
	bool inflight(){

		if (state == 3 || state == 7){
			return true;	
		} else {
			return false;
		}
		
	}

};
