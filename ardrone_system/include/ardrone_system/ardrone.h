#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <ardrone_autonomy/Navdata.h>


class drone{
private:
	ros::NodeHandle n;
	ros::Publisher move_publisher;
	ros::Publisher takeoff_publisher;
	ros::Publisher land_publisher;
	ros::Publisher reset_publisher;
	ros::Subscriber navdata_sub;
	int state,altd;
	float batteryPercent;
	std::vector<float> rotXYZ;

	geometry_msgs::Twist move;
	std_msgs::Empty empty;
	double start_time_sleeping;

	inline void navdata_callback(const ardrone_autonomy::Navdata::ConstPtr& msg){
  		state = msg -> state;
  		altd = msg -> altd;
  		batteryPercent = msg -> batteryPercent;
  		rotXYZ[0] = msg -> rotX;
  		rotXYZ[1] = msg -> rotY;
  		rotXYZ[2] = msg -> rotZ;
	}

public:

	

	drone(){
		move_publisher = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
		takeoff_publisher = n.advertise<std_msgs::Empty>("ardrone/takeoff",1);
		land_publisher = n.advertise<std_msgs::Empty>("ardrone/land",1);
		reset_publisher = n.advertise<std_msgs::Empty>("ardrone/reset",1);
		navdata_sub = n.subscribe("ardrone/navdata", 1, &drone::navdata_callback, this);
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

	inline void land(){
		land_publisher.publish(empty);
	}

	inline void takeoff(){
		takeoff_publisher.publish(empty);
		ros::Duration(start_time_sleeping).sleep();
	}

	inline void reset(){
		reset_publisher.publish(empty);
	}

	inline void set_start_time_sleeping(double t){
		start_time_sleeping = t;
	}

	float battery(){
		return batteryPercent;
	}

	std::vector<float> rotxyz(){
		return rotXYZ;
	}

	inline bool ok(){
		return ros::ok();
	}
	inline bool inflight(){

		if (state == 3 || state == 7){
			return true;	
		} else {
			return false;
		}
		
	}

};
