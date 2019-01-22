#include "ros/ros.h"
#include <ncurses.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"


class drone{
private:
    ros::NodeHandle n;
    ros::Publisher move_publisher;
    ros::Publisher takeoff_publisher;
    ros::Publisher land_publisher;
    ros::Publisher reset_publisher;
    geometry_msgs::Twist move; 
    std_msgs::Empty empty;
    double start_time_sleeping = 1.5;
public:
    drone(){
        move_publisher = n.advertise<geometry_msgs::Twist>("cmd_vel",1);    
    	takeoff_publisher = n.advertise<std_msgs::Empty>("ardrone/takeoff",1);
    	land_publisher = n.advertise<std_msgs::Empty>("ardrone/land",1);
    	reset_publisher = n.advertise<std_msgs::Empty>("ardrone/reset",1);
    }

    void drone_move(double x, double y, double z, double zz){ 
	    if(n.ok()){
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

};