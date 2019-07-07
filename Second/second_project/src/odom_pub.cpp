#include "ros/ros.h"
#include <cmath>
#include <string>
#include <nav_msgs/Odometry.h>

#define _USE_MATH_DEFINES
/*
class odom_pub{

public:
	odom_pub(){
		ros::Subscriber sub = n.subscribe("speedster", 1000, callback);//subscribe to telemetry topic

		odom_pub = n.advertise<nav_msgs::Odometry>("world", 50); //create odometry topic Publisher

		current_time = ros::Time::now(); //take first sys time value for dt calculation
		last_time = current_time; //last time is the same @ the first time
		ros::spin();
	}

private:
	ros::NodeHandle n; //node handler
	ros::Publisher odom_pub; //odometry topic publisher
	ros::Time current_time, last_time;


	void callback(const first_project::floatStampedConstPtr& speedL, const first_project::floatStampedConstPtr& speedR, const first_project::floatStampedConstPtr& steer)  //callback for message filter
	{
		//compute dt (delta t)
		current_time = ros::Time::now();
		double dt = (current_time - last_time).toSec();

		//get velocities from bag's topics
		float vL = speedL -> data;
		float vR = speedR -> data;
		float s = steer -> data;

};
*/
int main(int argc, char** argv)
{
	ros::init(argc, argv, "odom_pub");
	//tf_odom_pub my_odom_calculator;
	ros::spin();

	return 0;
}
