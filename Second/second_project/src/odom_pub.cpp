#include "ros/ros.h"
#include <cmath>
#include <string>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#define _USE_MATH_DEFINES

class odom_pub{

public:
	odom_pub(){
		sub = n.subscribe("/speedsteer", 1, &odom_pub::callbackTel, this);//subscribe to telemetry topic
		odom_publisher = n.advertise<nav_msgs::Odometry>("myTopic/odom", 1); //create odometry topic Publisher

		current_time = ros::Time::now(); //take first sys time value for dt calculation
		last_time = current_time; //last time is the same @ the first time
	}

private:
	ros::NodeHandle n; //node handler
	ros::Publisher odom_publisher; //odometry topic publisher
	ros::Subscriber sub;
	ros::Time current_time, last_time;
	double x_ackermann = 0.0;
	double y_ackermann = 0.0;
	double th_ackermann = 0.0;
	tf2::Quaternion quat_tf;


void callbackTel(const geometry_msgs::PointStamped::ConstPtr& msg)
	{
		//compute dt (delta t)
		current_time = ros::Time::now();
		double dt = (current_time - last_time).toSec();
		//retreive velocity and steering angle from msg
		double Vel = msg -> point.x;
		double steer = msg -> point.y;
		double sA = steer / (double)18 * M_PI / (double)180;

		double	w_ack = Vel * tan(sA) / 1.765;

		double delta_th_ack = w_ack * dt;
		th_ackermann += delta_th_ack;

		if(th_ackermann > 2 * M_PI) th_ackermann -= 2 * M_PI;
		if(th_ackermann < 0) th_ackermann += 2 * M_PI;

		double v_ack = w_ack * 1.765 / sin (sA);
		double delta_x_ack = v_ack * cos(th_ackermann) * dt;
		double delta_y_ack = v_ack * sin(th_ackermann) * dt;
		x_ackermann += delta_x_ack;
		y_ackermann += delta_y_ack;
		quat_tf.setRPY( 0, 0, th_ackermann );
		geometry_msgs::Quaternion odom_quat_ack = tf2::toMsg(quat_tf);
		nav_msgs::Odometry odom_ack;
		odom_ack.header.stamp = current_time;
		odom_ack.header.frame_id = "world";

		//set the position
		odom_ack.pose.pose.position.x = x_ackermann;
		odom_ack.pose.pose.position.y = y_ackermann;
		odom_ack.pose.pose.position.z = 0.0;
		odom_ack.pose.pose.orientation = odom_quat_ack;

		//set the velocity
		odom_ack.child_frame_id = "base_link";
		odom_ack.twist.twist.linear.x = v_ack * cos(th_ackermann);
		odom_ack.twist.twist.linear.y = v_ack * sin(th_ackermann);
		odom_ack.twist.twist.angular.z = w_ack;

	}

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "odom_pub");
	odom_pub my_odom_calculator;
	ros::spin();
	return 0;
}
