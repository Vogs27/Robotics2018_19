#include "ros/ros.h"
//with this we can implement a tf broadcaster that make the task of publishing transforms easier.
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <first_project/floatStamped.h>
#include <nav_msgs/Odometry.h>

using namespace message_filters;

class car_sub_pub
{
	public:
	car_sub_pub(){
		//subsribe to multiple topics with message_filters
		message_filters::Subscriber<first_project::floatStamped> speedL_sub(n, "speed_L_stamped", 1);
		message_filters::Subscriber<first_project::floatStamped> speedR_sub(n, "speed_R_stamped", 1);
		message_filters::Subscriber<first_project::floatStamped> steer_sub(n, "steer_stamped", 1);
 		TimeSynchronizer<first_project::floatStamped, first_project::floatStamped> sync(speedL_sub, speedR_sub, 10);
	}


	void callback(const first_project::floatStamped::ConstPtr& , const first_project::floatStamped::ConstPtr& ){
		//create the tfBroadcaster -> this will send the transformations
		tf::TransformBroadcaster br;
		//create the transform object
		tf::Transform transform;
		transform.setOrigin( tf::Vector3(0, 0, 0) );
		tf::Quaternion q;
		q.setRPY(0, 0, 0);
		transform.setRotation(q);

		//send the transformation (transform, time, "parentFrame", "childFrame")
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "turtle"));
	}

	private:
	//create the NodeHandle
	ros::NodeHandle n;
	};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "odometry_tf_publisher");
	car_sub_pub my_car_sub_pub;
	sync.registerCallback(boost::bind(callback, _1, _2));

	ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

	double vk;

  double vx = 0; //TODO
  double vy = 0; //TODO
  double vth = 0; //TODO

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(1.0);
  while(n.ok()){
    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    //Compute odometry given wheels' speed
		double vk = (speedL + speedR)/2
		double dt = (current_time - last_time).toSec();
    double delta_x = vk * cos(th) * dt;
    double delta_y = vk * sin(th) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
	return 0;
}
