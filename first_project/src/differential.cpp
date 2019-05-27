#include "ros/ros.h"
#include <cmath>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <first_project/floatStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <dynamic_reconfigure/server.h>
#include <first_project/xyparametersConfig.h>

#define _USE_MATH_DEFINES

double x = 0.0;
double y = 0.0;
double th = 0.0;

ros::Time current_time, last_time;

class differential_drive{

	public:
		differential_drive(){

			  message_filters::Subscriber<first_project::floatStamped> speedL(n, "speedL_stamped", 1);
			  message_filters::Subscriber<first_project::floatStamped> speedR(n, "speedR_stamped", 1);

			  typedef message_filters::sync_policies::ApproximateTime<first_project::floatStamped, first_project::floatStamped> MySyncPolicy;
			  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), speedL, speedR);

			  current_time = ros::Time::now();
			  last_time = current_time;
			  odom_pub = n.advertise<nav_msgs::Odometry>("world_differential", 50);

			  sync.registerCallback(boost::bind(&differential_drive::callback, this, _1, _2));
			ros::spin();
		}

	private:
		ros::NodeHandle n;
		tf::TransformBroadcaster odom_broadcaster; //transformation broadcaster
		geometry_msgs::TransformStamped odom_trans;
		ros::Publisher odom_pub; //odometry topic publisher
		dynamic_reconfigure::Server<first_project::xyparametersConfig> server;
		dynamic_reconfigure::Server<first_project::xyparametersConfig>::CallbackType f;

	void callbackParam(first_project::xyparametersConfig &config, uint32_t level) { //callback for dynamic reconfigure
		x = config.newX;
		y = config.newY;
		ROS_INFO("Reconfigure request: new xy position: %d, %d", config.newX, config.newY);
	}

	void callback(const first_project::floatStampedConstPtr& speedL, const first_project::floatStampedConstPtr& speedR) //callback for message message_filters
		{   //here we compute and publish odometry based on differential drive model
		    //compute dt (delta t)
		    current_time = ros::Time::now();
		    double dt = (current_time - last_time).toSec();

		    //get velocities from bag's topics
		    float vL = speedL -> data;
		    float vR = speedR -> data;
		    double v = 0;
		    double w = 0;

		 	//compute odometry with DIFFERENTIAL DRIVE model
			w = (vR - vL)/(double)1.3;
			double delta_th = w * dt;
			th += delta_th;
			if(th > 2 * M_PI) th -= 2 * M_PI;
			if(th < 0) th += 2 * M_PI;

			v = (vR + vL)/(double)2;
			double delta_x = v * cos(th) * dt;
			double delta_y = v * sin(th) * dt;
			x += delta_x;
			y += delta_y;

		 // ROS_INFO ("TIMEBAG: [%i,%i], dt: [%f], L: (%f), R: (%f) | X,Y,TH (%f, %f, %f)\n", speedL->header.stamp.sec, speedL->header.stamp.nsec, dt, vL, vR, x, y, th);

		    //quaternion created from yaw
		    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

		    //transformation published by tf

		    odom_trans.header.stamp = current_time;
		    odom_trans.header.frame_id = "world";
		    odom_trans.child_frame_id = "base_link_differential";

		    odom_trans.transform.translation.x = x;
		    odom_trans.transform.translation.y = y;
		    odom_trans.transform.translation.z = 0.0;
		    odom_trans.transform.rotation = odom_quat;

		    //send the transform with tf
		    odom_broadcaster.sendTransform(odom_trans);

		 	//publish odometry topic
		    nav_msgs::Odometry odom;
		    odom.header.stamp = current_time;
		    odom.header.frame_id = "world";

		    //set the position
		    odom.pose.pose.position.x = x;
		    odom.pose.pose.position.y = y;
		    odom.pose.pose.position.z = 0.0;
		    odom.pose.pose.orientation = odom_quat;

		    //set the velocity
		    odom.child_frame_id = "base_link";
		    odom.twist.twist.linear.x = v * cos(th);
		    odom.twist.twist.linear.y = v * sin(th);
		    odom.twist.twist.angular.z = w;

		    //publish the message
		    odom_pub.publish(odom);

		    last_time = current_time;

		}

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "differential");
  differential_drive my_differential_drive;
  ros::spin();

  return 0;
}
