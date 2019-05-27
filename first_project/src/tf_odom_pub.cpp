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
#include <first_project/steeringParametersConfig.h>

#define _USE_MATH_DEFINES

class tf_odom_pub{

	public:
		tf_odom_pub(){
			  message_filters::Subscriber<first_project::floatStamped> speedL(n, "speedL_stamped", 1); //subscribe to left wheel speed topic
			  message_filters::Subscriber<first_project::floatStamped> speedR(n, "speedR_stamped", 1); //subscribe to right wheel speed topic
			  message_filters::Subscriber<first_project::floatStamped> steer(n, "steer_stamped", 1); //subscribe to steering topic for ackermann model

			  typedef message_filters::sync_policies::ApproximateTime<first_project::floatStamped, first_project::floatStamped, first_project::floatStamped> MySyncPolicy;
			  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), speedL, speedR, steer); //sync speedL_stamped, speedR_stamped and steer_stamped topics

			  current_time = ros::Time::now(); //take first sys time value for dt calculation
			  last_time = current_time; //last time is the same @ the first time

			  odom_pub = n.advertise<nav_msgs::Odometry>("world", 50); //create odometry topic Publisher

			  sync.registerCallback(boost::bind(&tf_odom_pub::callback, this, _1, _2)); //attach callback for message filter

			  steeringReconfigure = boost::bind(&tf_odom_pub::callbackSteering, this, _1, _2); //attach callback for dynamic reconfigure of steering model
			  serverSteering.setCallback(steeringReconfigure);

			  XYreconfigure = boost::bind(&tf_odom_pub::callbackSetXY, this, _1, _2); //attach callback for dynamic reconfigure of xy position
			  serverXY.setCallback(XYreconfigure);

			ros::spin();
		}

	private:
		ros::NodeHandle n; //node handler
		tf::TransformBroadcaster odom_broadcaster; //transformation broadcaster
		geometry_msgs::TransformStamped odom_trans;

		ros::Publisher odom_pub; //odometry topic publisher

		dynamic_reconfigure::Server<first_project::steeringParametersConfig> serverSteering; //create a parameter server for steeringMode
		dynamic_reconfigure::Server<first_project::steeringParametersConfig>::CallbackType steeringReconfigure;

		dynamic_reconfigure::Server<first_project::xyparametersConfig> serverXY; //create a parameter server for x, y reconfigure
  		dynamic_reconfigure::Server<first_project::xyparametersConfig>::CallbackType XYreconfigure;

		ros::Time current_time, last_time;
		int steeringMode;
		double x_differential = 0.0;
		double y_differential = 0.0;
		double th_differential = 0.0;
		double x_ackermann = 0.0;
		double y_ackermann = 0.0;
		double th_ackermann = 0.0;


	void callbackSteering(first_project::steeringParametersConfig &config, uint32_t level) { //callback for steering model reconfigure
		steeringMode = config.odom_mode;
		ROS_INFO("Reconfigure request: steering param: %d", config.odom_mode);
	}

	void callbackSetXY(first_project::xyparametersConfig &config, uint32_t level) { //callback for x, y position dynamic reconfigure
			x_differential = config.newX;
			y_differential = config.newY;
			x_ackermann = config.newX;
			y_ackermann = config.newY;
			ROS_INFO("Reconfigure request: new xy position: %f, %f", config.newX, config.newY);
	}

	void callback(const first_project::floatStampedConstPtr& speedL, const first_project::floatStampedConstPtr& speedR)  //callback for message filter
		{
		    //compute dt (delta t)
		    current_time = ros::Time::now();
		    double dt = (current_time - last_time).toSec();

		    //get velocities from bag's topics
		    float vL = speedL -> data;
		    float vR = speedR -> data;
		    double v = 0;
		    double w = 0;

		    if(steeringMode == 0){
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
		    }
		    else{
			//compute odometry with ACKERMANN model
		    }

		 // ROS_INFO ("TIMEBAG: [%i,%i], dt: [%f], L: (%f), R: (%f) | X,Y,TH (%f, %f, %f)\n", speedL->header.stamp.sec, speedL->header.stamp.nsec, dt, vL, vR, x, y, th);

		    //quaternion created from yaw
		    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

		    //transformation published by tf

		    odom_trans.header.stamp = current_time;
		    odom_trans.header.frame_id = "world";
		    odom_trans.child_frame_id = "base_link";

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
  ros::init(argc, argv, "tf_odom_pub");
  tf_odom_pub my_tf_odom_calculator;
  ros::spin();

  return 0;
}
