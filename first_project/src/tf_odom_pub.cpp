#include "ros/ros.h"
#include <cmath>
#include <string>
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

			  sync.registerCallback(boost::bind(&tf_odom_pub::callback, this, _1, _2, _3)); //attach callback for message filter

			  /*steeringReconfigure = boost::bind(&tf_odom_pub::callbackSteering, this, _1, _2); //attach callback for dynamic reconfigure of steering model
			  serverSteering.setCallback(steeringReconfigure);
*/
			  XYreconfigure = boost::bind(&tf_odom_pub::callbackSetXY, this, _1, _2); //attach callback for dynamic reconfigure of xy position
			  serverXY.setCallback(XYreconfigure);

			ros::spin();
		}

	private:
		ros::NodeHandle n; //node handler
		tf::TransformBroadcaster odom_broadcaster; //transformation broadcaster
		geometry_msgs::TransformStamped odom_trans_diff;
		geometry_msgs::TransformStamped odom_trans_ack;

		ros::Publisher odom_pub; //odometry topic publisher

	/*	dynamic_reconfigure::Server<first_project::steeringParametersConfig> serverSteering; //create a parameter server for steeringMode
		dynamic_reconfigure::Server<first_project::steeringParametersConfig>::CallbackType steeringReconfigure;*/

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


/*	void callbackSteering(first_project::steeringParametersConfig &config, uint32_t level) { //callback for steering model reconfigure
		steeringMode = config.odom_mode;
		ROS_INFO("Reconfigure request: steering param: %d", config.odom_mode);
	}*/

	void callbackSetXY(first_project::xyparametersConfig &config, uint32_t level) { //callback for x, y position dynamic reconfigure
			char steeringString[13];
			x_differential = config.newX;
			y_differential = config.newY;
			x_ackermann = config.newX;
			y_ackermann = config.newY;
			steeringMode = config.odom_mode;
			if(steeringMode == 0) strcpy(steeringString,  "differential");
			if(steeringMode == 1) strcpy(steeringString, "ackermann");
			ROS_INFO("Reconfigure request: new position (X,Y): %f, %f | %s", config.newX, config.newY, steeringString);
	}

	void callback(const first_project::floatStampedConstPtr& speedL, const first_project::floatStampedConstPtr& speedR, const first_project::floatStampedConstPtr& steer)  //callback for message filter
		{

		    //compute dt (delta t)
		    current_time = ros::Time::now();
		    double dt = (current_time - last_time).toSec();

		    //get velocities from bag's topics
		    float vL = speedL -> data;
		    float vR = speedR -> data;
			float s = steer -> data;

//______________________DIFFERENTIAL DRIVE_______________________________________________---
		    double v_diff = 0;
		    double w_diff = 0;
			//compute odometry with DIFFERENTIAL DRIVE model
			w_diff = (vR - vL)/(double)1.3;
			double delta_th_diff = w_diff * dt;
			th_differential += delta_th_diff;

			if(th_differential > 2 * M_PI) th_differential -= 2 * M_PI;
			if(th_differential < 0) th_differential += 2 * M_PI;

			v_diff = (vR + vL)/(double)2;
			double delta_x_diff = v_diff * cos(th_differential) * dt;
			double delta_y_diff = v_diff * sin(th_differential) * dt;
			x_differential += delta_x_diff;
			y_differential += delta_y_diff;

		    //quaternion created from yaw
		    geometry_msgs::Quaternion odom_quat_diff = tf::createQuaternionMsgFromYaw(th_differential);

		    //transformation published by tf
		    odom_trans_diff.header.stamp = current_time;
		    odom_trans_diff.header.frame_id = "world";
		    odom_trans_diff.child_frame_id = "base_link";

		    odom_trans_diff.transform.translation.x = x_differential;
		    odom_trans_diff.transform.translation.y = y_differential;
		    odom_trans_diff.transform.translation.z = 0.0;
		    odom_trans_diff.transform.rotation = odom_quat_diff;


		 	//prepare odometry message
		    nav_msgs::Odometry odom_diff;
		    odom_diff.header.stamp = current_time;
		    odom_diff.header.frame_id = "world";

		    //set the position
		    odom_diff.pose.pose.position.x = x_differential;
		    odom_diff.pose.pose.position.y = y_differential;
		    odom_diff.pose.pose.position.z = 0.0;
		    odom_diff.pose.pose.orientation = odom_quat_diff;

		    //set the velocity
		    odom_diff.child_frame_id = "base_link";
		    odom_diff.twist.twist.linear.x = v_diff * cos(th_differential);
		    odom_diff.twist.twist.linear.y = v_diff * sin(th_differential);
		    odom_diff.twist.twist.angular.z = w_diff;

//___________________ACKERMANN STEERING_____________________________________

			double sA = 0;
			double v_ack = 0;
			double w_ack = 0;

			//compute odometry with ACKERMANN DRIVE model
			sA = s / (double)18 * M_PI / (double)180;
			w_ack = (vR + vL) / 2 * tan(sA) / 1.765;

			double delta_th_ack = w_ack * dt;
			th_ackermann += delta_th_ack;

			if(th_ackermann > 2 * M_PI) th_ackermann -= 2 * M_PI;
			if(th_ackermann < 0) th_ackermann += 2 * M_PI;

			v_ack = w_ack * 1.765 / sin (sA);
			double delta_x_ack = v_ack * cos(th_ackermann) * dt;
			double delta_y_ack = v_ack * sin(th_ackermann) * dt;
			x_ackermann += delta_x_ack;
			y_ackermann += delta_y_ack;

			//quaternion created from yaw
		 geometry_msgs::Quaternion odom_quat_ack = tf::createQuaternionMsgFromYaw(th_ackermann);

		 //transformation published by tf
		 odom_trans_ack.header.stamp = current_time;
		 odom_trans_ack.header.frame_id = "world";
		 odom_trans_ack.child_frame_id = "base_link";

		 odom_trans_ack.transform.translation.x = x_ackermann;
		 odom_trans_ack.transform.translation.y = y_ackermann;
		 odom_trans_ack.transform.translation.z = 0.0;
		 odom_trans_ack.transform.rotation = odom_quat_ack;


		 //prepare odometry message
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

//---------------------------------------------------------------------------

		    last_time = current_time;

			if(steeringMode == 0){
				//publish odometry with differential drive
			    odom_pub.publish(odom_diff);
				//send the transform with tf
			    odom_broadcaster.sendTransform(odom_trans_diff);
		  	}
		  	else{
		  		//publish odometry with ACKERMANN model
				odom_pub.publish(odom_ack);
				//send the transform with tf
			    odom_broadcaster.sendTransform(odom_trans_ack);

		  	}
		}

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_odom_pub");
  tf_odom_pub my_tf_odom_calculator;
  ros::spin();

  return 0;
}
