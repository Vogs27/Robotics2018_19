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
#include <first_project/customPoses.h>

#define _USE_MATH_DEFINES

class tf_odom_pub{

public:
	tf_odom_pub(){
		message_filters::Subscriber<first_project::floatStamped> speedL(n, "speedL_stamped", 1); //subscribe to left wheel speed topic
		message_filters::Subscriber<first_project::floatStamped> speedR(n, "speedR_stamped", 1); //subscribe to right wheel speed topic
		message_filters::Subscriber<first_project::floatStamped> steer(n, "steer_stamped", 1); //subscribe to steering topic for ackermann model

		typedef message_filters::sync_policies::ApproximateTime<first_project::floatStamped, first_project::floatStamped, first_project::floatStamped> MySyncPolicy;
		message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), speedL, speedR, steer); //sync speedL_stamped, speedR_stamped and steer_stamped topics

		odom_pub = n.advertise<nav_msgs::Odometry>("world", 50); //create odometry topic Publisher
		custom_poses_pub = n.advertise<first_project::customPoses>("custom_pose", 50); //create custom poses topic publisher

		current_time = ros::Time::now(); //take first sys time value for dt calculation
		last_time = current_time; //last time is the same @ the first time

		sync.registerCallback(boost::bind(&tf_odom_pub::callback, this, _1, _2, _3)); //attach callback for message filter

		XYreconfigure = boost::bind(&tf_odom_pub::callbackSetXY, this, _1, _2); //attach callback for dynamic reconfigure of xy position
		serverXY.setCallback(XYreconfigure);

		ros::spin();
	}

private:
	ros::NodeHandle n; //node handler
	tf::TransformBroadcaster odom_broadcaster; //transformation broadcaster
	geometry_msgs::TransformStamped odom_trans_diff;
	geometry_msgs::TransformStamped odom_trans_diff_lW;
	geometry_msgs::TransformStamped odom_trans_diff_rW;
	geometry_msgs::TransformStamped odom_trans_ack;
	geometry_msgs::TransformStamped odom_trans_ack_lfW;
	geometry_msgs::TransformStamped odom_trans_ack_rfW;
	geometry_msgs::TransformStamped odom_trans_ack_lrW;
	geometry_msgs::TransformStamped odom_trans_ack_rrW;

	ros::Publisher odom_pub; //odometry topic publisher
	ros::Publisher custom_poses_pub;

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
	double r_lW=0; //radiants left weel differential
	double r_rW=0; //radiants right weel differential
	double r_aLW=0; //radiants ackermann rear left weel
	double r_aRW=0; //radiants ackermann rear right weel
	double r_aLFW=0; //radiants ackermann front left weel
	double r_aRFW=0; //radiants ackermann front right weel

	void callbackSetXY(first_project::xyparametersConfig &config, uint32_t level) { //callback for x, y position dynamic reconfigure
		char steeringString[13];
		x_differential = config.newX;
		y_differential = config.newY;
		x_ackermann = config.newX;
		y_ackermann = config.newY;
		steeringMode = config.odom_mode;
		if(steeringMode == 0) strcpy(steeringString,  "differential");
		if(steeringMode == 1) strcpy(steeringString, "ackermann");
		ROS_DEBUG("Reconfigure request: new position (X,Y): %f, %f | %s", config.newX, config.newY, steeringString);
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

		//______________________DIFFERENTIAL DRIVE_______________________________________________
		double v_differential = 0;
		double w_differential = 0;
		//compute odometry with DIFFERENTIAL DRIVE model
		w_differential = (vR - vL)/(double)1.3;
		double delta_th_diff = w_differential * dt;
		th_differential += delta_th_diff;

		if(th_differential > 2 * M_PI) th_differential -= 2 * M_PI;
		if(th_differential < 0) th_differential += 2 * M_PI;

		v_differential = (vR + vL)/(double)2;
		double delta_x_diff = v_differential * cos(th_differential) * dt;
		double delta_y_diff = v_differential * sin(th_differential) * dt;
		x_differential += delta_x_diff;
		y_differential += delta_y_diff;

		//quaternion created from yaw
		geometry_msgs::Quaternion odom_quat_diff = tf::createQuaternionMsgFromYaw(th_differential);

		//transformation published by tf
		odom_trans_diff.header.stamp = current_time;
		odom_trans_diff.header.frame_id = "world";
		odom_trans_diff.child_frame_id = "base_link_differential";

		odom_trans_diff.transform.translation.x = x_differential;
		odom_trans_diff.transform.translation.y = y_differential;
		odom_trans_diff.transform.translation.z = 0.0;
		odom_trans_diff.transform.rotation = odom_quat_diff;

		//prepare odometry message
		nav_msgs::Odometry odom_diff;
		first_project::customPoses diff_pose;
		odom_diff.header.stamp = current_time;
		odom_diff.header.frame_id = "world";

		//set the position
		odom_diff.pose.pose.position.x = x_differential;
		odom_diff.pose.pose.position.y = y_differential;
		odom_diff.pose.pose.position.z = 0.0;
		odom_diff.pose.pose.orientation = odom_quat_diff;

		//set the velocity
		odom_diff.child_frame_id = "base_link_differential";
		odom_diff.twist.twist.linear.x = v_differential * cos(th_differential);
		odom_diff.twist.twist.linear.y = v_differential * sin(th_differential);
		odom_diff.twist.twist.angular.z = w_differential;

		diff_pose.model = "Differential drive";
		diff_pose.x= x_differential;
		diff_pose.y= y_differential;
		diff_pose.th= th_differential;

		//rotation of wheels
		r_lW += vL * dt;
		r_rW += vR * dt;

		if(r_rW > 2 * M_PI) r_rW -= 2 * M_PI;
		if(r_rW < 0) r_rW += 2 * M_PI;

		if(r_lW > 2 * M_PI) r_lW -= 2 * M_PI;
		if(r_lW < 0) r_lW += 2 * M_PI;

		//Right Wheel
		//quaternion created from pitch
		geometry_msgs::Quaternion odom_quat_diff_rW = tf::createQuaternionMsgFromRollPitchYaw(0.0, r_rW, 0.0);

		//transformation published by tf
		odom_trans_diff_rW.header.stamp = current_time;
		odom_trans_diff_rW.header.frame_id = "base_link_differential";
		odom_trans_diff_rW.child_frame_id = "right_wheel_differential";

		odom_trans_diff_rW.transform.translation.x = 0.0;
		odom_trans_diff_rW.transform.translation.y = -0.65;
		odom_trans_diff_rW.transform.translation.z = 0.0;
		odom_trans_diff_rW.transform.rotation = odom_quat_diff_rW;


		//Left Wheel
		//quaternion created from pitch
		geometry_msgs::Quaternion odom_quat_diff_lW = tf::createQuaternionMsgFromRollPitchYaw(0.0, r_lW, 0.0);

		//transformation published by tf
		odom_trans_diff_lW.header.stamp = current_time;
		odom_trans_diff_lW.header.frame_id = "base_link_differential";
		odom_trans_diff_lW.child_frame_id = "left_wheel_differential";

		odom_trans_diff_lW.transform.translation.x = 0.0;
		odom_trans_diff_lW.transform.translation.y = 0.65;
		odom_trans_diff_lW.transform.translation.z = 0.0;
		odom_trans_diff_lW.transform.rotation = odom_quat_diff_lW;




		//___________________ACKERMANN STEERING_____________________________________

		double sA = 0;
		double v_ack = 0;
		double w_ack = 0;
		double sAr = 0; //steering angle right
		double sAl = 0; //steering angle left
		double RICR = 0; //radius instantaneus center of rotation

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

		RICR = 1.765/tan(sA);

		//Rotation of left front wheel
		sAl = atan(1.765/RICR+0.65);
		r_aLFW += w_ack * 1.765 * dt/ sin(sAl);

		if(r_aLFW > 2 * M_PI) r_aLFW -= 2 * M_PI;
		if(r_aLFW < 0) r_aLFW += 2 * M_PI;

		//Rotation of right front wheels
		sAr = atan(1.765/RICR-0.65);
		r_aRFW += v_ack* dt;

		if(r_aRFW > 2 * M_PI) r_aRFW -= 2 * M_PI;
		if(r_aRFW < 0) r_aRFW += 2 * M_PI;

		//Rotation of left rear wheel
		r_aLW += w_ack * 1.765 * dt/ sin(sAr);

		if(r_aLW > 2 * M_PI) r_aLW -= 2 * M_PI;
		if(r_aLW < 0) r_aLW += 2 * M_PI;

		//Rotation of right rear wheel
		r_aRW += vR* dt;

		if(r_aRW > 2 * M_PI) r_aRW -= 2 * M_PI;
		if(r_aRW < 0) r_aRW += 2 * M_PI;

		//quaternion created from yaw
		geometry_msgs::Quaternion odom_quat_ack = tf::createQuaternionMsgFromYaw(th_ackermann);

		//transformation published by tf
		odom_trans_ack.header.stamp = current_time;
		odom_trans_ack.header.frame_id = "world";
		odom_trans_ack.child_frame_id = "base_link_ackermann";

		odom_trans_ack.transform.translation.x = x_ackermann;
		odom_trans_ack.transform.translation.y = y_ackermann;
		odom_trans_ack.transform.translation.z = 0.0;
		odom_trans_ack.transform.rotation = odom_quat_ack;

		//prepare odometry message
		nav_msgs::Odometry odom_ack;
		first_project::customPoses ack_pose;
		odom_ack.header.stamp = current_time;
		odom_ack.header.frame_id = "world";

		//set the position
		odom_ack.pose.pose.position.x = x_ackermann;
		odom_ack.pose.pose.position.y = y_ackermann;
		odom_ack.pose.pose.position.z = 0.0;
		odom_ack.pose.pose.orientation = odom_quat_ack;

		//set the velocity
		odom_ack.child_frame_id = "base_link_ackermann";
		odom_ack.twist.twist.linear.x = v_ack * cos(th_ackermann);
		odom_ack.twist.twist.linear.y = v_ack * sin(th_ackermann);
		odom_ack.twist.twist.angular.z = w_ack;

		ack_pose.model = "Ackermann";
		ack_pose.x = x_ackermann;
		ack_pose.y = y_ackermann;
		ack_pose.th = th_ackermann;

		//Rear Left Wheel
		//quaternion created from pitch
		geometry_msgs::Quaternion odom_quat_ack_rLW = tf::createQuaternionMsgFromRollPitchYaw(0.0, r_aLW, 0.0);

		//transformation published by tf
		odom_trans_ack_lrW.header.stamp = current_time;
		odom_trans_ack_lrW.header.frame_id = "base_link_ackermann";
		odom_trans_ack_lrW.child_frame_id = "left_rear_wheel_ackermann";

		odom_trans_ack_lrW.transform.translation.x = 0.0;
		odom_trans_ack_lrW.transform.translation.y = 0.65;
		odom_trans_ack_lrW.transform.translation.z = 0.0;
		odom_trans_ack_lrW.transform.rotation = odom_quat_ack_rLW;

		//Rear Right Wheel
		//quaternion created from pitch
		geometry_msgs::Quaternion odom_quat_ack_rRW = tf::createQuaternionMsgFromRollPitchYaw(0.0, r_aRW, 0.0);
		//transformation published by tf
		odom_trans_ack_rrW.header.stamp = current_time;
		odom_trans_ack_rrW.header.frame_id = "base_link_ackermann";
		odom_trans_ack_rrW.child_frame_id = "right_rear_wheel_ackermann";

		odom_trans_ack_rrW.transform.translation.x = 0.0;
		odom_trans_ack_rrW.transform.translation.y = -0.65;
		odom_trans_ack_rrW.transform.translation.z = 0.0;
		odom_trans_ack_rrW.transform.rotation = odom_quat_ack_rRW;


		//Front wheels

		//Front Left Wheel
		//quaternion created from pitch and yaw
		geometry_msgs::Quaternion odom_quat_ack_flW = tf::createQuaternionMsgFromRollPitchYaw(0.0, r_aLFW, sAl);

		//transformation published by tf
		odom_trans_ack_lfW.header.stamp = current_time;
		odom_trans_ack_lfW.header.frame_id = "base_link_ackermann";
		odom_trans_ack_lfW.child_frame_id = "left_front_wheel_ackermann";

		odom_trans_ack_lfW.transform.translation.x = 1.765;
		odom_trans_ack_lfW.transform.translation.y = 0.65;
		odom_trans_ack_lfW.transform.translation.z = 0.0;
		odom_trans_ack_lfW.transform.rotation = odom_quat_ack_flW;


		//Rear Right Wheel
		//quaternion created from pitch and yaw
		geometry_msgs::Quaternion odom_quat_ack_frW = tf::createQuaternionMsgFromRollPitchYaw(0.0, r_aRFW, sAr);
		//transformation published by tf
		odom_trans_ack_rfW.header.stamp = current_time;
		odom_trans_ack_rfW.header.frame_id = "base_link_ackermann";
		odom_trans_ack_rfW.child_frame_id = "right_front_wheel_ackermann";

		odom_trans_ack_rfW.transform.translation.x = 1.765;
		odom_trans_ack_rfW.transform.translation.y = -0.65;
		odom_trans_ack_rfW.transform.translation.z = 0.0;
		odom_trans_ack_rfW.transform.rotation = odom_quat_ack_frW;


		//___________________________________________________________________________

		last_time = current_time;

		if(steeringMode == 0){
			//publish odometry with differential drive
			odom_pub.publish(odom_diff);
			//send the transform with tf
			odom_broadcaster.sendTransform(odom_trans_diff);
			odom_broadcaster.sendTransform(odom_trans_diff_rW);
			odom_broadcaster.sendTransform(odom_trans_diff_lW);
			custom_poses_pub.publish(diff_pose);
		}
		else{
			//publish odometry with ACKERMANN model
			odom_pub.publish(odom_ack);
			//send the transform with tf
			odom_broadcaster.sendTransform(odom_trans_ack);
			odom_broadcaster.sendTransform(odom_trans_ack_lrW);
			odom_broadcaster.sendTransform(odom_trans_ack_rrW);
			odom_broadcaster.sendTransform(odom_trans_ack_lfW);
			odom_broadcaster.sendTransform(odom_trans_ack_rfW);
			custom_poses_pub.publish(ack_pose);
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
