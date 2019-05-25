
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <custom_messages/floatStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

double x = 0.0;
double y = 0.0;
double th = 0.0;

ros::Time current_time, last_time;

void callback(const custom_messages::floatStampedConstPtr& speedL, const custom_messages::floatStampedConstPtr& speedR)
{

    current_time = ros::Time::now();
    float vL = speedL -> data;
    float vR = speedR -> data;
    //compute odometry in a typical way given the velocities of the robot
  
    double v = (vL + vR)/2;
    double dt = (double) 1/10;

    double delta_th = (vR - vL)/(double)0.13 * dt;
    th += delta_th;
    
    double delta_x = v * cos(th) * dt;
    double delta_y = v * sin(th) * dt;
    x += delta_x;
    y += delta_y;
    

  ROS_INFO ("L: (%f), R: (%f) | X,Y,TH (%f, %f, %f)\n", vL, vR, x, y, th);
}

float toFloat(const custom_messages::floatStampedConstPtr& stamped){
	return stamped->data;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "subscriber_sync");

  ros::NodeHandle n;

  message_filters::Subscriber<custom_messages::floatStamped> speedL(n, "speedL_stamped", 1);
  message_filters::Subscriber<custom_messages::floatStamped> speedR(n, "speedR_stamped", 1);
  
  //typedef message_filters::sync_policies::ExactTime<geometry_msgs::Vector3Stamped, geometry_msgs::Vector3Stamped> MySyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<custom_messages::floatStamped, custom_messages::floatStamped> MySyncPolicy;
  
  
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), speedL, speedR);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  //create odom publisher
  /*ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

    
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate r(1.0);
    while(n.ok()){

    ros::spinOnce();               
    // check for incoming messages


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
*/
  return 0;
}


