
#include "ros/ros.h"
#include <custom_messages/floatStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>


void callback(const custom_messages::floatStampedConstPtr& msg1, const custom_messages::floatStampedConstPtr& msg2)
{
  ROS_INFO ("Received two messages: (%f) and (%f)", msg1->data, msg2->data);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "subscriber_sync");

  ros::NodeHandle n;

  message_filters::Subscriber<custom_messages::floatStamped> sub1(n, "speedL_stamped", 1);
  message_filters::Subscriber<custom_messages::floatStamped> sub2(n, "speedR_stamped", 1);
  
  //typedef message_filters::sync_policies::ExactTime<geometry_msgs::Vector3Stamped, geometry_msgs::Vector3Stamped> MySyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<custom_messages::floatStamped, custom_messages::floatStamped> MySyncPolicy;
  
  
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub1, sub2);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}


