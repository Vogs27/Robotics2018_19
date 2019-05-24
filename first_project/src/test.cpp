//Basic sub_pub with message message_filters
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <first_project/floatStamped.h>

const custom_messages::Num::ConstPtr& msg

void callback(const first_project::floatStamped::ConstPtr& msg1, const first_project::floatStamped::ConstPtr& msg2)
{
  ROS_INFO ("Received two messages: (%f,%f,%f) and (%f,%f,%f)", msg1->vector.x,msg1->vector.y,msg1->vector.z, msg2->vector.x, msg2->vector.y, msg2->vector.z);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "subscriber");

  ros::NodeHandle n;

  message_filters::Subscriber<first_project::floatStamped> sub1(n, "topic1", 1);
  message_filters::Subscriber<first_project::floatStamped> sub2(n, "topic2", 1);
  message_filters::TimeSynchronizer<first_project::floatStamped, first_project::floatStamped> sync(sub1, sub2, 10);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}
