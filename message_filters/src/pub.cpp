#include "ros/ros.h"
#include "geometry_msgs/Vector3Stamped.h"

#include <sstream>


int main(int argc, char **argv){
    
	ros::init(argc, argv, "publisher");
	ros::NodeHandle n;

	ros::Publisher pub1 = n.advertise<geometry_msgs::Vector3Stamped>("topic1", 1000);
	ros::Publisher pub2 = n.advertise<geometry_msgs::Vector3Stamped>("topic2", 1000);

	ros::Rate loop_rate(1);

	int count = 0;
  
  
  	while (ros::ok()){
	    
		geometry_msgs::Vector3Stamped msg1;
		geometry_msgs::Vector3Stamped msg2;
		
		msg1.header.stamp = ros::Time::now();
		msg1.header.frame_id = "f1";
		msg1.vector.x = 1;
		msg1.vector.y = 1;
		msg1.vector.z = 1;
		
		
		msg2.header.stamp = ros::Time::now();
		msg2.header.frame_id = "f2";
		msg2.vector.x = 2;
		msg2.vector.y = 2;
		msg2.vector.z = 2;



    		pub1.publish(msg1);
    		pub2.publish(msg2);
    		ROS_INFO ("Publishing message");

    		ros::spinOnce();

    		loop_rate.sleep();
    		++count;
  	}


  	return 0;
}
