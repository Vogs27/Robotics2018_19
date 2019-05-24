#include "ros/ros.h"
#include "std_msgs/String.h"
#include "custom_messages/Num.h"

#include <sstream>


int main(int argc, char **argv){
    
	ros::init(argc, argv, "talker");
	ros::NodeHandle n;

	ros::Publisher chatter_pub = n.advertise<custom_messages::Num>("chatter", 1000);

	ros::Rate loop_rate(10);

	int count = 0;
  
  
  	while (ros::ok()){
	    
	    	static int i=0;
		i=(i+1)%1000;
		custom_messages::Num msg;
		msg.num =i;
		chatter_pub.publish (msg);

  	}


  	return 0;
}
