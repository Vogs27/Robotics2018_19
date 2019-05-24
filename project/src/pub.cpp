#include "ros/ros.h"
//with this we can implement a tf broadcaster that make the task of publishing transforms easier.
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <first_project/floatStamped.h>

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
	ros::init(argc, argv, "subscribe_and_publish");
	car_sub_pub my_car_sub_pub;
	sync.registerCallback(boost::bind(callback, _1, _2));
	ros::spin();
	return 0;
}
