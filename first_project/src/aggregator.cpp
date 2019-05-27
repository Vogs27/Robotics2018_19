#include "ros/ros.h"
#include <cmath>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <first_project/floatStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <dynamic_reconfigure/server.h>
#include <first_project/aggparametersConfig.h>

#define _USE_MATH_DEFINES

double x = 0.0;
double y = 0.0;
double th = 0.0;

ros::Time current_time, last_time;

class aggregator{

	public:
		aggregator(){

			  message_filters::Subscriber<first_project::floatStamped> differential(n, "speedL_stamped", 1);
			  message_filters::Subscriber<first_project::floatStamped> ackermann(n, "speedR_stamped", 1);

			  typedef message_filters::sync_policies::ApproximateTime<first_project::floatStamped, first_project::floatStamped> MySyncPolicy;
			  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), differential, ackermann);

			  current_time = ros::Time::now();
			  odom_pub = n.advertise<nav_msgs::Odometry>("world", 50);

			  sync.registerCallback(boost::bind(&aggregator::callback, this, _1, _2));

			  f = boost::bind(&aggregator::callbackParam, this, _1, _2);
			  server.setCallback(f);
			ros::spin();
		}

	private:
		ros::NodeHandle n;
		tf::TransformBroadcaster odom_broadcaster; //transformation broadcaster
		geometry_msgs::TransformStamped odom_trans;
		ros::Publisher odom_pub; //odometry topic publisher
		dynamic_reconfigure::Server<first_project::aggparametersConfig> server;
		dynamic_reconfigure::Server<first_project::aggparametersConfig>::CallbackType f;
		int steeringMode;

	void callbackParam(first_project::aggparametersConfig &config, uint32_t level) { //dynamic_reconfigure callback method
		steeringMode = config.odom_mode;
		ROS_INFO("Reconfigure request: steering param: %d", config.odom_mode);
	}

	void callback(const first_project::floatStampedConstPtr& speedL, const first_project::floatStampedConstPtr& speedR) //subscriber and message filter callback method
		{

		    if(steeringMode == 0){

		    }
		    else{

		    }

		}

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "aggregator");
  aggregator my_aggregator;
  ros::spin();

  return 0;
}
