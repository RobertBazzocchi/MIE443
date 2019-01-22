#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <eStop.h>

#include <stdio.h>
#include <cmath>
#include <ctime>

using namespace std;

double angular = 0.0;
double linear = 0.0;

double BUMPER_STATE = 0;

void delaySec(double seconds)
{
	std::clock_t start;
	double duration = 0.0;

	printf("WAITING");

	while (duration < seconds)
	{
		start = std::clock();
		duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
	}
	printf("DONE WAITING");
}

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
	BUMPER_STATE = msg->state;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	// fill me in
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	teleController eStop;

	ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
	ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);

	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

	geometry_msgs::Twist vel;
	
	linear = 0.2;
	//angular = 0.5;
	while(ros::ok()){
		ros::spinOnce();	// UPDATE VALUES IN CALLBACK SUBSCRIPTION
		//.....**E-STOP DO NOT TOUCH**.......
		eStop.block();
		//...................................

		// fill me in
		if (BUMPER_STATE == 1)
		{
			linear = 0.0;
			angular = 0.5;

			vel.angular.z = angular;
  			vel.linear.x = linear;

  			vel_pub.publish(vel);

			//delaySec(0.1);
		}
		

  		vel.angular.z = angular;
  		vel.linear.x = linear;

  		vel_pub.publish(vel);
	}

	return 0;
}
