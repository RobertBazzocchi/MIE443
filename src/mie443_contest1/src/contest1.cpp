#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <eStop.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <stdio.h>
#include <cmath>
#include <ctime>

using namespace std;

// CALLBACK VARIABLES
bool bumperLeft = false, bumperCenter = false, bumperRight = false;

// Odometry
double posX, posY, yaw;
double pi = 3.1416;

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
	if (msg->bumper == 0)
	{
		bumperLeft = !bumperLeft;
	}
	else if (msg->bumper == 1)
	{
		bumperCenter = !bumperCenter;
	}
	else if (msg->bumper == 2)
	{
		bumperRight = !bumperRight;
	}
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	// fill me in
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	posX = msg->pose.pose.position.x;
	posY = msg->pose.pose.position.y;
	yaw = tf::getYaw(msg->pose.pose.orientation);

	ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees.", posX, posY, yaw, yaw*180.0/pi);
}

bool isBumperPressed()
{
	return bumperRight || bumperCenter || bumperLeft;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	teleController eStop;

	// Subscritpions
	ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
	ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
	ros::Subscriber odom = nh.subscribe("/odom", 1, odomCallback);

	// Publishers
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

	geometry_msgs::Twist vel;
	
	double linear = 0.0;
	double angular = 0.0;
	while(ros::ok())
	{
		// UPDATE VALUES IN CALLBACK SUBSCRIPTION
		ros::spinOnce();	

		//.....**E-STOP DO NOT TOUCH**.......
		eStop.block();
		//...................................

		angular = 0.0;
		linear = 0.0;

		if (!isBumperPressed())
		{
			if (posX < 0.5 && yaw < pi/12)
			{
				angular = 0.0;
				linear = 0.2;
			}
			else if (yaw < pi/2 && posX > 0.5)
			{
				angular = pi/6;
				linear = 0.0;
			}
		}
		else
		{
			angular = 0.0;
			linear = 0.0;
		}
		

  		vel.angular.z = angular;
  		vel.linear.x = linear;

  		vel_pub.publish(vel);
	}

	return 0;
}
