#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <eStop.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <random>

#include <cmath>

#define LIN_SPEED 0.15
#define OBSTICAL_DIST 0.5

using namespace std;

std::random_device rd;
std::mt19937 gen(rd());
std::bernoulli_distribution bernoulli(0.4);

bool turnLeft = true;

// CALLBACK VARIABLES
bool bumperLeft = false, bumperCenter = false, bumperRight = false;
bool isBumperHit = false;
bool spin = false;

// Odometry
bool isTurning = false;
double posX, posX_Init = 0.0, posX_Spin = 0.0;
double posY, posY_Init = 0.0, posY_Spin = 0.0; 
double yaw, yawInitial, yawDesired;
double pi = 3.1416;

// Laser Scan
double laserRangeLeft = 10, laserRangeRight = 10;
int laserSize = 0, laserOffset = 0, desiredAngle = 20;

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
	ROS_INFO("BUMPER CALL BACK!!");
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
	ROS_INFO("LASER CALL BACK!!");
	laserSize = (msg->angle_max - msg->angle_min)/msg->angle_increment;
	laserOffset = desiredAngle * pi / (180.0 * msg->angle_increment);

	for (int i = laserSize/2 - laserOffset; i < laserSize/2; ++i)
	{
		// ROS_INFO("laser size: %d, laserOffset: %d", laserSize/2, laserOffset);
		if (laserRangeLeft > msg->ranges[i])
		{
			laserRangeLeft = msg->ranges[i];
		}
	}
	for (int i = laserSize/2 + laserOffset; i >= laserSize/2; --i)
	{
		if (laserRangeRight > msg->ranges[i])
		{
			laserRangeRight = msg->ranges[i];
		}
	}
	
	//ROS_INFO("Distance Reading Left: %lf Right: %lf", laserRangeLeft, laserRangeRight);
	//ROS_INFO("Size of laser scan array: %i and size of offset: %i", laserSize, laserOffset);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	ROS_INFO("ODOM CALL BACK!!");
	posX = msg->pose.pose.position.x;
	posY = msg->pose.pose.position.y;
	yaw = tf::getYaw(msg->pose.pose.orientation);

	if (isTurning && std::abs(yaw - yawInitial) > pi / 2.5)
	{
		isTurning = false;
	}

	ROS_INFO("YAW_DIFF: %lf isTurning: %d", std::abs(yaw - yawInitial), isTurning);
	ROS_INFO("YAW: %lf", yaw);
}

bool isBumperPressed()
{
	return bumperRight || bumperCenter || bumperLeft;
}

double distFromInit()
{
	std::sqrt(std::pow(posX - posX_Init, 2) + std::pow(posY - posY_Init, 2));
}

double distFromLastSpin()
{
	std::sqrt(std::pow(posX - posX_Spin, 2) + std::pow(posY - posY_Spin, 2));
}

bool isObstical(double dist)
{
	return dist < OBSTICAL_DIST;
}

bool isObstical()
{
	return isObstical(laserRangeLeft) || isObstical(laserRangeRight);
}

void setTurnDirection()
{
	turnLeft = isObstical(laserRangeLeft) ? true : false;
	ROS_INFO ("turn left? %d", turnLeft);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	teleController eStop;

	// Subscritpions
	ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
	ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
	ros::Subscriber odom = nh.subscribe("/odom", 1, &odomCallback);

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
		linear = LIN_SPEED;


		if (!isBumperPressed())
		{
			if (isObstical() && !isTurning)
			{
				ROS_INFO("OBSTICAL DETECTED"); 
				yawInitial = yaw;
				isTurning = true;
				setTurnDirection();
			}

			if (laserRangeLeft > 0.5 && laserRangeLeft < 0.7)
			{
				linear = 0.8 * LIN_SPEED;	
			}
			else if (laserRangeRight > 0.5 && laserRangeRight < 0.7)
			{
				linear = 0.8 * LIN_SPEED;	
			}

			double dist = distFromLastSpin();
			if (!spin && dist > 0.2)
			{
				// ROS_INFO("yawDesired: %lf, yaw: %lf", yawDesired, yaw); 
				if (yaw < 0)
				{
					yawDesired = yaw + 2*pi;
				}
				else
				{
					yawDesired = yaw;
				}
				spin = true;
			}
		}
		else
		{
			posX_Init = posX;
			posY_Init = posY;
			isBumperHit = true;
		}

		if (isBumperHit)
		{
			if (distFromInit() < 0.2)
			{
				ROS_INFO("bumped");
				angular = turnLeft ? pi/6 : -pi/6;
				linear = -LIN_SPEED;
			}
			else
			{
				isBumperHit = false;
			}
		}

		if (isTurning && turnLeft)
		{
			ROS_INFO("turning left");
			angular = pi/6;
			linear = 0.0;	
			ros::spinOnce();
		}
		else if (isTurning && !turnLeft)
		{
			ROS_INFO("turning right");
			angular = -pi/6;
			linear = 0.0;
			ros::spinOnce();
		}

		if (spin)
		{
			ROS_INFO("360 spin");
			angular = pi/6;
			linear = 0.0;
			
			// Stop Condition
			float yawWack = yaw;
			if (yawWack < 0)
			{
				yawWack = 2*pi + yawWack;
			}
			
			//ROS_INFO("lower bound: %lf, yaw: %lf, upper bound: %lf", yawDesired-0.5, yawWack, yawDesired);
			if (yawWack > yawDesired-pi/18 && yawWack < yawDesired)
			{
				ROS_INFO("360 spin finsished");
				spin = false;
				posX_Spin = posX;
				posY_Spin = posY;
			}
		}
		
  		vel.angular.z = angular;
  		vel.linear.x = linear;
		//ROS_INFO("linear %lf, angular %lf", linear, angular);
  		vel_pub.publish(vel);
	}

	return 0;
}
