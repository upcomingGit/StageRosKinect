#include <stdio.h>
#include <time.h>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

using namespace ros;

class roomba
{
public:

	static const float RANGE_ANGLE_MIN = (-40.0*M_PI)/180.0;
	static const float RANGE_ANGLE_MAX = (40.0*M_PI)/180.0;
	static const float FORWARD_SPEED = 0.5;
	static const float THRESHOLD = 0.5;

	roomba();
	void start();
	//void obstacle(const sensor_msgs::LaserScan::ConstPtr& laser);
	void walk();
	void randomTurn();
	void laserCallback(const sensor_msgs::LaserScan::ConstPtr& laser);

private:

	NodeHandle n;
	Publisher pub;
	Subscriber sub;

	geometry_msgs::Twist angle, forward;
};

roomba::roomba() 
{
	pub = n.advertise<geometry_msgs::Twist>("robot_2/cmd_vel", 1000);
	sub = n.subscribe("robot_2/base_scan", 1000, &roomba::laserCallback, this);
}

void roomba::laserCallback(const sensor_msgs::LaserScan::ConstPtr& laser)
{
	//ROS_INFO_STREAM("Reaching the callback");

	int min = (laser->angle_max + RANGE_ANGLE_MIN)/laser->angle_increment;
	int max = (laser->angle_max + RANGE_ANGLE_MAX)/laser->angle_increment;

	float closest = laser->ranges[min];

	for (int i=min+1; i <= max; i++)
	{	
		if (laser->ranges[i] < closest)
			closest = laser->ranges[i];
	}
	if (closest < THRESHOLD)
		this->randomTurn();
	else
		this->walk();
}

void roomba::randomTurn()
{
	float randomAngle = 0;
	randomAngle = clock()/3000000.0;
	ROS_INFO_STREAM("Turning" << randomAngle);
	angle.angular.z = randomAngle;
	pub.publish(angle);
}

void roomba::walk()
{
	forward.linear.x = FORWARD_SPEED;
	pub.publish(forward);
}

void roomba::start()
{
	//ROS_INFO_STREAM("This is the starting node");
	spinOnce();
}


int main(int argc, char** argv)
{
	ROS_INFO_STREAM("Starting the Swarming node...");
	
	init(argc, argv, "robot_2");

	roomba Roomba1;

	Rate rate_loop(1000);

	while (ok())
	{
		Roomba1.start();
		rate_loop.sleep();
	}
	return 0; 
}