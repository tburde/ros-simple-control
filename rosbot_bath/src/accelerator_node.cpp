#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "rosgraph_msgs/Clock.h"

#include <ros/console.h>

#define LIN_A 0.50 //0.4
#define LIN_D 1.00 //0.6
#define ANG_A 0.500 //10
#define ANG_D ANG_A //10

#define LIN_JERK 0.3

ros::Time last_t;
ros::Time c_time;
static geometry_msgs::Twist last_command, command;
static bool init = true;

ros::Publisher cmd_vel_pub;




void CallbackCommand(const geometry_msgs::Twist::ConstPtr& msg)
{
	command.linear.x = msg->linear.x;
	command.angular.z = msg->angular.z;
	

	if (((command.linear.x - last_command.linear.x) < LIN_JERK) && command.linear.x <= 0.3 )
	{ 
		//command.linear.x = last_command.linear.x + ((c_time.toSec() - last_t.toSec())*(LIN_A));
	}
	else if ((last_command.linear.x < command.linear.x) && (last_command.linear.x + (c_time.toSec() - last_t.toSec())*(LIN_A) < command.linear.x))
	{ 
		command.linear.x = last_command.linear.x + ((c_time.toSec() - last_t.toSec())*(LIN_A));
	}
	else if ((last_command.linear.x > command.linear.x) && (last_command.linear.x - (c_time.toSec() - last_t.toSec())*(LIN_D) > command.linear.x))
	{
		command.linear.x = last_command.linear.x - ((c_time.toSec() - last_t.toSec())*(LIN_D));
	}

// ------------------------------------------------------------

	if (command.angular.z == 0)
	{
		//command.angular.z = last_command.angular.z + ((c_time.toSec() - last_t.toSec())*(ANG_A));
	}
	else if ((last_command.angular.z < command.angular.z) && (last_command.angular.z + (c_time.toSec() - last_t.toSec())*(ANG_A) < command.angular.z))
	{
		command.angular.z = last_command.angular.z + ((c_time.toSec() - last_t.toSec())*(ANG_A));
	}
	else if ((last_command.angular.z > command.angular.z) && (last_command.angular.z - (c_time.toSec() - last_t.toSec())*(ANG_D) > command.angular.z))
	{
		command.angular.z = last_command.angular.z - ((c_time.toSec() - last_t.toSec())*(ANG_D));
	}	
	
	cmd_vel_pub.publish(command);
	
	last_command.linear.x = command.linear.x;
	last_command.angular.z = command.angular.z;	
	last_t = c_time;
}



void CallbackClock(const rosgraph_msgs::Clock::ConstPtr& msg)
{
	c_time = msg->clock;
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "accelerator");
	ros::NodeHandle n;
	cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	ros::Subscriber sub1 = n.subscribe("cmd_vel_target", 100, CallbackCommand);
	ros::Subscriber sub2 = n.subscribe("clock", 100, CallbackClock);
	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
}
