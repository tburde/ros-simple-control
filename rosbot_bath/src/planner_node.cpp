/** Planner Node (C++)
*
*	ROS Planner for ROSbot Competition
*
*	Written by: Tanmay Burde
**/
#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include "rosgraph_msgs/Clock.h"

#include <ros/console.h>

#define MIN_AVOID_TIME 0.6 //2
#define MIN_MOVE_TIME 0.6 //2
#define TRAP_TIME 5

ros::Time last_avoid_t;
ros::Time c_time;
static geometry_msgs::Twist vel_msg, heading, avoider;
static bool avoiding, home, col_e;
static bool init = true;
static int trap_count = 0;

ros::Publisher cmd_vel_t_pub;


int prioritise(int priority = 1)
{
	if (priority == 1 || avoiding) 
	{
		return 1;
	}
	else if (col_e && !avoiding && home)
	{
		return 4;
	}
	else if ((!avoiding && last_avoid_t.toSec() < (c_time.toSec()-MIN_MOVE_TIME-MIN_AVOID_TIME)) && !home && !col_e )
	{
		return 5;
	}
	else if ((!avoiding && last_avoid_t.toSec() < (c_time.toSec()-MIN_MOVE_TIME-MIN_AVOID_TIME)) && !home )
	{
		return 2;
	}
	else if (!avoiding && last_avoid_t.toSec() > (c_time.toSec()-MIN_AVOID_TIME))
	{
		return 3;
	}
	else
	{
		return 10;
	}
}

void move(int priority)
{
	
	priority = prioritise(priority);
	//ROS_INFO("Trap Count: %d", trap_count);
	//ROS_INFO("Priority %d, Current Time %f, Last Avoid Time %f,  ", priority, c_time.toSec(),last_avoid_t.toSec() );
	switch (priority){
		case 1:
			vel_msg = avoider;
		break;
		
		case 2:
			vel_msg = heading;
			vel_msg.linear.x = 0.2; //0
		break;

		case 3:
			vel_msg.linear.x = 0.3; //0.2
			vel_msg.angular.z = 0;
		break;
		
		case 4:
			vel_msg.linear.x = 0.4; //0.2
			vel_msg.angular.z = 0;
		break;
		case 5:
			vel_msg.linear.x = 0.55; //0.2
			vel_msg.angular.z = heading.angular.z;
		break;
		
		default:
			vel_msg.linear.x = 0.8; //1.2
			vel_msg.angular.z = 0;
		break;
	}
	
	if(cmd_vel_t_pub)
	{
		cmd_vel_t_pub.publish(vel_msg);
	}
}

void CallbackHeading(const geometry_msgs::Twist::ConstPtr& msg)
{
	heading.linear = msg->linear;
	heading.angular = msg->angular;
	move(2);
}

void CallbackAvoid(const geometry_msgs::Twist::ConstPtr& msg)
{
	avoider.linear = msg->linear;
	avoider.angular = msg->angular;
	if (avoiding || init)
	{	
		last_avoid_t = c_time;
		init = false;
	}
	move(1);
}

void CallbackAvoid_Poll(const std_msgs::Bool::ConstPtr& msg)
{
	avoiding = !msg->data;
	if (avoiding && (last_avoid_t.toSec() > (c_time.toSec()-TRAP_TIME)))
	{
		trap_count++;
	}
	else if (avoiding && (last_avoid_t.toSec() < (c_time.toSec()-TRAP_TIME)))
	{
		trap_count = 0;
	}
	
	
}

void CallbackHome_Poll(const std_msgs::Bool::ConstPtr& msg)
{
	home = msg->data;
	
}

void CallbackCollision_Poll(const std_msgs::Bool::ConstPtr& msg)
{
	col_e = msg->data;
	
	
}

void CallbackClock(const rosgraph_msgs::Clock::ConstPtr& msg)
{
	c_time = msg->clock;
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "planner");
	ros::NodeHandle n;
	cmd_vel_t_pub = n.advertise<geometry_msgs::Twist>("cmd_vel_target", 10);
	ros::Subscriber sub1 = n.subscribe("heading_vel", 100, CallbackHeading);
	ros::Subscriber sub2 = n.subscribe("clock", 100, CallbackClock);
	ros::Subscriber sub3 = n.subscribe("avoid_vel", 100, CallbackAvoid);
	ros::Subscriber sub4 = n.subscribe("avoiding", 100, CallbackAvoid_Poll);
	ros::Subscriber sub5 = n.subscribe("heading_home", 100, CallbackHome_Poll);
	ros::Subscriber sub6 = n.subscribe("collision_expected", 100, CallbackCollision_Poll);
	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
}
