#!/usr/bin/env python

## Set Heading Action Server (Python)
#
#	Set target heading Action Server written for ROSbot Competition
#
#	Written by: Tanmay Burde


import rospy
import math
import time

import actionlib

from rosbot_bath.msg import SetHeadingAction, SetHeadingGoal, SetHeadingResult, SetHeadingFeedback

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

class set_heading(object):
	
	

	def __init__(self,pub):
		self._pub = pub
		self._server = actionlib.SimpleActionServer('set_heading', SetHeadingAction, self.callback, False)
		self._as = self._server
		self._as.start()
		self._yaw = 0
		self._yaw_t = 0
		self._reach = False
		self._vel = Twist()
	def CYawCallback(self, msg):
		self._yaw = msg.z
		#rospy.loginfo('Yaw {}, Target {}'.format(self._yaw, self._yaw_t))
	def orient(self,goal):
		
		yaw = self._yaw
		if yaw < 0:
			yaw = 360-abs(yaw)
		else:
			yaw = yaw

		rospy.loginfo('Yaw {}, Target {}'.format(yaw, self._yaw_t))
		if self._yaw_t-yaw > 1.5 and abs(self._yaw_t-yaw) > 180:
			self._vel.angular.z = -(((abs(yaw-self._yaw_t)-180)/180)+0.3) #0.2
		elif self._yaw_t-yaw < -1.5 and abs(self._yaw_t-yaw) >180:
			self._vel.angular.z = (((abs(yaw-self._yaw_t)-180)/180)+0.3)
		elif self._yaw_t-yaw < -1.5 and abs(self._yaw_t-yaw) <180:
			self._vel.angular.z = -(((abs(yaw-self._yaw_t))/180)+0.3)
		elif self._yaw_t-yaw > 1.5 and abs(self._yaw_t-yaw) <180:
			self._vel.angular.z = (((abs(yaw-self._yaw_t))/180)+0.3)
		else:
			self._vel.angular.z = 0
			self._reach = True
		self._pub.publish(self._vel)
		feedback = SetHeadingFeedback()
		feedback.yaw_f = yaw
		self._as.publish_feedback(feedback)
	def callback(self, goal):
		self._yaw_t = goal.yaw
		while(not self._reach):
			if self._as.is_preempt_requested():
				result = SetHeadingResult()
				result.yaw_a = self._yaw
				self._as.set_preemtped(result, "Heading Cancelled")
				return
			self.orient(goal.yaw)
			time.sleep(0.1)
			yaw_current = self._yaw
		result = SetHeadingResult()
		result.yaw_a = yaw_current #self._yaw
		self._as.set_succeeded(result, "Heading Set")
		self._vel.angular.z = 0	
		self._pub.publish(self._vel)
		self._reach = False
	
def main():
	rospy.init_node('set_heading_as')
	pub = rospy.Publisher('avoid_vel', Twist, queue_size=5) 
	head = set_heading(pub)#, server)
	rospy.Subscriber("/rpy", Vector3, head.CYawCallback)
	
	rospy.spin()
	
if __name__ == '__main__':
	main()

