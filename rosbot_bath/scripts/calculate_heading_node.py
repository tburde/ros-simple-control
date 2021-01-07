#!/usr/bin/env python

import rospy
import math

from rosbot_bath.srv import InitYaw, InitYawResponse

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class heading(object):
	
	

	def __init__(self,pub, pub_h):
		self._pub = pub
		self._pub_h = pub_h
		self.init_yaw = 0
		self.init_yaw2 = 0
		self.init_bool = -1
		#self.yaw = 0
		self._vel = Twist()
		self._avoiding = False
		self._facing_home = True
	def callback(self,msg):
		
		yaw = msg.z #(180/math.pi)

		if yaw < 0:
			yaw = 360-abs(yaw)
		else:
			yaw = yaw
		if self.init_bool < 0:
			self.init_yaw = yaw
			self.init_bool = 0
			print(self.init_bool)
		#self.init_yaw = 270
		rospy.loginfo('Yaw {}, Init {}'.format(yaw, self.init_yaw))
		#if self._avoiding:
		self._correct_yaw(yaw)
			
		#msg = Vector3()
		#msg.z = self.init_yaw
		self._pub_h.publish(self._facing_home)	
	def send_init_yaw(self, req):
		return self.init_yaw

	def _correct_yaw(self, yaw):
		if self.init_yaw-yaw > 1.5 and abs(self.init_yaw-yaw) > 180:
			self._vel.angular.z = -(((abs(yaw-self.init_yaw)-180)/180)+0.3) #0.2
			self._facing_home = False
		elif self.init_yaw-yaw > 1.5 and abs(self.init_yaw-yaw) >180:
			self._vel.angular.z = (((abs(yaw-self.init_yaw)-180)/180)+0.3)
			self._facing_home = False
		elif self.init_yaw-yaw < -1.5 and abs(self.init_yaw-yaw) <180:
			self._vel.angular.z = -(((abs(yaw-self.init_yaw))/180)+0.3)
			self._facing_home = False
		elif self.init_yaw-yaw > 1.5 and abs(self.init_yaw-yaw) <180:
			self._vel.angular.z = (((abs(yaw-self.init_yaw))/180)+0.3)
			self._facing_home = False
		else:
			self._vel.angular.z = 0
			self._facing_home = True
		self._pub.publish(self._vel)
		print(self._facing_home)
	
	def avoid_poll(self, msg):
		self._avoiding = msg.data
		#print(msg.data)

def main():
	rospy.init_node('heading')
	pub = rospy.Publisher('heading_vel', Twist, queue_size=1)
	pub_h = rospy.Publisher('heading_home', Bool, queue_size=5) 
	head = heading(pub, pub_h)
	rospy.Service('req_init_yaw', InitYaw, head.send_init_yaw)
	rospy.Subscriber("/rpy", Vector3, head.callback)
	rospy.Subscriber("avoiding", Bool, head.avoid_poll)
	rospy.spin()
	
if __name__ == '__main__':
	main()

