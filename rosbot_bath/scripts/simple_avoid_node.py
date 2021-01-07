#!/usr/bin/env python

import rospy
import math
import time

import actionlib

from rosbot_bath.srv import *

import rosbot_bath.msg

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool

class avoider(object):
	
	

	def __init__(self,pub, pub_e, req_init_yaw, client):
		self._pub = pub
		self._pub_e = pub_e
		self._req_init_yaw = req_init_yaw
		self._client = client
		resp = self._req_init_yaw(True)
		self._init_yaw = resp.init_yaw
		self._action_pending = False
		self._action_complete = False
		self._stop = Twist()
		self._stop.linear.x = 0
		self._stop.angular.z = 0
		self._reverse = Twist()
		self._reverse.linear.x = -0.1
		self._yaw_target = 180
		self._move = Twist()
		self._move.linear.x = 0.1
		self._yaw_offset = 0
		
	def collide(self,msg):
		if msg.data and not self._action_pending:
			self._pub_e.publish(self._stop)
			self.call_action()
			self._action_pending = True
			self._action_complete = False
			print("Sending Goal")
		elif msg.data and self._action_pending:
			self._pub_e.publish(self._reverse)
			#time.sleep(0.05)
			#time.sleep(0.05)
			#self._pub_e.publish(self._stop)
			#time.sleep(0.01)
			print("Emergency!")
		elif not msg.data and self._action_pending:
			#self._client.cancel_all_goals
			#self._action_pending = False
			#time.sleep(0.005)
			print("waiting")			
		elif not msg.data and not self._action_pending:
			#self._pub_e.publish(self._move)
			print("Free!")
			self._action_complete = True
			#time.sleep(0.001)
		
		self._pub.publish(self._action_complete)
		
	def forward_sense(self, msg):
		#do some processing
		offset = 15
		dist_l = msg.data[0]
		dist_r = msg.data[1]
		ang_l = msg.data[2]
		ang_ml = msg.data[3]
		ang_mr = msg.data[4]
		ang_r = msg.data[5]
		
		if (ang_ml == 0 and ang_mr ==0):
			if ang_l < ang_r:
				#self._yaw_target = self._init_yaw+ang_l+offset
				if (self._init_yaw + ang_l + offset > 360):				
					#self._yaw_target = 360-abs(self._init_yaw+ang_l+offset)
					self._yaw_target = self._init_yaw-abs(360-ang_l-offset)
				else:
					self._yaw_target = self._init_yaw+ang_l+offset
			elif ang_l >= ang_r:
				if (self._init_yaw - ang_r - offset < 0) :				
					self._yaw_target = 360-abs(self._init_yaw-ang_r-offset)
				else:
					self._yaw_target = self._init_yaw-ang_r-offset
		elif (ang_ml+ang_mr)> 30 and dist_l > 1.2 and dist_r >1.2:
			self._yaw_target = self._init_yaw
		elif (ang_ml+ang_mr) < 30: 
			if ang_l < ang_r:
				#self._yaw_target = self._init_yaw+ang_l
				if (self._init_yaw + ang_l + offset  > 360):				
					self._yaw_target = self._init_yaw-abs(360-ang_l-offset)
				else:
					self._yaw_target = self._init_yaw+ang_l+offset
			elif ang_l >= ang_r:
				if (self._init_yaw - ang_r - offset < 0):				
					self._yaw_target = 360-abs(self._init_yaw-ang_r-offset)
				else:
					self._yaw_target = self._init_yaw-ang_r-offset
	def call_action(self):
		self._client.wait_for_server()		
		goal = rosbot_bath.msg.SetHeadingGoal(yaw = self._yaw_target)
		#goal.yaw = self._yaw_target
		self._client.send_goal(goal, done_cb = self._heading_set)
		
	def _heading_set(self, state, result):
		self._action_pending = False
		self._action_complete = True
		self._pub.publish(self._action_complete)
	
	def callback(self, msg):
		yaw = msg.z
		if yaw < 0:
			yaw = 360-abs(yaw)
		else:
			yaw = yaw
		self._yaw_offset = self._init_yaw - yaw
		self._init_yaw = yaw
	
def main():
	rospy.init_node('simple_avoid')
	pub = rospy.Publisher('avoiding', Bool, queue_size=1)
	pub_e = rospy.Publisher('cmd_vel_target', Twist, queue_size=1)
	#pub_h = rospy.Publisher('init_heading', Vector3, queue_size=5) 
	req_init_yaw = rospy.ServiceProxy('req_init_yaw', InitYaw)
	client = actionlib.SimpleActionClient('set_heading', rosbot_bath.msg.SetHeadingAction)
	avoid = avoider(pub, pub_e, req_init_yaw, client)
	rospy.Subscriber("collision_imminent", Bool, avoid.collide)
	#rospy.Subscriber("/set_heading/status", Bool, avoid.collide)
	rospy.Subscriber("fused_forward", Float32MultiArray, avoid.forward_sense)
	rospy.Subscriber("/rpy", Vector3, avoid.callback)
	rospy.spin()
	
if __name__ == '__main__':
	main() 

