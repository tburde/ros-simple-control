#!/usr/bin/env python

## Sensor Fusion Node (Python)
#
#	Sensor fusion node written for ROSbot Competition
#
#	Written by: Tanmay Burde


import rospy
import math

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool

from rosbot_bath.srv import *


class fusion_core(object):

	def __init__(self,pub1, pub2, pub3, pub4, pub5, pub_e):
		self._pub1 = pub1
		self._pub2 = pub2
		self._pub3 = pub3
		self._pub4 = pub4
		self._pub5 = pub5
		self._pub_e = pub_e
		self._fl = 0
		self._fr = 0
		self._rl = 0
		self._rr = 0
		self._scan_p  = Float32MultiArray()#1,[0,0,0,0])
		self._scan_fl  = Float32MultiArray()#1,[0,0,0,0])
		self._scan_fr  = Float32MultiArray()#1,[0,0,0,0])
		self._scan_l  = Float32MultiArray()#1,[0,0,0,0])
		self._scan_r  = Float32MultiArray()#1,[0,0,0,0])
		self._fused_f = 0
		self._ready = False
	
	def callback_fl(self,msg):

		self._fl = msg.range
		self.fused_power()

	def callback_fr(self,msg):
		
		self._fr = msg.range
		self.fused_power()

	def callback_rl(self,msg):
		
		self._rl = msg.range
		self.fused_power()

	def callback_rr(self,msg):
		
		self._rr = msg.range
		self.fused_power()

	def callback_scan(self,msg):
		#call serivce for processing
		#print("Service calling")
		try:
			segment_scan = rospy.ServiceProxy('lidar_segment', ScanSegment)
			#print("Service created")
			resp = segment_scan(msg.ranges)
			#print("Service called")
			self._scan_fl = resp.scan_fl
			self._scan_fr = resp.scan_fr
			self._scan_l = resp.scan_l
			self._scan_r = resp.scan_r
			#print("Values stored")
		except rospy.ServiceException as e:
			print("Service call failed: %s"%e)
		self._ready = True
		self.fused_power()

	def fused_power(self):
		col_dis = 0.3
		col_exp = 1.2
		if self._ready:
		
			temp_fusor = [0, 0, 0, 0, 0, 0]
			temp_fusor[2] = self._scan_fl[3]
			temp_fusor[3] = self._scan_fl[2]
			if (self._scan_fr[2] > 0):
				temp_fusor[4] = 360-self._scan_fr[2]
			if (self._scan_fr[3] > 0):
				temp_fusor[5] = 360-self._scan_fr[3]
			if self._fl <0.8:
				temp_fusor[0] = min(self._scan_fl[4], 0.05+float(self._fl))
			else:
				temp_fusor[0] = self._scan_fl[4]
			if self._fr <0.8:
				temp_fusor[1] = min(self._scan_fr[4], 0.05+float(self._fr))
			else:
				temp_fusor[1] = self._scan_fr[4]
			#print(self._scan_fl[4]- float(self._fl))
			msg_p = Float32MultiArray(data = temp_fusor)
		
			if (temp_fusor[0] < col_dis and self._scan_fl[4] != 0) or (temp_fusor[1] < col_dis and self._scan_fr[4] != 0):
				msg_bool = True
				self._pub_e.publish(Twist())	 #emergency stop			
			else:
				msg_bool = False

			if (temp_fusor[0] < col_exp and self._scan_fl[4] != 0) or (temp_fusor[1] < col_exp and self._scan_fr[4] != 0):
				msg_bool_e = True
			else:
				msg_bool_e = False
			self._pub1.publish(msg_p)
			self._pub2.publish(msg_bool)
			self._pub3.publish(Float32MultiArray(data =self._scan_l))
			self._pub4.publish(Float32MultiArray(data =self._scan_r))
			self._pub5.publish(msg_bool_e)

	

def main():
	rospy.init_node('fusion_reactor')
	pub1 = rospy.Publisher('fused_forward', Float32MultiArray, queue_size=5)
	pub2 = rospy.Publisher('collision_imminent', Bool, queue_size=5)
	pub3 = rospy.Publisher('left_segment', Float32MultiArray, queue_size=5)
	pub4 = rospy.Publisher('right_segment', Float32MultiArray, queue_size=5)
	pub5 = rospy.Publisher('collision_expected', Bool, queue_size=5)
	pub_e = rospy.Publisher('cmd_vel_target', Twist, queue_size=5)	
	uranium = fusion_core(pub1, pub2, pub3, pub4, pub5, pub_e)
	rospy.Subscriber("/range/fl", Range, uranium.callback_fl)
	rospy.Subscriber("/range/fr", Range, uranium.callback_fr)
	rospy.Subscriber("/range/rl", Range, uranium.callback_rl)
	rospy.Subscriber("/range/rr", Range, uranium.callback_rr)
	rospy.Subscriber("/scan", LaserScan, uranium.callback_scan)
	rospy.spin()
	
if __name__ == '__main__':
	main()

