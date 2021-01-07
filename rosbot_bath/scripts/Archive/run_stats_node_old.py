#!/usr/bin/env python

import rospy
import math

from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

class position(object):
	def __init__(self, fl, fr, rl, rr):
		self.fl = fl
		self.fr = fr
		self.rl = rl
		self.rr = rr

class run_stats(object):
	
	last_pos = position(fl, fr, rl, rr)
	

	def __init__(self,pub):
		self._pub = pub
		self.last_pos.fl = 0
		self.last_pos.fr = 0
		self.last_pos.rl = 0
		self.last_pos.rr = 0
		#self._

	def callback(msg):
		fl = msg.position[1]
		fr = msg.position[2]
		rl = msg.position[3]
		rr = msg.position[4]
		rospy.loginfo('FL: {}, FR: {}'.format(fl,fr))
		
		self.last_pos.fl = fl
		self.last_pos.fr = fr
		self.last_pos.rl = rl
		self.last_pos.rr = rr
		
	def compare(x, y):
		
	

def main():
	rospy.init_node('run_stats')
	pub = rospy.Publisher('distance_travel', Float32MultiArray)
	stats = run_stats(pub)
	rospy.Subscriber("/joint_states", JointState, stats.callback)
	rospy.spin()
	
if __name__ == '__main__':
	main()
	
