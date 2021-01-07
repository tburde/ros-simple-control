#!/usr/bin/env python

import rospy
import math

from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from rosgraph_msgs.msg import Clock

class position(object):
	def __init__(self, fl, fr, rl, rr):
		self.fl = fl
		self.fr = fr
		self.rl = rl
		self.rr = rr

class run_stats(object):
	
	

	def __init__(self,pub):
		self._pub = pub
		#self.last_pos.fl = 0
		#self.last_pos.fr = 0
		#self.last_pos.rl = 0
		#self.last_pos.rr = 0
		#self._
		self._last_pos = position(0, 0, 0, 0)
		self._total_move = position(0, 0, 0, 0)
		self._average_move = 0
		self._elapsed_time = 0
		self._start_time = 0
		self._start = True

	def callback(self,msg):
		fl = msg.position[0]
		fr = msg.position[1]
		rl = msg.position[2]
		rr = msg.position[3]
		self.current_pos = position(fl, fr, rl, rr)
		#rospy.loginfo('Current: FL: {}, FR: {}'.format(fl,fr))
		if self._last_pos.fl != 0 or self._last_pos.fr != 0 or self._last_pos.rl != 0 or self._last_pos.rr != 0:
			self.compare(self.current_pos, self._last_pos)
		
		#rospy.loginfo('Total: FL: {}, FR: {}'.format(self._total_move.fl,self._total_move.fr))
		
		msg = Float32MultiArray()
		msg.data = [self._average_move*0.041, self._total_move.fl*0.041, self._total_move.fr*0.041, self._total_move.rl*0.041, self._total_move.rr*0.041]		
		self._pub.publish(msg)
		self._last_pos.fl = fl
		self._last_pos.fr = fr
		self._last_pos.rl = rl
		self._last_pos.rr = rr
		self._print_stats()
	def check_time(self, msg):
		time_now = msg.clock.secs + (msg.clock.nsecs/1e9)
		if self._start:
			self._start_time = time_now
			self._start = False
		else: 
			self._elapsed_time = time_now - self._start_time 
		self._print_stats()
	def compare(self,current, last):
		
		if round(abs(current.fl),2) != round(abs(last.fl),2):
			self._total_move.fl += abs(current.fl - last.fl)

		if round(abs(current.fr),2) != round(abs(last.fr),2):
			self._total_move.fr += abs(current.fr - last.fr)

		if round(abs(current.rl),2) != round(abs(last.rl),2):
			self._total_move.rl += abs(current.rl - last.rl)

		if round(abs(current.rr),2) != round(abs(last.rr),2):
			self._total_move.rr += abs(current.rr - last.rr)
		
		self._average_move = (self._total_move.fl+self._total_move.fr+self._total_move.rl+self._total_move.rr)/4
	def _print_stats(self):
		rospy.loginfo('Average Movement: {:.3f}, Elapsed Time: {:.2f}'.format(self._average_move*0.041, self._elapsed_time))
		

def main():
	rospy.init_node('run_stats')
	pub = rospy.Publisher('distance_travel', Float32MultiArray, queue_size = 1)
	stats = run_stats(pub)
	rospy.Subscriber("/joint_states", JointState, stats.callback)
	rospy.Subscriber("/clock", Clock, stats.check_time)
	rospy.spin()


if __name__ == '__main__':
	main()

