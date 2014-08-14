#!/usr/bin/env python

import roslib; roslib.load_manifest('crab_state_machine')
import rospy
import smach
import smach_ros

#define state follower
class follower_1(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['outcome1'])

	def execute(self):
		rospy.logino('Executing State follower_1')
		self.joy_sub = rospy.Subscriber("/joy",Joy,self.joy_callback)
		#Service to Start Follower
		while (self.a_bool == 0):
			rospy.logino('follower_1 State Waiting')
		#Service to stop Follower
		#Kill Joy Subscriber
		#Return Outcome1

	def joy_callback (self, joy_values):
		self.a_bool = 0
		self.buttons = joy._values.buttons
		if (self.buttons[0] == 1):
			self.a_bool = 1

class wanderer(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['outcome2'])

	def execute(self):
		rospy.logino('Executing State wanderer')
		self.joy_sub = rospy.Subscriber("/joy",Joy,self.joy_callback)
		#Service to Start Follower
		while (self.a_bool == 0):
			rospy.logino('wanderer State Waiting')
		#Service to stop Follower
		#Kill Joy Subscriber
		#Return Outcome2

	def joy_callback (self, joy_values):
		self.a_bool = 0
		self.buttons = joy._values.buttons
		if (self.buttons[0] == 1):
			self.a_bool = 1

class follower_2(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['outcome3'])

	def execute(self):
		rospy.logino('Executing State follower_2')
		self.joy_sub = rospy.Subscriber("/joy",Joy,self.joy_callback)
		#Service to Start Follower
		while (self.a_bool == 0):
			rospy.logino('follower_2 State Waiting')
		#Service to stop Follower
		#Kill Joy Subscriber
		#Return Outcome3

	def joy_callback (self, joy_values):
		self.a_bool = 0
		self.buttons = joy._values.buttons
		if (self.buttons[0] == 1):
			self.a_bool = 1			

class sit(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['outcome4'])

	def execute(self):
		rospy.logino('Executing State sit')
		self.joy_sub = rospy.Subscriber("/joy",Joy,self.joy_callback)
		#Service to Start Follower
		while (self.a_bool == 0):
			rospy.logino('sit State Waiting')
		#Service to stop Follower
		#Kill Joy Subscriber
		#Return Outcome4

	def joy_callback (self, joy_values):
		self.a_bool = 0
		self.buttons = joy._values.buttons
		if (self.buttons[0] == 1):
			self.a_bool = 1			

def main():
	rospy.init_node(crab_controller)

	#Create SMACH State MAachine
	sm = smach.StateMachine(=['outcome4'])

	#open the container
