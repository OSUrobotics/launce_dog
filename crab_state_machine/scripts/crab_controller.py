#!/usr/bin/env python

import roslib; roslib.load_manifest('crab_state_machine')
import rospy
import smach
import smach_ros
from geometry_msgs.msg import Twist
from turtlebot_msgs.srv import SetFollowStateRequest 
from turtlebot_msgs.srv import SetFollowStateResponse 
from turtlebot_msgs.srv import SetFollowState
from sensor_msgs.msg import Joy

#define state follower
class stop(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['outcome1','outcome2'])
		change_state = rospy.ServiceProxy('/turtlebot_follower/change_state', SetFollowState)
		rospy.wait_for_service('/turtlebot_follower/change_state')
		while (change_state(SetFollowStateRequest.STOPPED).result != SetFollowStateResponse.OK):
			pass
		self.count = 0
		self.stop_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist)
		self.a_bool = 0



	def execute(self, userdata):
		rospy.loginfo('Executing State stop')
		self.joy_sub = rospy.Subscriber("/joy",Joy,self.joy_callback)
		self.command = Twist()
		self.command.linear.x = 0.0
		self.command.linear.y = 0.0
		self.command.linear.z = 0.0
		self.command.angular.x = 0.0
		self.command.angular.y = 0.0
		self.command.angular.z = 0.0
		self.stop_pub.publish(self.command)
		if (self.count == 10):
			return 'outcome2'
		self.count = self.count + 1
		while (self.a_bool == 0):
			wait = 1
			rospy.loginfo('stop State Waiting')
		self.joy_sub.unregister()
		self.a_bool = 0 
		return 'outcome1'

	def joy_callback (self, joy_values):
		self.buttons = joy_values.buttons
		if (self.buttons[0] == 1):
			self.a_bool = 1

class follower(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['outcome1'])
		self.a_bool = 0


	def execute(self, userdata):
		rospy.loginfo('Executing State follower')
		self.joy_sub = rospy.Subscriber("/joy",Joy,self.joy_callback)
		
		rospy.wait_for_service('/turtlebot_follower/change_state')
		change_state = rospy.ServiceProxy('/turtlebot_follower/change_state', SetFollowState)
		while (change_state(SetFollowStateRequest.FOLLOW).result != SetFollowStateResponse.OK):
			pass

		while (self.a_bool == 0):
			rospy.loginfo('follower State Waiting')
		while (change_state(SetFollowStateRequest.STOPPED).result != SetFollowStateResponse.OK):
			pass
		self.joy_sub.unregister()
		self.a_bool = 0 
		return 'outcome1'

	def joy_callback (self, joy_values):
		self.buttons = joy_values.buttons
		if (self.buttons[0] == 1):
			self.a_bool = 1

class wanderer(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['outcome1'])
		self.wanderer_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist)
		self.a_bool = 0


	def execute(self, userdata):
		rospy.loginfo('Executing State wanderer')
		self.joy_sub = rospy.Subscriber("/joy",Joy,self.joy_callback)
		#Service to Start Follower
		while (self.a_bool == 0):
			rospy.loginfo('wanderer State Waiting')
		#Service to stop Follower
		self.joy_sub.unregister()
		self.a_bool = 0 
		return 'outcome1'

	def joy_callback (self, joy_values):
		self.buttons = joy_values.buttons
		if (self.buttons[0] == 1):
			self.a_bool = 1

class sit(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['outcome1'])
		self.sit_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist)
		self.a_bool = 0

	def execute(self, userdata):
		rospy.loginfo('Executing State sit')
		self.joy_sub = rospy.Subscriber("/joy",Joy,self.joy_callback)
		self.command = Twist()
		self.command.linear.x = 0.0
		self.command.linear.y = 0.0
		self.command.linear.z = 0.0
		self.command.angular.x = 0.0
		self.command.angular.y = 0.0
		self.command.angular.z = 0.0
		self.sit_pub.publish(self.command)
		while (self.a_bool == 0):
			rospy.loginfo('sit State Waiting')
		self.joy_sub.unregister()
		self.a_bool = 0 
		return 'outcome1'

	def joy_callback (self, joy_values):
		self.buttons = joy_values.buttons
		if (self.buttons[0] == 1):
			self.a_bool = 1			

def main():
	rospy.init_node('crab_controller')

	#Create SMACH State MAachine
	sm = smach.StateMachine(outcomes = ['outcome3'])

	#open the container
	with sm:
		#Add states to the container
		smach.StateMachine.add('stop', stop(),
								transitions = {'outcome1': 'follower_1','outcome2': 'outcome3'})

		smach.StateMachine.add('follower_1', follower(),
								transitions = {'outcome1': 'wanderer'})

		smach.StateMachine.add('wanderer', wanderer(),
								transitions = {'outcome1': 'follower_2'})

		smach.StateMachine.add('follower_2', follower(),
								transitions = {'outcome1': 'sit'})

		smach.StateMachine.add('sit', sit(),
								transitions = {'outcome1': 'stop'})

		
		#Execute SMACH plan
	sis = smach_ros.IntrospectionServer('virtual_smach', sm, '/SM_ROOT')
	sis.start()
	outcome = sm.execute()

	rospy.spin()
	sis.stop()

if __name__ == '__main__':
	main()