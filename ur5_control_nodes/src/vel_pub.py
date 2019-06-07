#!/usr/bin/env python

import rospy
import actionlib
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from control_msgs.msg import *
from trajectory_msgs.msg import *

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

#global k
#to talk to virtual model
def talker():
	global joint_pos
	global theta
	global jvel1
	global jvel2
	global jvel
	theta=0
	pub = rospy.Publisher('/ur_driver/joint_speed', JointTrajectory, queue_size=10)
	rospy.init_node('pttraj')
	rate = rospy.Rate(125) # 10hz
	hello_str = JointTrajectory()
	hello_str.header = Header()
	hello_str.joint_names=JOINT_NAMES
	joint_states=rospy.wait_for_message("joint_states", JointState)
	jvel=joint_states.position[0]
	while not rospy.is_shutdown():
		#joint_states=rospy.wait_for_message("joint_states", JointState)
		#jvel1=joint_states.position[0]
		#jvel2=jvel1-jvel
		#jvel2=jvel2*180/3.14
		#print jvel2*180/3.14
		hello_str.points=[
				JointTrajectoryPoint(velocities=(-1,0,0,0,0,0), time_from_start=rospy.Duration(0.0)),
				JointTrajectoryPoint(time_from_start=rospy.Duration(0.0))]
		theta=theta+1.0/125
		theta1=theta*180/3.14

		print theta1
		pub.publish(hello_str)
		rate.sleep()
		#print k
		hello_str.header.seq=hello_str.header.seq+1
		hello_str.header.stamp = rospy.Time.now()
		pub.publish(hello_str)
		rate.sleep()
if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass