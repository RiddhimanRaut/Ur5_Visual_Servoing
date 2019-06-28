#!/usr/bin/env python
import rospy
from ur5_control_nodes.msg import floatList
from sensor_msgs.msg import JointState
import numpy as np
from math import *
from std_msgs.msg import Header
from control_msgs.msg import *
from trajectory_msgs.msg import *

joint_vel=None   
JOINT_NAMES=['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint','wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
def joint_velocity_callback(data):
    global joint_vel
    joint_vel = data.data
    # print eff_vel
    
    
def joint_states(velocity):
    global JOINT_NAMES
    # print velocity
    
    pub = rospy.Publisher('/ur_driver/joint_speed', JointTrajectory, queue_size=10)
    rate = rospy.Rate(125) # 10hz
    hello_str = JointTrajectory()
    hello_str.header = Header()
    hello_str.joint_names=JOINT_NAMES
    hello_str.points=[JointTrajectoryPoint(velocities=velocity, time_from_start=rospy.Duration(0.0)),JointTrajectoryPoint(time_from_start=rospy.Duration(0.0))]
    hello_str.header.seq=hello_str.header.seq+1
    hello_str.header.stamp-rospy.Time.now()
    pub.publish(hello_str)
    rate.sleep()

def run_ur5():
    global joint_vel
    rospy.init_node('run_ur5', anonymous=True)
    

    rospy.Subscriber("ur5_joint_velocities", floatList, joint_velocity_callback)
    while not rospy.is_shutdown():
        joint_states(joint_vel)
    

if __name__ == '__main__':
    
        try:
            run_ur5()
        except rospy.ROSInterruptException:
            pass
