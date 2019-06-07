#!/usr/bin/env python
import rospy
from ur5_control_nodes.msg import floatList
from sensor_msgs.msg import JointState
import numpy as np
from math import *
from std_msgs.msg import Header
from control_msgs.msg import *
from trajectory_msgs.msg import *


joint_positions=None
coordinates=None
def position_callback(position):
    global joint_positions
    joint_positions=position.position
    


def coordinate_callback(data):
    global coordinates
    coordinates=data.data



def joint_velocities(pose,points):
    
    
    if pose!=None:
        if points!=None:
            theta_x = -pi/2
            theta_y = pi/2
            
            
            int_matrix = None
            fl = 531.15
            desired_points = [93.99359819243081, -51.51572208623612, 837, -84.35322914705328, 56.03464507625683, 814, 100.01882884579176, 55.130860478252686, 817]

            desired_points = np.asarray(desired_points)
            points=np.asarray(points)
            pix_vel=[]
        
            for index in range(len(points)):
                if (index+1)%3==0:
                    index+=1
                else:
                    pix_vel.append(points[index]-desired_points[index])
            # print pix_vel
            pix_vel=np.asarray(pix_vel) #1v8 array
            pix_vel=np.matrix(pix_vel)
            pix_vel=np.ndarray.transpose(pix_vel)
            

            for i in range(0,len(points),3):
                u = desired_points[i]
                v = desired_points[i+1]
                z = desired_points[i+2]
                new_mat=np.array([[ -fl/z, 0, u/z, u*v/fl, -(fl*fl+u*u)/fl, v],[ 0, -fl/z, v/z, (fl*fl+v*v)/fl, -u*v/fl, -u ]])
                # new_mat=np.array([[ 0, 0, u/z,0,0,0],[ 0, 0, v/z, 0,0,0 ]])
                if i==0:
                    int_matrix=new_mat
                else:
                    int_matrix=np.concatenate((int_matrix,new_mat))
            
            
            int_matrix=np.matrix(int_matrix) #8v6 matrix
            inverse=np.linalg.pinv(int_matrix) #6v8 matrix
            
            cam_vel_np=-0.000005*inverse*pix_vel #6v1 matrix
            
            
            cam_vel_lin = cam_vel_np[[0,1,2],:]
            cam_vel_angular = cam_vel_np[[3,4,5],:]
            

            
            
            Rx=np.array([[1,0,0],[0,cos(theta_x),-sin(theta_x)],[0,sin(theta_x),cos(theta_x)]])
            Ry=np.array([[cos(theta_y) , 0, sin(theta_y)],[0, 1, 0],[-sin(theta_y), 0, cos(theta_y)]])
            # Lt=np.array([[1,0,0,-210],[0,1,0,0],[0,0,1,-85],[0,0,0,1]])
            # # print Rx
            # # print Ry
            # cam_vel_lin=Ry*cam_vel_lin
            # cam_vel_lin=Rx*cam_vel_lin
            
            # cam_vel_lin=np.vstack((cam_vel_lin,[1]))
            
            # cam_vel_lin=Lt*cam_vel_lin
            # cam_vel_lin=np.delete(cam_vel_lin,(3),axis=0)
            
            # cam_vel_angular=Rx*(Ry*cam_vel_angular)
            # cam_vel_angular=np.vstack((cam_vel_angular,[1]))
            # cam_vel_angular=Lt*cam_vel_angular
            # cam_vel_angular=np.delete(cam_vel_angular,(3),axis=0)

            cam_vel_np=np.concatenate((cam_vel_lin,cam_vel_angular))
            # cam_vel_np[2]=-cam_vel_np[2]
            # cam_vel_np[5]=-cam_vel_np[5]
            # cam_vel_np=np.array([[0],[0],[0],[0],[0],[0]])
            # cam_vel_np=np.ndarray.transpose(cam_vel_np)
            # cam_vel_list=np.ndarray.tolist(cam_vel_np)
            # cam_vel_final=cam_vel_list[0]
            temp1 = cam_vel_np.item((0,0))
            temp2 = cam_vel_np.item((1,0))
            temp3 = cam_vel_np.item((2,0))
            temp4 = cam_vel_np.item((3,0))
            temp5 = cam_vel_np.item((4,0))
            temp6 = cam_vel_np.item((5,0))
            # cam_vel_np[0,0] = temp3
            # cam_vel_np[1,0] = -temp1
            # cam_vel_np[2,0] = temp2
            # cam_vel_np[3,0] = temp6
            # cam_vel_np[4,0] = -temp4
            # cam_vel_np[5,0] = temp5
            cam_vel_np[0,0] = temp3
            cam_vel_np[1,0] = temp1
            cam_vel_np[2,0] = temp2
            cam_vel_np[3,0] = temp6
            cam_vel_np[4,0] = temp4
            cam_vel_np[5,0] = temp5
        
            
            print cam_vel_np
            
            
            
            Q1=pose[0]
            Q2=pose[1]
            Q3=pose[2]
            Q4=pose[3]
            Q5=pose[4]
            Q6=pose[5]
            jacobian=np.array([[ (2183*np.cos(Q1))/20000 + (823*np.cos(Q1)*np.cos(Q5))/10000 + (17*np.cos(Q2)*np.sin(Q1))/40 - (1569*np.sin(Q1)*np.sin(Q2)*np.sin(Q3))/4000 + (823*np.cos(Q2 + Q3 + Q4)*np.sin(Q1)*np.sin(Q5))/10000 - (591*np.cos(Q2 + Q3)*np.sin(Q1)*np.sin(Q4))/6250 - (591*np.sin(Q2 + Q3)*np.cos(Q4)*np.sin(Q1))/6250 + (1569*np.cos(Q2)*np.cos(Q3)*np.sin(Q1))/4000, np.cos(Q1)*((1569*np.sin(Q2 + Q3))/4000 + (17*np.sin(Q2))/40 + np.sin(Q5)*((823*np.cos(Q2 + Q3)*np.sin(Q4))/10000 + (823*np.sin(Q2 + Q3)*np.cos(Q4))/10000) + (591*np.cos(Q2 + Q3)*np.cos(Q4))/6250 - (591*np.sin(Q2 + Q3)*np.sin(Q4))/6250),                         np.cos(Q1)*((591*np.cos(Q2 + Q3 + Q4))/6250 + (1569*np.sin(Q2 + Q3))/4000 + (823*np.sin(Q2 + Q3 + Q4)*np.sin(Q5))/10000),                         np.cos(Q1)*((591*np.cos(Q2 + Q3 + Q4))/6250 + (823*np.sin(Q2 + Q3 + Q4)*np.sin(Q5))/10000), (823*np.cos(Q1)*np.cos(Q2)*np.cos(Q5)*np.sin(Q3)*np.sin(Q4))/10000 - (823*np.cos(Q1)*np.cos(Q2)*np.cos(Q3)*np.cos(Q4)*np.cos(Q5))/10000 - (823*np.sin(Q1)*np.sin(Q5))/10000 + (823*np.cos(Q1)*np.cos(Q3)*np.cos(Q5)*np.sin(Q2)*np.sin(Q4))/10000 + (823*np.cos(Q1)*np.cos(Q4)*np.cos(Q5)*np.sin(Q2)*np.sin(Q3))/10000, 0],[ (2183*np.sin(Q1))/20000 - (17*np.cos(Q1)*np.cos(Q2))/40 + (823*np.cos(Q5)*np.sin(Q1))/10000 - (823*np.cos(Q2 + Q3 + Q4)*np.cos(Q1)*np.sin(Q5))/10000 + (591*np.cos(Q2 + Q3)*np.cos(Q1)*np.sin(Q4))/6250 + (591*np.sin(Q2 + Q3)*np.cos(Q1)*np.cos(Q4))/6250 - (1569*np.cos(Q1)*np.cos(Q2)*np.cos(Q3))/4000 + (1569*np.cos(Q1)*np.sin(Q2)*np.sin(Q3))/4000, np.sin(Q1)*((1569*np.sin(Q2 + Q3))/4000 + (17*np.sin(Q2))/40 + np.sin(Q5)*((823*np.cos(Q2 + Q3)*np.sin(Q4))/10000 + (823*np.sin(Q2 + Q3)*np.cos(Q4))/10000) + (591*np.cos(Q2 + Q3)*np.cos(Q4))/6250 - (591*np.sin(Q2 + Q3)*np.sin(Q4))/6250),                         np.sin(Q1)*((591*np.cos(Q2 + Q3 + Q4))/6250 + (1569*np.sin(Q2 + Q3))/4000 + (823*np.sin(Q2 + Q3 + Q4)*np.sin(Q5))/10000),                         np.sin(Q1)*((591*np.cos(Q2 + Q3 + Q4))/6250 + (823*np.sin(Q2 + Q3 + Q4)*np.sin(Q5))/10000), (823*np.cos(Q1)*np.sin(Q5))/10000 - (823*np.cos(Q2)*np.cos(Q3)*np.cos(Q4)*np.cos(Q5)*np.sin(Q1))/10000 + (823*np.cos(Q2)*np.cos(Q5)*np.sin(Q1)*np.sin(Q3)*np.sin(Q4))/10000 + (823*np.cos(Q3)*np.cos(Q5)*np.sin(Q1)*np.sin(Q2)*np.sin(Q4))/10000 + (823*np.cos(Q4)*np.cos(Q5)*np.sin(Q1)*np.sin(Q2)*np.sin(Q3))/10000,                      0],[                                                                                                                                                                                                                                                                                            0,                                                      (591*np.sin(Q2 + Q3 + Q4))/6250 - (823*np.sin(Q2 + Q3 + Q4 + Q5))/20000 - (1569*np.cos(Q2 + Q3))/4000 - (17*np.cos(Q2))/40 + (823*np.sin(Q2 + Q3 + Q4 - Q5))/20000, (591*np.sin(Q2 + Q3 + Q4))/6250 - (823*np.sin(Q2 + Q3 + Q4 + Q5))/20000 - (1569*np.cos(Q2 + Q3))/4000 + (823*np.sin(Q2 + Q3 + Q4 - Q5))/20000, (591*np.sin(Q2 + Q3 + Q4))/6250 - (823*np.sin(Q2 + Q3 + Q4 + Q5))/20000 + (823*np.sin(Q2 + Q3 + Q4 - Q5))/20000,                                                                                                                                                                           - (823*np.sin(Q2 + Q3 + Q4 + Q5))/20000 - (823*np.sin(Q2 + Q3 + Q4 - Q5))/20000,                                                     0],[                                                                                                                                                                                                                                                                                            0,                                                                                                                                                                                                  np.sin(Q1),                                                                                                                           np.sin(Q1),                                                                                                np.sin(Q1),                                                                                                                                                                                                                           np.sin(Q2 + Q3 + Q4)*np.cos(Q1),   np.cos(Q5)*np.sin(Q1) - np.cos(Q2 + Q3 + Q4)*np.cos(Q1)*np.sin(Q5)],[                                                                                                                                                                                                                                                                                            0,                                                                                                                                                                                                 -np.cos(Q1),                                                                                                                          -np.cos(Q1),                                                                                               -np.cos(Q1),                                                                                                                                                                                                                           np.sin(Q2 + Q3 + Q4)*np.sin(Q1), - np.cos(Q1)*np.cos(Q5) - np.cos(Q2 + Q3 + Q4)*np.sin(Q1)*np.sin(Q5)],[                                                                                                                                                                                                                                                                                            1,                                                                                                                                                                                                        0,                                                                                                                                 0,                                                                                                      0,                                                                                                                                                                                                                                  -np.cos(Q2 + Q3 + Q4),                            -np.sin(Q2 + Q3 + Q4)*np.sin(Q5)]])
            jacobian=np.matrix(jacobian)
            jacobian=np.linalg.pinv(jacobian)
            joint_vel=jacobian*cam_vel_np
            joint_vel=joint_vel
            joint_vel=np.ndarray.transpose(joint_vel)

            
            joint_vel=joint_vel.tolist()
            joint_vel=joint_vel[0]
            joint_vel=tuple(joint_vel)
            print "Joint Velocity: "
            print joint_vel
            

            
            
            #Creating the publisher
            cam_pub=rospy.Publisher('ur5_joint_velocities',floatList,queue_size=10)
            new_List=floatList()
            new_List.data=joint_vel
            # print new_List
            rate = rospy.Rate(100) #20 Hz
            cam_pub.publish(new_List) 
            rate.sleep()
            # print pix_vel

def vs_ur5():
    global joint_positions
    global coordinates
    rospy.init_node('vs_ur5')
    rospy.Subscriber('/3_point_features', floatList,coordinate_callback)
    rospy.Subscriber("/joint_states", JointState, position_callback)
    
    while not rospy.is_shutdown():
        
        joint_velocities(joint_positions,coordinates)


  

    




if __name__ == '__main__':
    try:
        vs_ur5()
    except rospy.ROSInterruptException:
        pass

    
