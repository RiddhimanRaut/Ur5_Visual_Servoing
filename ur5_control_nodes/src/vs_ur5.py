#!/usr/bin/env python
import rospy
from ur5_control_nodes.msg import floatList
from sensor_msgs.msg import JointState
import numpy as np
from math import *
from std_msgs.msg import Header
from control_msgs.msg import *
from trajectory_msgs.msg import *
import jacobian_func
import cv2

joint_positions = None
coordinates = None
# c=0
j_velocity = None

def position_callback(position):
    global joint_positions
    joint_positions = position.position

def coordinate_callback(data):
    global coordinates
    coordinates = data.data

def joint_velocities(pose,points):
    if pose != None:
        if points != None:
            int_matrix = None
            fl = 531.15
            # Put your desired points as an array here, from the output of the object_detect.py node.

            desired_points =[96.40369045377518, -88.57089060440595, 768, 203.65279608360007, 73.20655243833554, 806, 9.640369045377518, 22.59461495010356, 815, 279.57070231594804, -37.055168518169836, 780]

            desired_points = np.asarray(desired_points) #list to numpy array
            
            points = np.asarray(points) #real-time points
            
            error = []
        
            for index in range(len(points)):
                if (index+1)%3 == 0:
                    index += 1
                else:
                    error.append(points[index] - desired_points[index])
            
            avg_x=0
            avg_y=0
            error = np.asarray(error) #1v8 array
            for err_x in error[0:len(error):2]:
                avg_x+=err_x**2
            for err_y in error[1:len(error):2]:
                avg_y+=err_y**2
            mean_error=(avg_x+avg_y)**0.5
            # print "Mean error:", mean_error
            if mean_error<=20:
                error=np.array([0,0,0,0,0,0,0,0])
            

            error = np.matrix(error)
            error = np.ndarray.transpose(error)
            u1 = points[0]
            v1 = points[1]
            z1 = points[2]
            u2 = points[3]
            v2 = points[4]
            z2 = points[5]
            u3 = points[6]
            v3 = points[7]
            z3 = points[8]
            u4 = points[9]
            v4 = points[10]
            z4 = points[11]
            
            int_matrix = np.array([   [ -fl/z1,  0,    u1/z1,     u1*v1/fl,     -(fl*fl+u1*u1)/fl,  v1],
                                      [    0,  -fl/z1, v1/z1, (fl*fl+v1*v1)/fl,      -u1*v1/fl,    -u1],
                                      [ -fl/z2,  0,    u2/z2,     u2*v2/fl,      -(fl*fl+u2*u2)/fl, v2],
                                      [    0,  -fl/z2, v2/z2, (fl*fl+v2*v2)/fl,      -u2*v2/fl,    -u2],
                                      [ -fl/z3,  0,    u3/z3,     u3*v3/fl,      -(fl*fl+u3*u3)/fl, v3],
                                      [    0,  -fl/z3, v3/z3, (fl*fl+v3*v3)/fl,      -u3*v3/fl,    -u3],
                                      [ -fl/z4,  0,    u4/z4,     u4*v4/fl,      -(fl*fl+u4*u4)/fl, v4],
                                      [    0,  -fl/z4, v4/z4, (fl*fl+v4*v4)/fl,      -u4*v4/fl,    -u4]
                                    ])
            
            
            int_matrix = np.matrix(int_matrix) #8v6 matrix
            # inverse    = np.linalg.pinv(int_matrix) #6v8 matrix
            
            L_trans = np.transpose(int_matrix)
            x = (np.linalg.det(int_matrix*L_trans))**0.5
            print "MOM_L : ",x 
            H = L_trans*int_matrix
            Hdiag=np.diag(H)
            Hdiag=np.diag(Hdiag)
            H_sum = H+0.3*Hdiag
            H_sum_inverse = np.linalg.pinv(H_sum)
            
            
            cam_vel_np = -0.001*H_sum_inverse*L_trans*error #Levenberg-Marquadt Controller
                 
            
            temp1 = cam_vel_np.item((0,0))
            temp2 = cam_vel_np.item((1,0))
            temp3 = cam_vel_np.item((2,0))
            temp4 = cam_vel_np.item((3,0))
            temp5 = cam_vel_np.item((4,0))
            temp6 = cam_vel_np.item((5,0))
            

            cam_vel_np[0,0] = temp3
            cam_vel_np[1,0] = temp1
            cam_vel_np[2,0] = temp2
            cam_vel_np[3,0] = temp6
            cam_vel_np[4,0] = temp4
            cam_vel_np[5,0] = temp5

            
            
            Q1=pose[0]
            Q2=pose[1]
            Q3=pose[2]
            Q4=pose[3]
            Q5=pose[4]
            Q6=pose[5]

            jacobian=jacobian_func.calc_jack(Q1,Q2,Q3,Q4,Q5,Q6)

            
            jacobian=np.matrix(jacobian)
            J_trans = np.transpose(jacobian)
            y = (np.linalg.det(jacobian*J_trans))**0.5
            print "MOM_J : ",y
            jacobian=np.linalg.pinv(jacobian)
            joint_vel=jacobian*cam_vel_np
            joint_vel=joint_vel
            joint_vel=np.ndarray.transpose(joint_vel)

            
            joint_vel=joint_vel.tolist()
            joint_vel=joint_vel[0]
           
            joint_vel=tuple(joint_vel)
            
            print "Joint Velocity: "
            print joint_vel
            return joint_vel
            
            
def vs_ur5():
    global joint_positions
    global coordinates
    global j_velocity
    rospy.init_node('vs_ur5')
    rospy.Subscriber('/3_point_features', floatList,coordinate_callback,queue_size=1)
    rospy.Subscriber("/joint_states", JointState, position_callback,queue_size=1)

    #Creating the publisher
    cam_pub=rospy.Publisher('ur5_joint_velocities',floatList,queue_size=10)

    while not rospy.is_shutdown():
        
        j_velocity = joint_velocities(joint_positions,coordinates)
        new_List   = floatList()
        new_List.data = j_velocity    
        cam_pub.publish(new_List) 

if __name__ == '__main__':
    try:
        vs_ur5()
    except rospy.ROSInterruptException:
        pass

    
