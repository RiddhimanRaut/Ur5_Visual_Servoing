#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from ur5_control_nodes.msg import floatList
from cv_bridge import CvBridge, CvBridgeError
import imutils




bridge    = CvBridge()
rgb_img   = np.zeros((480,640,3),np.uint8)
depth_img = np.zeros((480,640))






 
# initialize OpenCV's special multi-object tracker
trackers = cv2.MultiTracker_create()

def add_lists(l1,l2):
    res_list = [l1[i] + l2[i] for i in range(len(l1))] 
    return res_list 

def div_lists(l1,const):
    res_list = [l1[i]/const for i in range(len(l1))] 
    return res_list 


def rgb_callback(rgb_msg):
    global rgb_img   
    rgb_img = bridge.imgmsg_to_cv2(rgb_msg, "bgr8") 

def d_callback(msg):
    global depth_img
    depth_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    
    

def camshift(frame,dimg):
    global trackers
    fl=531.15
    
    # frame = imutils.resize(frame, width=600)
    (success, boxes) = trackers.update(frame)
    
    list_centres=[]
    for box in boxes:
        (x, y, w, h) = [int(v) for v in box]
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        
        cx = x+w/2
        cy = y+h/2
        if cx<320 and cy<480:
            zo = dimg[cx][cy]
        else:
            zo = dimg[320][240] 
        cv2.circle(frame, (cx,cy), 5, (0,0,0), -1)
        cx = -(cx-320)/(fl/640)
        cy = -(cy-240)/(fl/480)

        list_centres=list_centres+[cx,cy,zo]
        
    
    cv2.imshow("Frame",frame)
    
    key=cv2.waitKey(1) & 0xFF
    if key == ord('s'):
        box = cv2.selectROI("Frame",frame,False,False)
        tracker = cv2.TrackerKCF_create()
        trackers.add(tracker, frame, box)
        return list_centres 

    elif key == ord("q"):
		return 1
    else:
        return list_centres
    

def main():
    global rgb_img
    global depth_img
    
    global trackers
    x=0
    
    rospy.init_node('d_points', anonymous=True)
    rospy.Subscriber('/camera/rgb/image_raw', Image, rgb_callback )
    rospy.Subscriber('/camera/depth/image_raw', Image, d_callback)
    pub = rospy.Publisher('3_point_features', floatList, queue_size = 1)
    features = floatList()
    
    while not rospy.is_shutdown():
        while x!=1:
            x = camshift(rgb_img,depth_img)
            
            if len(x)<1:
                print "empty"
            else:
                x = np.asarray(x)
                c=0
                while c<9:
                    x = add_lists(x, camshift(rgb_img,depth_img))
                    c+=1
                c=c+1
                x = div_lists(x,c)
            features.data = x
            print features.data
            pub.publish(features)

        else:
            cv2.destroyAllWindows()
            break
        
            
        
        

    

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass