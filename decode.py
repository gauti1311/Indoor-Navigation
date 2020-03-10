#!/usr/bin/env python
import rospy
import cv2
import math
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pylibdmtx.pylibdmtx import decode  

camera_mat = np.array([[ [999.461170663331, 0, 642.2582577578172,],[ 0, 996.9611451866272, 474.1471906434548],[0.0000, 0.0000, 1.0000 ]], dtype=float)    ##taken from raspicam_package
dist_coeff = [0.1644274736100891, -0.2717244716038656, -0.002867946281892625, 0, 0]
axis = np.float32([[20,0,0], [0,20,0], [0,0,10]])     ## 3D_co-ordinates to draw xyz axis
l = 20
def callback(data):
    global msg
    global new_img 
    new_img = 'True'
    msg = data
    
def main():
    global msg
    global new_img
    new_img = bool()
    msg = Image()
    rospy.init_node('decode_node', anonymous =True)
    rospy.loginfo('decode node started')
    rospy.Subscriber("/camera/image",Image,callback)
    bridge = CvBridge()
    rate = rospy.Rate(1)
    new_img = False
    while not rospy.is_shutdown():
        if new_img : 
            ## convert opencv-image from msg with cv_bridge
            img = bridge.imgmsg_to_cv2(msg,"bgr8")                 
            height, width = img.shape[:2]                                
            ## decoding DM with decode func of libdmtx with args no of dmtx, shape and timeout in ms                   
            my_data = decode((img.tobytes(), width, height),shape=0, max_count= 1)  
            ## calculate corners of all dmtx 
            info, position = my_data[0], my_data[1]   #because origin pixel (0,0)
            corners = [(position[2],height1-position[3]),(position[0],height1-position[1]),(position[4],height1-position[5]),(position[6],height1-position[7])]  # location of four corners    
            for i in xrange(len(corners)):
                cv2.circle(img,(int(corners[i][0]),int(all_corners[i][1])),2,(250,0,250),2)  
            ## 3D points with length l
            img_points = []
            img_points.append(corners)
            obj_points = [(0,0,0),(0,l,0),(l,l,0),(l,0,0)]
            obj_points = np.array(obj_points , dtype=np.float32)  
            ## perspective n-points to get pose
            _, rvec, tvec = cv2.solvePnP(obj_points, img_points, camera_mat, dist_coeff)     
            ## project axis_coord into 2D  
            imgpts, _ = cv2.projectPoints(axis, rvec, tvec, camera_mat, dist_coeff) 
            ## draw 3D axis on image          
            _draw_axis(img, tuple(corners[0]), imgpts) 
            ## rotation matrix using rodrigues   
            rmtx , _ = cv2.Rodrigues(rvec)
            ## normal vector of plane 
	    print rmtx,tvec
            Nvec= [rmtx[0][2], rmtx[1][2],rmtx[2][2]] 
            showImage(img) 
        else :
            print('No new images')
        rate.sleep()
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
     
   
