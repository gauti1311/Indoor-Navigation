#!/usr/bin/env python
import cv2
import numpy as np
import glob
from func import showImage , _imgpoints , _real_points
from pylibdmtx.pylibdmtx import decode

axis = np.float32([[10,0,0], [0,10,0], [0,0,-10]])    ## 3D_co-ordinates 
(columns, rows) = (,)   #assign no of columns and row of dmtx in one image
(length,gap) = (,)   # original length and gap between two dtmx in millimeter
def main():
    while True:  
        images = glob.glob('*.png') 
        print('no of images available for calibration:{}'.format(len(images)))
        img_points = []
        obj_points = []
        for i in range(len(images)):
            print images[i]
            img = cv2.imread(images[i])             
            showImage(img)
            height, width = img.shape[:2]
	##decode all_dmtx 
	my_data = decode((img.tobytes(), width, height), max_count= columns*rows, shape= , timeout=5000) # shape is int e.g. for 10*10 dmtx shape=0, for 11*11 is 1 and so on.  in img.
	if len(my_data) == columns*rows:
	    _,img_point = _imgpoints(my_data,height-1)       
	    img_points.append(np.array(np.reshape(img_point,(1,80,2)),dtype=np.float32))
	    obj_point = []
	    ## generate obj_points in 3D with z=0
	    obj_point.append(_real_points(rows,columns,length, gap))
	    obj_points.append(obj_point)
	else:
	    print('Not all data_matrix decoded:{}'.format(len(my_data))) 
	obj_points = np.array(obj_points,dtype=np.float32)  
	## Iterative camera_calibration 
        _, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, (width,height),None,None)
        total_error = 0
	## project obj_points using derived camera param
        for i in xrange(len(obj_points)):
            imgpoints2, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], mtx, dist)
            img_points_proj = np.reshape(imgpoints2,(1,80,2))
	    ## calcualte mean_error
            error = cv2.norm(img_points[i],img_points_proj, cv2.NORM_L2)/len(img_points_proj[0])
            print error
            total_error += error
            _img_points = [img_points[i][k] for k in xrange(len(img_points[i]))]
            _imgpoints_proj = [img_points_proj[0][l] for l in xrange(len(img_points_proj[0]))]
            _img = cv2.imread(images[i])          
	   ## project points on every image
            for j in xrange(len(_imgpoints_proj)):
                ctr = (int(_img_points[0][j][0]),int(_img_points[0][j][1]))   
                cv2.circle(_img,ctr,2,(255,0,0),2)   
                ctr2 = (int(_imgpoints_proj[j][0]),int(_imgpoints_proj[j][1]))
                cv2.circle(_img,ctr2,2,(0,0,255),2)
            showImage(_img)
        print ('camera_matrix:{}\n Dist_coeff:{}\n error_per corner:{}'.format(mtx ,dist, total_error/len(obj_points)))
        f = open('calibration.txt', 'w' )
        f.write('Camera_matrix :{}\n Dist_coeff :{}\n '.format(mtx,dist))
        f.close
        break        
        
if __name__ == '__main__':
    try:
        main()
    except EnvironmentError:
        pass
