#!/usr/bin/env python
import cv2
import math
import matplotlib.pyplot as plt
import numpy as np
import sympy as sp
from numpy.linalg import norm 


#---------function to sort with first param--------------
def first(val): 
    return val[0] 

#----------------------function to show image------------------
def showImage(img):                                                   
    cv2.imshow('image', img)
    cv2.waitKey(2000) 

#--------------------function to draw XYZ axis--------------------------
def _draw_axis(img, ctr, imgpts):   
    ctr = (int(ctr[0]),int(ctr[1]))                                                
    img = cv2.line(img,ctr, tuple(imgpts[0].ravel()), (0,0,255), 4)
    img = cv2.line(img,ctr, tuple(imgpts[1].ravel()), (0,255,0), 4)
    img = cv2.line(img,ctr, tuple(imgpts[2].ravel()), (255,0,0), 4)
    return img 


# -------------------function to draw rectangle around dmtx------------
def _rect(img,pt1,pt2,pt3,pt4):                         
    img = cv2.line(img,pt1,pt2,(0,255,255),1)
    img = cv2.line(img,pt2,pt3,(0,255,255),1)
    img = cv2.line(img,pt3,pt4,(0,255,255),1)
    img = cv2.line(img,pt4,pt1,(0,255,255),1)
    return img

#-------------------function to extract info from reference-------------------    
def _basic_info(basic_data):
    basic_info = basic_data[0]
    info_list = basic_info.split(':')
    (pattern_size_X, gap_x) = (float(info_list[2]),float(info_list[8]))
    (columns,rows) = (int(info_list[5]),int(info_list[6]))
    module_size = int(info_list[7])
    if module_size == 10 :
        dmtx_symbol = 0
    elif module_size == 12:
        dmtx_symbol = 1
    mm_per_pxl = (pattern_size_X/columns)/(module_size+gap_x*2)
    length = mm_per_pxl*module_size                           
    gap_mm = mm_per_pxl*gap_x*2     
    return columns,rows,length, gap_mm,dmtx_symbol

#---------------function to generate 3D object points in actual dimensions--------------
def _real_points(rows,columns, length ,gap):    
    l = length 
    w = gap
    thd_points = [] 
    for i in range(0,rows):
        for j in range(0,columns):
	    thd_point = [(j*(l+w),i*(l+w),0),(j*(l+w),l+i*(l+w),0),(l+j*(l+w),l+i*(l+w),0),(l+j*(l+w),i*(l+w),0)]  
	    thd_points =  thd_points + thd_point   
    thd_points = np.array(thd_points , dtype=np.float32)                   
    return thd_points


#----------------------function to calculate corners pixels of dmtx--------------
def _imgpoints(my_data,height1):    
    all_corners = []
    img_points = []
    data_position = []    
    data_tuple = [(my_data[i][0], my_data[i][1]) for i in xrange(len(my_data))]  
    data_tuple.sort(key=first)
    for i in xrange(len(my_data)):
        if len(data_tuple[i][0]) == 6:     
            small_corner = data_tuple[i][1]
            data_position.append(small_corner)  
        else :
           pass
    for i in xrange(len(data_position)): 
        corners = [(data_position[i][2],height1-data_position[i][3]),(data_position[i][0],height1-data_position[i][1]),(data_position[i][4],height1-data_position[i][5]),(data_position[i][6],height1-data_position[i][7])]  # location of four corners  
        all_corners += corners
        img_points.append(corners)
        corners = []       
    return all_corners,img_points

#--------------------img processing incase of lower image quality and env. param --------------------
def _img_processing(img):                   
    blur = cv2.GaussianBlur(img, (3, 3), 0)                    
    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)           
    thresh = cv2.adaptiveThreshold(gray, 200, adaptiveMethod=cv2.ADAPTIVE_THRESH_GAUSSIAN_C, thresholdType= cv2.THRESH_BINARY ,blockSize=41, C=4) 
    return thresh


#--------------- function to create subplot(bar and line)-----------------
def _subplot(title, x_axis,y_axis1,y_axis2,plot1_name,plot2_name):
    fig, axs = plt.subplots(2)
    fig.suptitle(str(title))
    axs[0].bar(x_axis,y_axis1)   
    axs[0].set_title(str(plot1_name))
    axs[1].plot(x_axis,y_axis2)
    axs[1].set_title(str(plot2_name))
    return plt.savefig(title+'.pdf')

# ------------------function to calculate transformation mtx of plane intersection 
def _plane_intersction(Nvecs,Tvecs):   
    Nvec1 = np.array(Nvecs[0])
    Nvec2 = np.array(Nvecs[1])
    angle = 180- math.acos(np.dot(Nvec1,Nvec2)/(np.linalg.norm(Nvec1)*np.linalg.norm(Nvec2)))*180/math.pi
    tvec1 = Tvecs[0]
    tvec2 = Tvecs[1]
    Xvec = np.add(Nvec1, Nvec2)   
    X_norm = np.linalg.norm(Xvec)   ## do normalization
    Xvec = Xvec/X_norm
    Zvec = np.cross(Nvec2,Nvec1)
    Z_norm = np.linalg.norm(Zvec)
    Zvec = Zvec/Z_norm
    Yvec = np.cross(Zvec,Xvec)
    rot_mtx = np.transpose(np.array([Xvec,Yvec,Zvec]))
    rvec ,_ = cv2.Rodrigues(rot_mtx)
    y = (tvec1[1]+tvec2[1])/2                                                                
    sp.init_printing()
    x,z = sp.symbols('x,z')
    arr = np.array(([x],[y],[z]))
    f = np.dot(Nvec1,np.subtract(arr,tvec1))
    g = np.dot(Nvec2,np.subtract(arr,tvec2))
    f = sp.Eq(f[0], 0)
    g = sp.Eq(g[0], 0)
    solution = sp.solve([f,g],x,z)
    #print solution
    vec = []
    for _ , j in solution.iteritems() :
        coord = j 
        vec.append(coord) 
    tvec =  np.reshape(np.array(np.float32([[vec[0],y,vec[1]]])),(3,1))
    return rvec,tvec,rot_mtx ,angle

#-----------------------function to calculate root_mean_sq_error---------------------------
def _error(array1,array2,l,n):                                                
    Error = [np.subtract(array1[i],array2[i]) for i in xrange(len(array2))]
    dist = [(Error[i][j][0])**2 + (Error[i][j][1])**2  for i in xrange(len(Error)) for j in xrange(len(Error[0]))]
    SSE =  np.sum(dist)
    MSE = SSE/(l*n)
    RMSE = math.sqrt(MSE)
    return RMSE, MSE, SSE, math.sqrt(max(dist)), math.sqrt(min(dist))

