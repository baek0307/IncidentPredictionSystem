####################################################################################
#
#	모노카메라를 통한 거리검출(예측)
#
#	작성자 : 백장현 
#	수정일 : 2020/09/22
#
#	find T for estimate real location using pixel location
#
#	Pixel location (in screen) : (Px,Py)
#	Real location (in real world) : (Rx,Ry) 
#	we assume camera's location as (0.0) in real world
# 
#	|a,b,c|     |Px|     |Rx|
#	|d,e,f|  X  |py|  => |Ry|
#	|0,0,1|     |1 |     |1 |
#
#	here is the code to find matrix T ( T = [[a,b,c],[d,e,f],[0,0,1]] )
#	
#
#		
####################################################################################

from __future__ import print_function
from ctypes import *
import math
import random
import os
import numpy as np




try:
	import cv2
	from ar_markers import detect_markers
except ImportError:
	raise Exception('Error: OpenCv is not installed')



# find mark's location pixel 
def find_mat_marker():
    capture = cv2.VideoCapture(1)
    # capture = cv2.VideoCapture('markcheck4.mp4')
    p1x,p2x,p3x,p4x,p1y,p2y,p3y,p4y =0,0,0,0,0,0,0,0

    if capture.isOpened():
        frame_captured, frame = capture.read()
    else:
        frame_captured = False
    while frame_captured:
        markers = detect_markers(frame)
        for marker in markers:
            # print("ID {}'s position is {}".format(marker.id, marker.center))
            if marker.id ==185 :
                p1x,p1y = marker.center[0] , marker.center[1]
                print("p1(x,y) : " ,p1x,p1y)
            elif marker.id ==992:
                p2x,p2y = marker.center[0] , marker.center[1]
                print("p2(x,y) : " ,p2x,p2y)
            elif marker.id ==2808:
                p3x,p3y = marker.center[0] , marker.center[1]
                print("p3(x,y) : " ,p3x,p3y)
            elif marker.id ==3540:
                p4x,p4y = marker.center[0] , marker.center[1]
                print("p4(x,y) : " ,p4x,p4y)

        cv2.imshow("Test Frame",frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        frame_captured, frame = capture.read()
    capture.release()
    cv2.destroyAllWindows()

    return p1x,p2x,p3x,p4x,p1y,p2y,p3y,p4y

   



   

            

# need to find pixel(x,y) for calculate
# def check_point():
# direct input metod
def input_dataset():


    ## we can get this data from "find_mat_marker"
    # p1x,p1y = map(int,input('Input P1 x , P1 y \n :').split())
    # p2x,p2y = map(int,input('Input P2 x , P2 y \n :').split())
    # p3x,p3y = map(int,input('Input P3 x , P3 y \n :').split())
    # p4x,p4y = map(int,input('Input P4 x , P4 y \n :').split())

    p1x,p2x,p3x,p4x,p1y,p2y,p3y,p4y = find_mat_marker()

    r1x,r1y = map(int,input('Input R1 x , R1 y \n :').split())
    r2x,r2y = map(int,input('Input R2 x , R2 y \n :').split())
    r3x,r3y = map(int,input('Input R3 x , R3 y \n :').split())
    r4x,r4y = map(int,input('Input R4 x , R4 y \n :').split())


    # pixel_x = np.array([p1x,p2x,p3x,p4x])
    # pixel_y = np.array([p1y,p2y,p3y,p4y])
    # real_x = np.array([r1x,r2x,r3x,r4x])
    # real_y = np.array([r1y,r2y,r3y,r4y])

    # print(pixel_x , pixel_y ,real_x ,real_y)

    result = np.array([[p1x,p2x,p3x,p4x],[p1y,p2y,p3y,p4y],[r1x,r2x,r3x,r4x],[r1y,r2y,r3y,r4y]])
    return result




    
def make_T(arr):
    # Point1,2,3 pixel map
    p_1 = np.array([ [arr[0][0],arr[0][1],arr[0][2]]
                    ,[arr[1][0],arr[1][1],arr[1][2]] 
                    ,[1,1,1]])
    # Point1,2,4 pixel map                
    p_2 = np.array([ [arr[0][0],arr[0][1],arr[0][3]]
                    ,[arr[1][0],arr[1][1],arr[1][3]] 
                    ,[1,1,1]])
    # Point1,3,4 pixel map
    p_3 = np.array([ [arr[0][0],arr[0][2],arr[0][3]]
                    ,[arr[1][0],arr[1][2],arr[1][3]] 
                    ,[1,1,1]])
    # Point2,3,4 pixel map                   
    p_4 = np.array([ [arr[0][1],arr[0][2],arr[0][3]]
                    ,[arr[1][1],arr[1][2],arr[1][3]] 
                    ,[1,1,1]])

    # Point1,2,3 Real map _x
    rx_1 = np.array([arr[2][0],arr[2][1],arr[2][2]])
     # Point1,2,4 Real map _x
    rx_2 = np.array([arr[2][0],arr[2][1],arr[2][3]])
     # Point1,3,4 Real map _x
    rx_3 = np.array([arr[2][0],arr[2][2],arr[2][3]])
     # Point2,3,4 Real map _x
    rx_4 = np.array([arr[2][1],arr[2][2],arr[2][3]])
    # Point1,2,3 Real map _y
    ry_1 = np.array([arr[3][0],arr[3][1],arr[3][2]])
     # Point1,2,4 Real map _y
    ry_2 = np.array([arr[3][0],arr[3][1],arr[3][3]])
     # Point1,3,4 Real map _y
    ry_3 = np.array([arr[3][0],arr[3][2],arr[3][3]])
     # Point2,3,4 Real map _y
    ry_4 = np.array([arr[3][1],arr[3][2],arr[3][3]])

    # calc reverse T
    reverse1 = np.linalg.inv(p_1)
    reverse2 = np.linalg.inv(p_2)
    reverse3 = np.linalg.inv(p_3)
    reverse4 = np.linalg.inv(p_4)
    # calc parameter a,b,c,d,e,f of Matrix T
    result_abc1 = np.dot(rx_1,reverse1)
    result_def1 = np.dot(ry_1,reverse1)
    result_abc2 = np.dot(rx_2,reverse2)
    result_def2 = np.dot(ry_2,reverse2)
    result_abc3 = np.dot(rx_3,reverse3)
    result_def3 = np.dot(ry_3,reverse3)
    result_abc4 = np.dot(rx_4,reverse4)
    result_def4 = np.dot(ry_4,reverse4)
    result_abc = (result_abc1 + result_abc2 + result_abc3 + result_abc4)/4
    result_def = (result_def1 + result_def2 + result_def3 + result_def4)/4

    print("[a,b,c] : \n",result_abc)
    print("[d,e,f] : \n", result_def)





def make_polyfit_test():
    # P1(271,289) -> R1(-39,452)
    # P2(374,318) -> R2(39,400)
    # P3(320,318) -> R3(0,400)
    # p = np.array([271,289],[374,318],[320,318])
    # r = np.array([])

    # value1 => ([[P1x,P2x,P3x],[P1y,P2y,P3y],[1,1,1]])
    # value2 => ([R1x,R2x,R3x])
    # value3 => ([R1y,R2y,R3y])
    value1 = np.array([[369,272,266],[191,192,162],[1,1,1]])
    value2 = np.array([-39,39,39])
    value3 = np.array([452,452,400])
    # Idont know what to say ... yuk haeng ryual 
    reverse1 = np.linalg.inv(value1)

    result_abc = np.dot(value2 , reverse1)
    result_def = np.dot(value3 , reverse1)
    
    print("[a,b,c] : \n",result_abc)
    print("[d,e,f] : \n", result_def)

   
# make_polyfit_test()
# find_mat_marker()

array = input_dataset()
make_T(array)