#Ros
import rospy
from std_msgs.msg import String
from kf import *
#KF
import cv2, numpy as np
import copy
from math import erf
#Jetbot
from jetbot import Robot
import time
import traitlets
#Keyboard
from pynput.keyboard import Key, Listener
#Thread
import threading
from queue import Queue


#a : object frist location, b : car frist location
a = KjKalman([-10,600], dt=0.1)
b = KjKalman([700,100], dt=0.1)

x = 0
y = 0
status = 0

leftVelocity = 0.
rightVelocity = 0.

robot = Robot()
statusQueue = Queue()
#########################################################
mapImage = np.zeros((700,700, 3), dtype=uint8)
fourcc = cv2.cv.CV_FOURCC(*'MP4V')
##########################################################

def CallBack(msg):
        #out data.py
    nameClass, x, y = msg.data.split(' ')
    print('object:',nameClass, x, y)
        #result
    a.PredictAndUpdate()
    if nameClass == 'person':
        CorrectedPos = a.UpdateWithSensorInput([x,y])
    b.PredictAndUpdate()
    if nameClass == 'car':
        CorrectedPos = b.UpdateWithSensorInput([x,y])
    crashTime, crashProbability = PredictCrash(a, b, 350, 100, threshHold=0.5, timeSlot=1, maxTime=5)
    RosRobot(crashTime, crashProbability)

###############################################################
    global mapImage
    aPos = (int(a.statePost[0]), int(a.statePost[1]))
    cv2.circle(mapImage, aPos, 3, (128,128,255), -1)
    bPos = (int(a.statePost[0]), int(a.statePost[1]))
    cv2.circle(mapImage, bPos, 3, (128,255,128), -1)
    
###############################################################
def RxListener():
    print('\n***ROS Ready***')
        #init node name, anonymous:duplication
    rospy.init_node('listener',anonymous=False)
        #make node, topic:chatter
    rospy.Subscriber("chatter", String, CallBack)
        #loop
    rospy.spin()

def RosRobot(crashTime, crashProbability):
    global leftVelocity
    global rightVelocity
    if crashTime >= 0 and crashProbability > 0.7:
        status = 2
        print('crashTime:', crashTime, 'crashProbability:', crashProbability, 'robot crash', status)
        robot.left_motor.value = 0.
        robot.right_motor.value = 0.
    else:
        status = 1
        print('crashTime:', crashTime, 'crashProbability:', crashProbability, 'robot run', status)
    statusQueue.put(status)
    
def ContorlRobot(key):
    global leftVelocity
    global rightVelocity
    print('\n***Robot Ready***')
        #contorl robot by keyboard
    robot.left_motor.value = leftVelocity
    robot.right_motor.value = rightVelocity
    if key == Key.up:
        print('***Run Forward***')
        leftVelocity = 0.35
        rightVelocity = 0.34
    elif key == Key.down:
        print('***Run Stop***')
        leftVelocity = 0.
        rightVelocity = 0.    
    elif key == Key.left:
        print('***Run Left***')
        rightVelocity = 0.34
        if leftVelocity < 0.:
            leftVelocity = 0.
        else :
            leftVelocity -= 0.05          
    elif key == Key.right:
        leftVelocity = 0.35
        if rightVelocity < 0.:
            rightVelocity = 0.
        else :
            rightVelocity -= 0.05
        print('***Run Right***')
      
    elif key == Key.space:
        print('***Run Backward***')
        leftVelocity = -0.35
        rightVelocity = -0.34     
    else :
        robot.stop() 
     
    print(key)
    
def RunRobot(status, statusQueue):
    if statusQueue.empty() is False:
    	status = statusQueue.get()
        
    if status != 2:
        with Listener(ContorlRobot) as keyboardInput:
            print('\n***Robot Standby***')
            keyboardInput.join()

def OpenThread():
    RunRobotOpenThread = threading.Thread(target=RunRobot, args=(status, statusQueue))
    RunRobotOpenThread.start()

    RxListener()
    
if __name__=='__main__':
    OpenThread()
