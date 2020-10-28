import rospy
from std_msgs.msg import String
from kf import *

import cv2, numpy as np
import copy
from math import erf

from jetbot import Robot
import time
import traitlets

#a : object frist location, b : car frist location
a = KjKalman([-30,140], dt=0.1)
b = KjKalman([100,150], dt=0.1)

x = 0
y = 0

#crashTime = 100
#crashProbability = 0
robot = Robot()

def callback(msg):
        #out data.py
    nameClass, x, y = msg.data.split(' ')       
    print(x,y,'object')
        #result
    a.PredictAndUpdate()
    CorrectedPos = a.UpdateWithSensorInput([x,y])
    b.PredictAndUpdate()
    #CorrectedPos = b.UpdateWithSensorInput(bPos[i])
    crashTime, crashProbability = PredictCrash(a, b, 100, 300, threshHold=0.5, timeSlot=1, maxTime=5)
    #print(crashTime, crashProbability, 'callback 68, 200')
    rosRobot(crashTime, crashProbability)
    
def listener():
        #init node name, anonymous:duplication
    rospy.init_node('listener',anonymous=False)
        #make node, topic:chatter
    rospy.Subscriber("chatter", String, callback)
        #loop
    rospy.spin()

def rosRobot(crashTime, crashProbability):
    
    if crashTime >= 0 and crashProbability > 0.7:
        print(crashTime, crashProbability, 'robot crash')
        time.sleep(0.1)
        robot.forward(0)
    else:
        print(crashTime, crashProbability, 'robot run')
        robot.forward(1)
    
if __name__=='__main__':
    listener()
