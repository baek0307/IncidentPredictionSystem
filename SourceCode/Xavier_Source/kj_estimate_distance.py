from ctypes import *
import random
import cv2
import time
import darknet
import rospy
from std_msgs.msg import String
from threading import Thread, enumerate
from kj_argument_utils import *

import numpy as np
import car_position_calculator as cp
        

#Class for measure topview position of person and car
class DistanceEstimater:
    #class init
    #darknetWidth: input image width(must be fixed)
    #darknetHeight: input image height(must be fixed)
    #camNum: camera index
    def __init__(self, darknetWidth, darknetHeight, camNum):
        self.darknetWidth = darknetWidth
        self.darknetHeight = darknetHeight
        self.capture = cv2.VideoCapture(camNum)
        #self.capture = cv2.VideoCapture('./data/dist_test_02.mp4')
        #self.capture = cv2.VideoCapture('./data/arcar.mp4')
        self.prev_time = time.time()
        self.fps = -1
        self.frame = None
        self.darknetImage = darknet.make_image(self.darknetWidth, self.darknetHeight, 3)
        self.args = parser()
        check_arguments_errors(self.args)
        self.network, self.classNames, self.classColors = darknet.load_network(
            self.args.config_file,
            self.args.data_file,
            self.args.weights,
            batch_size=1
        )
        self.detections = None
        #rospy.init_node('talker',anonymous = False)
        #self.pub = rospy.Publisher('chatter',String,queue_size=10)
        #self.rate = rospy.Rate(100)
        
        self.carPosImage = np.ones((self.darknetHeight, self.darknetWidth), dtype=np.ubyte) * 255
        self.roiCutter = cp.ROICutter(darknetWidth, darknetHeight, camNum)
        
    def __del__(self):
        self.capture.release()
        cv2.destroyAllWindows()
    
    #draw image that current processing
    def DrawFrame(self):
        cv2.imshow('frame', self.frame)

    #don't use(function for debugging)
    def run(self):
        while True:
            if cv2.waitKey(1) == 27 :
                cv2.destroyAllWindows()
                break
            if self.ReadImage() == False:
                break
            #self.GetCarPosition()
            self.DetectObjects()
            self.DrawBoundingBoxes()
            cv2.imshow('aaa', self.frame)
            #self.Talker()
            #fps = 1./(time.time() - self.prev_time)
            #self.prev_time = time.time()
            #print("FPS: {}".format(fps))

    #read image to self.frame from videocapture
    def ReadImage(self):
        ret, self.frame = self.capture.read()
        if not ret:
            return False
        return True

    #detect persons from current frame
    #input: no input(use current frame which class object already has)
    #output: no output(write bounding boxes information to detections(parameter) which class object has)
    def DetectObjects(self):
        if self.frame is None:
            self.detections = None
            return
        frameRGB = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
        frameResized = cv2.resize(frameRGB, (self.darknetWidth, self.darknetHeight),interpolation=cv2.INTER_LINEAR)
	
        darknet.copy_image_from_bytes(self.darknetImage, frameResized.tobytes())
        self.detections = darknet.detect_image(self.network, self.classNames, self.darknetImage, thresh=self.args.thresh)
        darknet.print_detections(self.detections, self.args.ext_output)


    #draw bounding boxes to detected persons
    #input: no input(use current frame which class object already has)
    #output: no output(draw bounding boxes to current frame which class object already has)
    def DrawBoundingBoxes(self):
        if self.frame is not None:
            darknet.draw_boxes(self.detections, self.frame, self.classColors)

    #Get car position from current image frame
    #input: no input(use current frame which class object already has)
    #output: pixel position of car(marked with green color)
    def GetCarPosition(self):
        frameHLS = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HLS)
        h, l, s = cv2.split(frameHLS)
        
        #hls threshhold for get green color
        hThreshHold = cv2.inRange(h, 64, 88)
        lThreshHold = cv2.inRange(l, 16, 240)

        self.carPosImage = cv2.bitwise_and(frameHLS, frameHLS,  mask=hThreshHold)
        self.carPosImage = cv2.bitwise_and(self.carPosImage, self.carPosImage, mask=lThreshHold)

        self.carPosImage = cv2.cvtColor(self.carPosImage, cv2.COLOR_HLS2BGR)

        kernel = np.ones((5,5), np.uint8)
        self.carPosImage = cv2.morphologyEx(self.carPosImage, cv2.MORPH_OPEN, kernel)
        self.carPosImage = self.roiCutter.CutImage(self.carPosImage)

        
        _,g,_ = cv2.split(self.carPosImage)
        meanPixel = cp.CalculateMeanPixel(self.carPosImage)

        return meanPixel



if __name__ == '__main__':
    # Darknet doesn't accept numpy images.
    # Create one with image we reuse for each detect
    # width = darknet.network_width(network)
    # height = darknet.network_height(network)
    distanceEstimater = DistanceEstimater(640,480,0)
    distanceEstimater.run()
    distanceEstimater2 = DistanceEstimater(640,480,1)
    distanceEstimater2.run()

