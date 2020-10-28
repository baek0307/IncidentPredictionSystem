import kj_estimate_distance as ed
import rospy
import queue
import car_position_calculator as cp
import threading
import cv2
import numpy as np
from std_msgs.msg import String

q = queue.Queue()
running = True

#cam0: camera capturing hallway
#cam1: camera capturing water purifier


#convert topview position of cam1 to cam0
#cause cam0 axis is our base axis
#input: position based on cam1
#output: position based on cam0
def ConvertCam1PosToCam0Pos(pos):
	return (pos[1]-110, 100-pos[0])

#convert pixel position to topview position based on cam0
#input: information about detected persons by darknet yolo
#output: topview position of detected person based on cam0
def ConvertCam0PixelToGlobalPosForPerson(detections):
	ret = None
	for label, confidence, bbox in detections:
		x, y, w, h = bbox
		conv_x = int(640-x)
		conv_y = int(480-y -(h/2))

		dist_x = int((-0.77 *conv_x) + (0.0371 * conv_y) + 240)
		dist_y = int((0.00908 * conv_x) + (1.76 *conv_y) + 115 )
		ret =  (dist_x, dist_y)
	return ret

#Get car position from current image frame
#input: no input(use current frame which class object already has)
#output: pixel position of car(marked with green color)
def GetCarPosition(frame, roiCutter):
	frameHLS = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
	h, l, s = cv2.split(frameHLS)
	hThreshHold = cv2.inRange(h, 64, 88)
	lThreshHold = cv2.inRange(l, 16, 240)

	carPosImage = cv2.bitwise_and(frameHLS, frameHLS,  mask=hThreshHold)
	carPosImage = cv2.bitwise_and(carPosImage, carPosImage, mask=lThreshHold)

	carPosImage = cv2.cvtColor(carPosImage, cv2.COLOR_HLS2BGR)
	kernel = np.ones((7,7), np.uint8)
	carPosImage = cv2.morphologyEx(carPosImage, cv2.MORPH_OPEN, kernel)
	carPosImage = roiCutter.CutImage(carPosImage)

        
	_,g,_ = cv2.split(carPosImage)
	meanPixel = cp.CalculateMeanPixel(carPosImage)

	return meanPixel

#wait for protect running too fast
def Wait():
	cv2.waitKey(1)

#function for process frames captured by cam0
#input: index of camera capturing hallway
#runs while cam0 is cunnected
def ThreadForCam0(cam0Index):#detect person
	global q, running
	detector = ed.DistanceEstimater(640, 480, cam0Index)
	while detector.capture.isOpened():
		Wait()
		if detector.ReadImage() is True:
			cv2.imshow('cam0', detector.frame)
			carPos = detector.GetCarPosition()
			detector.DetectObjects()
			personPos = ConvertCam0PixelToGlobalPosForPerson(detector.detections)
			detector.DrawBoundingBoxes()
			cv2.imshow('bbox', detector.frame)
			if False:
				carPos = cp.CalculateLocationForCam0(carPos)
				msg = 'car' + ' ' + str(int(carPos[0])) + ' ' + str(int(carPos[1]))
				q.put(msg)
			if personPos is not None:
				msg = 'person' + ' ' + str(int(personPos[0])) + ' ' + str(int(personPos[1]))
				q.put(msg)

#function for process frames captured by cam1
#input: index of camera capturing water purifier
#runs while cam1 is cunnected
def ThreadForCam1(cam1Index):#detect car
	capture = cv2.VideoCapture(cam1Index)
	roiCutter = cp.ROICutter(640, 480, cam1Index)
	global q, running
	while capture.isOpened():
		Wait()
		ret, frame = capture.read()
		if not ret:
			continue
		cv2.imshow('cam1', frame)
		pos = GetCarPosition(frame, roiCutter)
		if pos is not None:
			pos = cp.CalculateLocationForCam1(pos)
			pos = ConvertCam1PosToCam0Pos(pos)
			msg = 'car' + ' ' + str(int(pos[0])) + ' ' + str(int(pos[1]))
			q.put(msg)

#ros function sending data about position of objects to jetbot
def MessageSender():
        #message type:String, topic:chatter
	global q, running
	rospy.init_node('talker',anonymous = True)
	pub = rospy.Publisher('chatter',String,queue_size=100)
	rate = rospy.Rate(100)
	while running:
		while q.empty() is not False:
			msg = q.get()
			string = "%s"%msg
			rospy.loginfo(string)
			pub.publish(String(string))
			rate.sleep()
		Wait()

if __name__=='__main__':
	cam0Index = 1
	cam1Index = 0
	cam0Thread = threading.Thread(target=ThreadForCam0, args=([cam0Index]))

	cam0Thread.start()
	MessageSender()

	cam0Thread.join()

	cv2.destroyAllWindows()
