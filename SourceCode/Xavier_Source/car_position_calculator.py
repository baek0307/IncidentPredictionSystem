import cv2
import numpy as np


#Class for cut ROI from image to remove noise caused by background
class ROICutter:
	#Class init
	#imageWidth: input image width(must be fixed)
	#imageHeight: input image height(must be fixed)
	#camNum: camera index(odd number -> water purifier direction
	#		     even number -> hallway direction		
	def __init__(self, imageWidth,imageHeight, camNum):
		if camNum%2==1:#water direction
			self.roi = np.array([[(220,20),(420,20),(550,480),(0,480)]], dtype=np.int32)
		elif camNum%2==0:#window direction
			self.roi = np.array([[(260,200),(370,200),(450,480),(200,480)]], dtype=np.int32)
		else:
			raise Exception('Not Registered camera'
					+ str(camNum)
					+'! Call 0 or 1 cam for ROI')
		self.mask = np.zeros((imageHeight, imageWidth, 3), np.uint8)
		cv2.fillPoly(self.mask, self.roi, (255,255,255))
		#cv2.imshow('mask', self.mask)

	#Cut image by ROI
	#input: image to cut
	#output: cut image
	def CutImage(self, image):
		return cv2.bitwise_and(image, self.mask)

#Calculate mean pixel position where pixel value is not 0
#input: one channel image
#output: mean postion of pixel position 
def CalculateMeanPixel(oneChannelImage):
	nonZeroPositions = np.nonzero(oneChannelImage)
	nonZeroHeightLength = len(nonZeroPositions[0])
	nonZeroWidthLength = len(nonZeroPositions[1])
	if nonZeroHeightLength == 0:
		return None
	w = 0
	h = 0
	for i in range(0, nonZeroHeightLength):
		h += nonZeroPositions[0][i]
	for i in range(0, nonZeroWidthLength):
		w += nonZeroPositions[1][i]
	return (float(w)/float(nonZeroWidthLength),
		float(h)/float(nonZeroHeightLength))

#Translate pixel position to topview position(custumized for cam0)
#cam0 captures hallway direction
#input: pixel position
#output: topview position
def CalculateLocationForCam0(position):	
	conv_w = int(640-position[0])
	conv_h = int(480-position[1])

	dist_x = int((-0.77 *conv_w) + (0.0371 * conv_h) + 240)
	dist_y = int((0.00908 * conv_w) + (1.76 *conv_h) + 115 )
	return (dist_x, dist_y)

#Translate pixel position to topview position(custumized for cam0)
#cam1 captures hallway direction
#input: pixel position
#output: topview position
def CalculateLocationForCam1(position):	
	dist_x = int((0.6558 *position[0]) + (0.0187 * position[1]) - 223.71)
	if position[1] < 150:
		dist_y = int((-0.0122 * position[0]) + (-1.8975 *position[1]) + 722.83 )
	else:
		dist_y = int((-0.0122 * position[0]) + (-0.948 *position[1]) + 572.83 )
	return (dist_x, dist_y)
