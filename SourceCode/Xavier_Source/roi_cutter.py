import cv2
import numpy as np


#class for cut roi from image
class ROICutter:
	#class init
	#camNum: camera index
	#imageWidth: input image width(must be fixed)
	#imageHeight: input image height(must be fixed)
	def __init__(self, camNum, imageWidth,imageHeight):
		if camNum==0:
			self.roi = np.array([[(220,20),(420,20),(550,480),(0,480)]], dtype=np.int32)
		elif camNum==1:
			self.roi = np.array([[(260,200),(370,200),(450,480),(200,480)]], dtype=np.int32)
		else:
			raise Exception('Not Registered camera! Call 0 or 1 cam for ROI')
		self.mask = np.zeros((imageHeight, imageWidth, 3), np.uint8)
		cv2.fillPoly(self.mask, self.roi, (255,255,255))

	#cut image by ROI
	#input: image for cut
	#output: cut image by ROI
	def CutImage(self, image):
		return cv2.bitwise_and(image, self.mask)

#calculate mean position of non 0 pixels
#input: grayscale image
#output: mean position of non 0 pixels
def CalculateMeanPosition(oneChannelImage):
	nonzeroPositions = np.nonzero(oneChannelImage)
	printnonZeroPositions
	return nonzeroPositions
