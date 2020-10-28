import cv2, numpy as np
import copy
from math import erf

'''
KjKalman is Rapping Class of cv2.KalmanFilter
If you want to update state without sensor input, call KjKalman.PredictAndUpdate()
Or with sensor -> call KjKalman.UpdateWithSensorInput(inputPosition[x,y])
For predict probability of object1 & 2 call PredictCrash(object1, object2, distanceX, distanceY, threshHold=0.5, timeSlot=1, maxTime=5)
object1 & 2 is KjKalman object
distanceX and distanceY is x&y axis distance which is standard for collision prediction
threshHold is minimum probability for deciding wether crash or not
timeSlot is term of prediction time
maxTime is end of prediction time
'''

#internal function / Do Not Call
def XyToXyTNumpy(arr):
    ret = np.array([[np.float32(arr[0])], [np.float32(arr[1])]])
    return ret

class KjKalman():
    def __init__(self, pos, dt=0.1, r=0.0001, q=0.001):#, pos):#=np.array([[0.],[0.]])):
        self.dt = dt
        self.r = r
        self.q = q
        self.kf = cv2.KalmanFilter(4,2)#state domain: (x,y,vx,vy)T / measurement domain: (x,y)
        self.kf.measurementMatrix = np.array([[1,0,0,0],[0,1,0,0]],np.float32)#H
        self.kf.transitionMatrix = np.array([[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]],np.float32)#A
        self.kf.processNoiseCov = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],np.float32) * q#Q: increase covarient with measurement
        self.kf.measurementNoiseCov = np.array([[1,0],[0,1]],np.float32) * r#R: decrease covarient with measurement
        self.kf.errorCovPost = 1. * np.ones((2, 2))#P init
        self.kf.statePre = np.array([[pos[0]], [pos[1]], [0], [1]], dtype=np.float32)
        self.kf.correct(XyToXyTNumpy(pos))   

    #Predict next state and update kf.statepre, kf.statepost / return predicted state
    def PredictAndUpdate(self):
        return self.kf.predict()

    #Update kf.statepost with 
    def UpdateWithSensorInput(self, pos):
        return self.kf.correct(XyToXyTNumpy(pos))

    def copy(self):
        ret = KjKalman([self.kf.statePost[0], self.kf.statePost[1]], self.dt, self.r, self.q)
        ret.kf.errorCovPost = self.kf.errorCovPost
        ret.kf.statePre = self.kf.statePre
        ret.kf.statePost = self.kf.statePost  
        return ret      

#get 2 kjkf Object, x&y collision distance, threshhold of collision probability, timeslot, maxtime
#return predicted crash time from now & probability / if no crash, return -1, -1
def PredictCrash(object1, object2, distanceX, distanceY, threshHold=0.5, timeSlot=1, maxTime=5):
    a = object1.copy()
    b = object2.copy()
    currentTime = 0.
    aCurrentTime = 0.
    bCurrentTime = 0.
    while currentTime <= maxTime:
        while aCurrentTime <= currentTime:
            a.PredictAndUpdate()
            aCurrentTime += a.dt
        while bCurrentTime <= currentTime:
            b.PredictAndUpdate()
            bCurrentTime += b.dt
        collisionProbability = CollisionProbability(a,b,distanceX,distanceY)
        if collisionProbability > threshHold:
            return currentTime, collisionProbability
        currentTime += timeSlot
    return -1, -1


#Calculate Probavility of Collision(object1 and object2)
#return predicted collision time from current and probability, if not crash return -1, -1
def CollisionProbability(object1,object2,distanceX, distanceY):
    xSigmaSquare = object1.kf.errorCovPost[0][0] + object2.kf.errorCovPost[0][0]
    xCollisionProbability = (
				erf((distanceX - object1.kf.statePost[0] + object2.kf.statePost[0]) / np.sqrt(2*xSigmaSquare))
				- erf((-1.*distanceX - object1.kf.statePost[0] + object2.kf.statePost[0]) / np.sqrt(2*xSigmaSquare))
			)/2.
    ySigmaSquare = object1.kf.errorCovPost[1][1] + object2.kf.errorCovPost[1][1]
    yCollisionProbability = (
				erf((distanceY - object1.kf.statePost[1] + object2.kf.statePost[1]) / np.sqrt(2*xSigmaSquare))
				- erf((-1.*distanceY - object1.kf.statePost[1] + object2.kf.statePost[1]) / np.sqrt(2*xSigmaSquare))
			)/2.
    return xCollisionProbability * yCollisionProbability

#Excuted when $python3 kjkf.py
if __name__ == '__main__':
    WINDOW_SIZE = 512
    NOISE_SCALE = 5
    BLUE = (255,0,0)
    WHITE = (255,255,255)
    RED = (0,0,255)
    GREEN = (0,255,0)
    BLACK = (0,0,0)
    NUM_OF_POINTS = 32

    def DrawImage(image):
        cv2.imshow('image', image)
        cv2.waitKey()
        cv2.destroyAllWindows()

    def InRange(num, min, max):
        if(min>num):
            return False
        elif(max<num):
            return False
        else:
            return True

    #Init Window
    cv2.namedWindow("KjKfEx")

    #Make Background
    image = np.zeros((WINDOW_SIZE, WINDOW_SIZE,3), np.uint8)
    cv2.rectangle(image, (0,0), (WINDOW_SIZE-1, WINDOW_SIZE-1), WHITE, -1)

    #Create Input with Noise
    aPos = ((np.random.rand(NUM_OF_POINTS,2) - 0.5) * 2 * NOISE_SCALE)
    bPos = ((np.random.rand(NUM_OF_POINTS,2) - 0.5) * 2 * NOISE_SCALE)
    offset = WINDOW_SIZE/(NUM_OF_POINTS)
    for i in range(0, NUM_OF_POINTS):
        aPos[i][0] += i*offset
        aPos[i][1] += i*offset
        bPos[i][0] += i*offset
        bPos[i][1] += WINDOW_SIZE-i*offset

    #KalmanFilter Init
    a = KjKalman(aPos[0], dt=0.1)
    b = KjKalman(bPos[0], dt=0.1)

    #Line Init
    aLine = [[int(aPos[0][0]), int(aPos[0][1])], [int(aPos[0][0]), int(aPos[0][1])]]
    bLine = [[int(bPos[0][0]), int(bPos[0][1])], [int(bPos[0][0]), int(bPos[0][1])]]

    #Draw Image
    for i in range(1, NUM_OF_POINTS):
        #a
        #Update and predict and draw kf
        a.PredictAndUpdate()
        CorrectedPos = a.UpdateWithSensorInput(aPos[i])
        if InRange(aPos[i][0], 0, WINDOW_SIZE-1) and InRange(aPos[i][1], 0, WINDOW_SIZE-1):
            #Draw Input Points
            cv2.circle(image, (int(aPos[i][1]), int(aPos[i][0])), 3, RED, -1)
            #Draw Filtered Line
            aLine[i%2] = [int(CorrectedPos[0]), int(CorrectedPos[1])]
            cv2.line(image, (aLine[0][0], aLine[0][1]), (aLine[1][0], aLine[1][1]), RED, 1)
        #b
        #Update and predict and draw kf
        b.PredictAndUpdate()
        CorrectedPos = b.UpdateWithSensorInput(bPos[i])
        if InRange(bPos[i][0], 0, WINDOW_SIZE-1) and InRange(bPos[i][1], 0, WINDOW_SIZE-1):
            #Draw Input Points
            cv2.circle(image, (int(bPos[i][0]), int(bPos[i][1])), 3, BLUE, -1)
            #Draw Filtered Line
            bLine[i%2] = [int(CorrectedPos[0]), int(CorrectedPos[1])]
            cv2.line(image, (bLine[0][0], bLine[0][1]), (bLine[1][0], bLine[1][1]), BLUE, 1)

        collisionTime, collisionProbability = PredictCrash(a,b,10,10)
        text = "Crash after: {}, Prob: {}".format(collisionTime, collisionProbability)
        print(text)
        textPos = WINDOW_SIZE//3
        cv2.rectangle(image, (textPos,0), (WINDOW_SIZE-1, 15), WHITE, -1)
        cv2.putText(image,text,(textPos,15),cv2.FONT_HERSHEY_COMPLEX,0.5,BLACK,1)

        cv2.imshow("KjKfEx",image)
        cv2.waitKey(3000//NUM_OF_POINTS)
            
    while True:
        k = cv2.waitKey(1000) &0xFF
        if k == 27: break