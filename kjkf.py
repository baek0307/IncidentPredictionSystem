import cv2, numpy as np

#internal function / Do Not Call
def XyToXyTNumpy(arr):
    ret = np.array([[np.float32(arr[0])], [np.float32(arr[1])]])
    return ret

class KjKalman():
    def __init__(self, pos, dt=0.1, r=10, q=1):#, pos):#=np.array([[0.],[0.]])):
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

    #State Matrix after time sec / return [x, y, vx, vy]
    def StateAfterNSec(self, time):
        currentTime = float(0)
        x = self.kf.statePost
        #p = self.kf.errorCovPost        
        while currentTime < time:
            x = np.matmul(self.kf.transitionMatrix, x)
            #p = np.matmul(self.kf.transitionMatrix, p)
            #p = np.matmul(p, self.kf.transitionMatrix.transpose()) + self.kf.processNoiseCov
            currentTime += self.dt
        return x#,p      

    #Predict next state and update kf.statepre, kf.statepost / return predicted state
    def PredictAndUpdate(self):
        return self.kf.predict()

    #Update kf.statepost with 
    def UpdateWithSensorInput(self, pos):
        return self.kf.correct(XyToXyTNumpy(pos))

def CrashAfterNSec(a, b, d, timeslot=1, maxTime=5):
    currentTime = 0.
    while currentTime < maxTime:
        #aPos, aCov = a.PositionAfterNSec(currentTime)
        #bPos, bCov = b.PositionAfterNSec(currentTime)
        aPos = a.StateAfterNSec(currentTime)
        bPos = b.StateAfterNSec(currentTime)
        
        xDistance = aPos[0] - bPos[0]
        yDistance = aPos[1] - bPos[1]
        DistanceSquare = xDistance * xDistance + yDistance * yDistance
        #xSigmaSquare = (aCov[0][0] * bCov[0][0]) / (aCov[0][0] + bCov[0][0])
        #ySigmaSquare = (aCov[1][1] * bCov[1][1]) / (aCov[1][1] + bCov[1][1])
        if (d * d) > DistanceSquare:
            #xSigmaSquare = (aCov[0][0] * bCov[0][0]) / (aCov[0][0] + bCov[0][0])
            #ySigmaSquare = (aCov[1][1] * bCov[1][1]) / (aCov[1][1] + bCov[1][1])
            #xColisionProbability = 1 / np.sqrt(xSigmaSquare * 2 * np.pi)
            #yColisionProbability = 1 / np.sqrt(ySigmaSquare * 2 * np.pi)
            return currentTime, #xColisionProbability * yColisionProbability
        currentTime += timeslot
    return -1#, -1


#all inputs are float
def tmpFunc(accuracy, distance, meanDifference, sigma):
    maxSum = 0
    maxX = (distance - meanDifference) / sigma
    maxTmp = maxX

    minSum = 0
    minX = ((-1.)*distance - meanDifference) / sigma
    minTmp = minX

    print("\n{} ~ {}".format(minX, maxX))
    for i in range(1, int(accuracy)+1):
        maxSum += maxTmp
        maxTmp = (-1.) * maxTmp * maxX * maxX / i / 2. / (2.*float(i)+1.) * (2.*float(i)-1.)
        minSum += minTmp
        minTmp = (-1.) * minTmp * minX * minX / i / 2. / (2.*float(i)+1.) * (2.*float(i)-1.)

    return (maxSum-minSum) / np.sqrt(2. * np.pi)
        

#Calculate Probavility of Collision(object1 and object2)
#return predicted collision time from current and probability, if not crash return -1, -1
def CollisionProbability(object1,object2,distanceX, distanceY,timeslot=1, maxTime=5):
    xCollisionProbability = tmpFunc(4, 1, object1.kf.statePost[0] - object2.kf.statePost[0],
				np.sqrt(object1.kf.errorCovPost[0][0] + object2.kf.errorCovPost[0][0]))
    yCollisionProbability = tmpFunc(4, 1, object1.kf.statePost[1] - object2.kf.statePost[1],
				np.sqrt(object1.kf.errorCovPost[1][1] + object2.kf.errorCovPost[1][1]))
    CollisionProbability = xCollisionProbability * yCollisionProbability
    print(CollisionProbability)
    return CollisionProbability

#Excuted when $python3 kjkf.py
if __name__ == '__main__':
    WINDOW_SIZE = 512
    NOISE_SCALE = 10
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

        #collisionTime, collisionProbability = 1, CollisionProbability(a,b,10, 1, 0.1)
        collisionTime = CrashAfterNSec(a,b,10,timeslot=0.1, maxTime=5)
        text = "Crash after: {}".format(collisionTime)
        print(text)
        textPos = WINDOW_SIZE//3
        cv2.rectangle(image, (textPos,0), (WINDOW_SIZE-1, 15), WHITE, -1)
        cv2.putText(image,text,(textPos,15),cv2.FONT_HERSHEY_COMPLEX,0.5,BLACK,1)

        cv2.imshow("KjKfEx",image)
        cv2.waitKey(3000//NUM_OF_POINTS)
            
    while True:
        k = cv2.waitKey(1000) &0xFF
        if k == 27: break
