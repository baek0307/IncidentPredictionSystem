from jetbot import Robot
import time
import traitlets
from pynput.keyboard import Key, Listener

robot = Robot()
status = 0
def on_press(key):
    print('\n***Robot Ready***')
    
    if key == Key.up:
        print('***Run Forward***')
        robot.forward(0.6)
        
    elif key == Key.down:
        print('***Run Stop***')
        robot.stop()
        
    elif key == Key.left:
        print('***Run Left***')
        robot.left(0.4)
        
    elif key == Key.right:
        print('***Run Right***')
        robot.right(0.4)
        
    elif key == Key.space:
        print('***Run Backward***')
        robot.backward(0.4)
        
    else :
        robot.stop() 
        
    print(key)
    #keyboardInput.stop()
def RunRobot():
    if status != 2:
        with Listener(on_press) as keyboardInput:
            print('\n***Robot Standby***')
            keyboardInput.join()
    
if __name__=='__main__':   
    RunRobot()