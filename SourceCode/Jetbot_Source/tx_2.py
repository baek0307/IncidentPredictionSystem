import rospy
from std_msgs.msg import String
import testcnt

def talker():
        #message type:String, topic:chatter
    pub = rospy.Publisher('chatter',String,queue_size=10)
        #init node name, anonymous:duplication
    rospy.init_node('talker',anonymous = False)
        #loop rate
    rate = rospy.Rate(10)
        #input data init
    testNumber = 0
    while not rospy.is_shutdown():
            #input data.py
        testNumber = testcnt.cntup(testNumber)
            #Save data to string
        str = "%s"%testNumber
            #print terminal
        rospy.loginfo(str)
            #topic to register 
        pub.publish(String(str))
            #delay
        rate.sleep()
    
if __name__=='__main__':
    try:
        talker()
    except rospy.ROSinterruptException:
        pass