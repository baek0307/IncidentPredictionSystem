import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter',String,queue_size=10)
    rospy.init_node('talker',anonymous = True)
    rate = rospy.Rate(10)
    testNumber = 0
    while not rospy.is_shutdown():
        # str = "hello world %s"%rospy.get_time()
        testNumber += 1
        str = "hello world %s"%testNumber
        rospy.loginfo(str)
        pub.publish(String(str))
        rate.sleep()
    
if __name__=='__main__':
    try:
        talker()
    except rospy.ROSinterruptException:
        pass