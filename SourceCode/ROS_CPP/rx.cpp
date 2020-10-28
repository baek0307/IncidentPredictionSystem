#include "ros/ros.h"
#include "std_msgs/String.h"

void senderCallback(const std_msgs:;String::ConstPtr& msg)
{
	ROS_INFO("Receive Data: %s", msg->data.c_str());
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rx");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("sender", 1000, senderCallback);
	ros ::spin();
	return 0;
}
