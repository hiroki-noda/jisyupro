#!/usr/bin/env python
import rospy
from sensor_msgs.msg import ChannelFloat32

def callback(data):
    rospy.loginfo(data)

def listener():
    rospy.init_node('listener',anonymous=True)
    rospy.Subscriber("chatter", ChannelFloat32, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
