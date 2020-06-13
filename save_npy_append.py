#!/usr/bin/env python
import rospy
import sys
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math
import time
from getch import getch, pause
from geometry_msgs.msg import Twist

class cvBridgeDemo:
    def __init__(self):
        self.node_name = "cv_bridge_demo"
        rospy.init_node(self.node_name)
        rospy.on_shutdown(self.cleanup)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("input_image", Image, self.image_callback, queue_size=1)
        self.image_pub = rospy.Publisher("output_image", Image, queue_size=1)
        #self.samples = []
        self.load = np.load('samples_key3.npy')
        self.samples = self.load.tolist()
        self.counter = 0
        self.z = 0

    def image_callback(self, ros_image):
        try:
            input_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError, e:
            print e
        output_image = self.process_image(input_image)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(output_image, "8UC3"))

        cv2.imshow(self.node_name, output_image)   
        cv2.waitKey(1)

    def process_image(self, frame):
        resize = cv2.resize(frame, dsize=None, fx=0.4, fy=0.4)
        image = np.array(resize) #to ndarray
        image = image.flatten() #one-dimensional array
        tw = Twist()
        p = rospy.Publisher('twist',Twist,queue_size = 10)
        key = getch()
        tw.linear.x = 1.0
        if (key == 'a'):
            self.z = 1.0
            #image = np.insert(image, 0, 0) #insert 0 to top
            #self.samples.append(image)
        elif (key == 'w'):
            self.z = 0
            #image = np.insert(image, 0, 1) #insert 2 to top
            #self.samples.append(image)
        elif (key == 'd'):
            self.z = -1.0
            #image = np.insert(image, 0, 2) #insert 4 to top
            #self.samples.append(image)
        elif (key == 's'):
            tw.linear.x = 0.0
            pass
        elif (key == 'x'):
            #samples = np.array(self.samples)
            #print(samples.shape)
            #np.save('samples_key3',samples)
            #print("save complete")
            exit(0)
        else:
            pass
        if self.z  < -1:
		self.z = -1
	elif self.z > 1:
		self.z = 1
        tw.angular.z = self.z
        print("x = {},z = {}".format(tw.linear.x,tw.angular.z))
        p.publish(tw)
        time.sleep(0.01)
        return resize
        
    def cleanup(self):
        cv2.destroyAllWindows()

if __name__ == '__main__':
    cvBridgeDemo()
    rospy.spin()
