#!/usr/bin/env python
import rospy
import sys
import cv2
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math
import time
import scipy.special as sp
from sensor_msgs.msg import Range

#PC

class cvBridgeDemo:
    def __init__(self,input_layer,hidden_layer,output_layer,learning_late):
        self.node_name = "cv_bridge_demo"
        rospy.init_node(self.node_name)
        rospy.on_shutdown(self.cleanup)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("input_image", Image, self.image_callback, queue_size=1)
        self.image_pub = rospy.Publisher("output_image", Image, queue_size=1)
        self.input = input_layer
        self.hidden = hidden_layer
        self.output = output_layer
        self.eta = learning_late
        self.distance = 100.0
        self.z = 0
        self.pre_x = 32
        self.pre_y = 24
        
    def callback(self,data):
        self.distance = data.range
        #print(self.distance)

    def image_callback(self, ros_image):
        try:
            input_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError, e:
            print e
        output_image = self.process_image(input_image)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(output_image, "8UC1"))

        cv2.imshow(self.node_name, output_image)   
        cv2.waitKey(1)

    def process_image(self, frame):
        tw = Twist()
        p = rospy.Publisher('twist',Twist,queue_size = 10)
        #rospy.init_node('range_listener',anonymous=True)
        rospy.Subscriber("range", Range, self.callback)
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) #gray
        #frame = cv2.resize(frame, dsize=None, fx=0.1, fy=0.1)
        ret,threshold = cv2.threshold(gray,127,255,cv2.THRESH_BINARY_INV)
        threshold = cv2.resize(threshold, dsize=None, fx=0.4, fy=0.4)
        print(threshold.shape)
        kernel = np.ones((5,5),np.uint8)
	threshold = cv2.erode(threshold,kernel,iterations = 4)
        #edges = cv2.Canny(gray,50,150,apertureSize = 3) #find edge
        #edges = cv2.resize(edges, dsize=None, fx=0.1, fy=0.1)
        if self.pre_x < 6:
            imgEdge,contours,hierarchy = cv2.findContours(threshold[0:self.pre_x+6,:], 1, 2)
        elif self.pre_x > 58:
            imgEdge,contours,hierarchy = cv2.findContours(threshold[self.pre_x-6:0,:], 1, 2)
        else:
            imgEdge,contours,hierarchy = cv2.findContours(threshold[self.pre_x-6:self.pre_x+6], 1, 2)

        tw.linear.x = 1.0
        if contours == []:
            print("Not Found")
            pass
        else:
            cnt = contours[0]
            M = cv2.moments(cnt)
            if M['m00']==0:
                print("zero")
                pass
            else:
                cx = float(M['m10']/M['m00'])
                cy = float(M['m01']/M['m00'])
                cv2.circle(threshold,(int(round(cx)),int(round(cy))),5,(0,0,255),3)
                self.z = -(cx-32.0)/32.0*0.45#0.414

        
        print(self.z)
	if self.z < -1:
		self.z = -1
	elif self.z > 1:
		self.z = 1
        if self.distance > 20.0:
            tw.linear.x = 0.95
        else:
            tw.linear.x = 0.0
        ####################
        tw.angular.z = self.z
        print("x = {},z = {}".format(tw.linear.x,tw.angular.z))
        p.publish(tw)
        return threshold
        
    def cleanup(self):
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        cvBridgeDemo(3073,100,5,0.1)
        time.sleep(0.1)
        rospy.spin()
    except KeyboardInterrupt:
        pass
print("done")
