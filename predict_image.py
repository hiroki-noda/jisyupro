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
    def __init__(self,input_layer,hidden_layer1,hidden_layer2,output_layer,learning_rate):
        self.node_name = "cv_bridge_demo"
        rospy.init_node(self.node_name)
        rospy.on_shutdown(self.cleanup)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("input_image", Image, self.image_callback, queue_size=1)
        self.image_pub = rospy.Publisher("output_image", Image, queue_size=1)
        self.input = input_layer
        self.hidden1 = hidden_layer1
        self.hidden2 = hidden_layer2
        self.output = output_layer
        self.eta = learning_rate
        self.w_ih = np.load('w_ih.npy')
        self.w_hh = np.load('w_hh.npy')
        self.w_ho = np.load('w_ho.npy')
        self.distance = 100.0

    def predict(self,input_data):
        x_input = input_data
        u_hidden1 = np.dot(np.array(self.w_ih),x_input)
        x_hidden1 = sp.expit(u_hidden1)
        u_hidden2 = np.dot(np.array(self.w_hh),x_hidden1)
        x_hidden2 = sp.expit(u_hidden2)
        u_output = np.dot(np.array(self.w_ho),x_hidden2)
        x_output = sp.expit(u_output)
        print(x_output)
        return x_output

    def callback(self,data):
        self.distance = data.range
        #print(self.distance)

    def image_callback(self, ros_image):
        try:
            input_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError, e:
            print e
        output_image = self.process_image(input_image)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(output_image, "8UC3"))

        cv2.imshow(self.node_name, output_image)   
        cv2.waitKey(1)

    def zscore(self,x):
        xmean = x.mean(keepdims=True)
        xstd = np.std(x,keepdims=True)
        zscore = (x-xmean)/xstd
        return zscore

    def process_image(self, frame):
        resize = cv2.resize(frame, dsize=None, fx=0.4, fy=0.4)
        #gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) #gray
        #edges = cv2.Canny(gray,50,150,apertureSize = 3) #find edge
        #edges = cv2.resize(edges, dsize=None, fx=0.1, fy=0.1)
        test_image = np.array(resize) #to ndarray
        test_image = test_image.flatten() #one-dimensional array
        test_image = test_image.astype(np.float32)
        test_image = self.zscore(test_image)
        test_image = np.insert(test_image,0,1)
        predicted_label = np.argmax(self.predict(test_image))
        print(predicted_label)
        tw = Twist()
        p = rospy.Publisher('twist',Twist,queue_size = 10)
        rospy.Subscriber("range", Range, self.callback)
        tw.linear.x = 1.0
        if (self.distance > 20.0):
            if (predicted_label == 0):
                tw.angular.z = 1.0
            elif (predicted_label == 1):
                tw.angular.z = 0.4
            elif (predicted_label == 2):
                tw.angular.z = 0.0
            elif (predicted_label == 3):
                tw.angular.z = -0.4
            else:
                tw.angular.z = -1.0
        else:
            ##depend on sensor##
            tw.linear.x = 0.0
        p.publish(tw)
        time.sleep(0.01)
        return frame
        
    def cleanup(self):
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        cvBridgeDemo(9217,1000,100,5,0.08)
        rospy.spin()
    except KeyboardInterrupt:
        pass
print("done")
