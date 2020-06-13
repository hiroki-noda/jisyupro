#!/usr/bin/env python
import rospy
import sys
import cv2
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class cvBridgeDemo:
    def __init__(self):
        self.node_name = "cv_bridge_demo"
        rospy.init_node(self.node_name)
        rospy.on_shutdown(self.cleanup)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("input_image", Image, self.image_callback, queue_size=1)
        self.image_pub = rospy.Publisher("output_image", Image, queue_size=1)
	self.pre_x = 320
	self.pre_y = 240
	
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
	
	###########distanse######
	a = np.where(frame <= 0,255,frame)
	min = np.min(a)
	#########################
	#white-brack
	img_gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
	kernel = np.ones((5,5),np.float32)/25
	dst = cv2.filter2D(img_gray,-1,kernel)
	#threshold
	ret, img = cv2.threshold(dst,min+12,255,cv2.THRESH_BINARY_INV)
	#erode
	kernel = np.ones((5,5),np.uint8)
	img = cv2.erode(img,kernel,iterations = 4)
	# ORB (Oriented FAST and Rotated BRIEF)
	detector = cv2.ORB_create()
	keypoints = detector.detect(img)
	reps = np.empty((0,2),int)
	for point in keypoints:
		x = point.pt[0]
		y = point.pt[1]
		rep = np.array([[x,y]])
		reps = np.append(reps,rep,axis=0)
	center_x = np.sum(reps[:,0])/len(reps[:,0])
        center_y = np.sum(reps[:,1])/len(reps[:,1])
	self.pre_x = center_x
	self.pre_y = center_y
        cv2.circle(img,(int(round(center_x)),int(round(center_y))),10,(0,0,255),3)
	joy = Joy()
	p = rospy.Publisher('joy',Joy, queue_size = 10)
	#############Speed Control##################
	if (min < 20):
		print("stop!!")
		x = 0.0
	elif (20 <= min and min < 30):
		print("slow down")
		x = 0.03
	elif (30 <= min and min < 40):
		print("keep")
		x = 0.06
	elif (40 <= min and min < 45):
		print("speed up")
		x = 0.1
	else:
		print("Not Found")
		x = 0.0

	#############Steering Control###############	
	if (x != 0.0):
		if (center_x < 120):
			print("left 2")
			z = 0.1
		elif (center_x >= 120 and center_x < 240):
			print("left 1")
			z = 0.05
		elif (center_x >= 240 and center_x < 400):
			print("straight")
			z = 0.0
		elif (center_x >= 400 and center_x < 520):
			print("right 1")
			z = -0.05
		else:
			print("right 2")
			z = -0.1
	else:
		z = 0.0
	joy.axes = [z, x, 0.0, 0.0, 0.0, 0.0]
	p.publish(joy)
	out = cv2.drawKeypoints(img, keypoints, None)
        return out

    def cleanup(self):
        cv2.destroyAllWindows()

if __name__ == '__main__':
    cvBridgeDemo()
    rospy.spin()
