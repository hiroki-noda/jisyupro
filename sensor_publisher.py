#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Range
import time
import random
#import RPi.GPIO as GPIO

#raspberry pi

def reading(sensor):
    #GPIO.setwarnings(False)
    #GPIO.setmode(GPIO.BOARD)
    TRIG = 11
    ECHO = 13
    if sensor == 0:
        #GPIO.setup(TRIG,GPIO.OUT)
        #GPIO.setup(ECHO,GPIO.IN)
        #GPIO.output(TRIG, GPIO.LOW)
        time.sleep(0.3)
         
        #GPIO.output(TRIG, True)
        time.sleep(0.00001)
        #GPIO.output(TRIG, False)
 
        #while GPIO.input(ECHO) == 0:
          #signaloff = time.time()
         
        #while GPIO.input(ECHO) == 1:
          #signalon = time.time()
 
        timepassed = signalon - signaloff
        distance = timepassed * 17000
        return distance
        #GPIO.cleanup()
    else:
        print "Incorrect usonic() function varible."

def talker():
    pub = rospy.Publisher('range',Range,queue_size = 10)
    rospy.init_node('range_talker')
    rg = Range()
    while not rospy.is_shutdown():
        str =  random.randint(0,100) #reading(0)
        rg.range = str
        rospy.loginfo(str)
        pub.publish(rg)
        rospy.sleep(5.0)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
