#!/usr/bin/env python
import rospy
import RPi.GPIO as GPIO
from geometry_msgs.msg import Twist
import time

#raspberry pi

GPIO.setmode(GPIO.BOARD)
GPIO.setup(29, GPIO.OUT)
GPIO.setup(31, GPIO.OUT)
GPIO.setup(33, GPIO.OUT)
GPIO.setup(35, GPIO.OUT)
p1 = GPIO.PWM(31, 100)
p2 = GPIO.PWM(29, 100)
q1 = GPIO.PWM(35, 100)
q2 = GPIO.PWM(33, 100)
print("set complete")
time.sleep(2)
p1.start(0)
p2.start(0)
q1.start(0)
q2.start(0)
p1.ChangeDutyCycle(50)
print("p1 start")
time.sleep(2)
q1.ChangeDutyCycle(50)
print("q1 start")
time.sleep(2)

class twistListener:
    def __init__(self):
        self.pre_x = 0
        self.pre_z = 0
        self.node_name = "twist_listener"
        rospy.init_node(self.node_name)
        rospy.Subscriber("twist",Twist,self.callback)

    def callback(self,tw):
        x = tw.linear.x #linearx : 0.0~1.0
        z = tw.angular.z #angularz : -1.0~1.0
        if(z == self.pre_z and x == self.pre_x):
            time.sleep(0.01)
            pass
        else:
            if x > 0:
                if abs(z - self.pre_z) > 0.05 or abs(x - self.pre_x) > 0.05:
                    while (abs(z - self.pre_z) > 0.05) or abs(x - self.pre_x) > 0.05:
                        print("z - pre_z  = {}".format(abs(z - self.pre_z)))
                        print("x - pre_x  = {}".format(abs(x - self.pre_x)))
                        z = (z + self.pre_z)/2
                        x = (x + self.pre_x)/2
                duty = 50 * x
                if (z == 0.0):
                    print("straight")
                    print("p1={},p2={},q1={},q2={}".format(duty,0,duty,0))
                    p1.ChangeDutyCycle(duty)
                    p2.ChangeDutyCycle(0)
                    q1.ChangeDutyCycle(duty)
                    q2.ChangeDutyCycle(0)
                elif (0.0 < z and z <= 1.0):
                    print("left1")
                    print("p1={},p2={},q1={},q2={}".format(duty,0,duty*(1-z),0))
                    p1.ChangeDutyCycle(duty)
                    p2.ChangeDutyCycle(0)
                    q1.ChangeDutyCycle(duty * (1 - z))
                    q2.ChangeDutyCycle(0)
                elif (-1.0 <= z and z < 0.0):
                    print("right1")
                    print("p1={},p2={},q1={},q2={}".format(duty*(1+z),0,duty,0))
                    p1.ChangeDutyCycle(duty * (1 + z))
                    p2.ChangeDutyCycle(0)
                    q1.ChangeDutyCycle(duty)
                    q2.ChangeDutyCycle(0)
                else:
                    pass
            else:
                pass
            time.sleep(0.01)
            self.pre_x = x
            self.pre_z = z
        
if __name__ == '__main__':
    try:
        twistListener()
        rospy.spin()
    except KeyboardInterrupt:
        pass

p1.stop()
p2.stop()
q1.stop()
p2.stop()
GPIO.cleanup(29)
GPIO.cleanup(31)
GPIO.cleanup(33)
GPIO.cleanup(35)
print("done")
