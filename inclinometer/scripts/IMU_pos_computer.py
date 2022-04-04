#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import numpy as np

class QuatToEuler:
    def __init__(self, imuTopic, publishTopic):
        self.subscriber = rospy.Subscriber(imuTopic, Imu, self.covertAndPublish)
        self.publisher = rospy.Publisher(publishTopic, Twist, queue_size=10)
        self.twist_msg = Twist()

    def covertAndPublish(self, msg):
        q0 = msg.orientation.w
        q1 = msg.orientation.x
        q2 = msg.orientation.y
        q3 = msg.orientation.z
        self.twist_msg.angular.x = np.arctan2( (2(q0*q1 + q2*q3)) / (1 - 2(q1*q1 + q2*q2)) )
        self.twist_msg.angular.y = np.arcsin( 2(q0*q2 - q3*q1) )
        self.twist_msg.angular.z = np.arctan2( (2(q0*q3 + q1*q2)) / (1 - 2(q2*q2 + q3*q3)) )
        self.publisher.publish(self.twist_msg)


if __name__=="__main__":
    quatToEuler = QuatToEuler("/imu1/inclinometer", "/converted/euler")
    rospy.spin()