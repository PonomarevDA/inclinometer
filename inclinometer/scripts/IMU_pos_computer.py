#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose
import numpy as np

class QuatToEuler:
    def __init__(self):
        self.subscriber1 = rospy.Subscriber("/imu1/sensordata", Imu, self.Imu1Publish)
        self.subscriber2 = rospy.Subscriber("/imu2/sensordata", Imu, self.Imu2Publish)
        self.publisher1 = rospy.Publisher('/imu1/inclinometer', Pose, queue_size=10)
        self.publisher2 = rospy.Publisher('/imu2/inclinometer', Pose, queue_size=10)
        self.test_publisher = rospy.Publisher('/data_test', Pose, queue_size=10)
        self.pose_msg = Pose()

    def Imu1Publish(self, msg):
        self.pose_msg.position.x = 0
        self.pose_msg.position.y = -1
        self.pose_msg.position.z = 0
        self.pose_msg.orientation.w = msg.orientation.w
        self.pose_msg.orientation.x = msg.orientation.x
        self.pose_msg.orientation.y = msg.orientation.y
        self.pose_msg.orientation.z = msg.orientation.z
        self.publisher1.publish(self.pose_msg)
        q_r, q_i, q_j, q_k = self.pose_msg.orientation.x, self.pose_msg.orientation.y, self.pose_msg.orientation.z
        # self.pose_msg.position.x = 2*(q_i*q_k +
        # self.pose_msg.position.y = -1
        # self.pose_msg.position.z = 0

    def Imu2Publish(self, msg):
        self.pose_msg.position.x = 0
        self.pose_msg.position.y = 1
        self.pose_msg.position.z = 0
        self.pose_msg.orientation.w = msg.orientation.w
        self.pose_msg.orientation.x = msg.orientation.x
        self.pose_msg.orientation.y = msg.orientation.y
        self.pose_msg.orientation.z = msg.orientation.z
        self.publisher2.publish(self.pose_msg)


if __name__=="__main__":
    rospy.init_node("IMU_to_euler_converter")
    quatToEuler = QuatToEuler()
    rospy.spin()