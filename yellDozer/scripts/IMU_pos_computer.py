#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
import numpy as np

class NodePositionComputer:
    def __init__(self):
        self.subscriber1 = rospy.Subscriber("/imu1/sensordata", Imu, self.Imu1Callback)
        self.subscriber2 = rospy.Subscriber("/imu2/sensordata", Imu, self.Imu2Callback)
        self.subscriber3 = rospy.Subscriber("/imu3/sensordata", Imu, self.Imu3Callback)
        self.Joint_pub = rospy.Publisher('/yelldozer/joint_states', JointState, queue_size=10)
        self.imu_recv_msg = [Imu() for i in range(3)]
        self.JointState_msg = JointState()
        self.setup()

    def setup(self):
        self.JointState_msg.name = ["gripper_shaft_joint", "bucket_shaft_joint"]
        rospy.Timer(rospy.Duration(1.0 / 10.0), self.sendJointPos)

    def Imu1Callback(self, msg):
        self.imu_recv_msg[0] = msg

    def Imu2Callback(self, msg):
        self.imu_recv_msg[1] = msg

    def Imu3Callback(self, msg):
        self.imu_recv_msg[2] = msg

    def getAngle(self, idx):
        q0 = self.imu_recv_msg[idx].orientation.w
        q1 = self.imu_recv_msg[idx].orientation.x
        q2 = self.imu_recv_msg[idx].orientation.y
        q3 = self.imu_recv_msg[idx].orientation.z
        
        return np.arctan2(2*(q0*q1 + q2*q3), (1 - 2*(q1*q1 + q2*q2)))


    def sendJointPos(self, event=None):
        # first imu
        imu1_angle = self.getAngle(0)

        # second imu
        imu2_angle = self.getAngle(1) 
        imu2_angle = imu1_angle - imu2_angle 

        # third imu
        imu3_angle = self.getAngle(2)
        imu3_angle = imu1_angle - imu3_angle - imu2_angle

        self.JointState_msg.header.seq += 1
        self.JointState_msg.header.stamp = rospy.Time.now()
        self.JointState_msg.position = [-imu2_angle, -imu3_angle]

        self.Joint_pub.publish(self.JointState_msg)

if __name__=="__main__":
    rospy.init_node("IMU_to_euler_converter")
    nodePositionComputer = NodePositionComputer()
    rospy.spin()


            
