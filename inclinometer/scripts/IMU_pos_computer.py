#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
#from geometry_msgs.msg import Pose
import numpy as np

class NodePositionComputer:
    def __init__(self):
        self.subscriber1 = rospy.Subscriber("/imu1/sensordata", Imu, self.Imu1Callback)
        self.subscriber2 = rospy.Subscriber("/imu2/sensordata", Imu, self.Imu2Callback)
        self.subscriber2 = rospy.Subscriber("/imu3/sensordata", Imu, self.Imu3Callback)
        self.publisher1 = rospy.Publisher('/yelldozer/inclinometer', JointState, queue_size=10)
        #self.publisher2 = rospy.Publisher('/imu2/inclinometer', Pose, queue_size=10)
        #self.test_publisher = rospy.Publisher('/data_test', Pose, queue_size=10)
        self.imu_recv_msg = [Imu() for i in range(3)]
        self.setup()

    def setup(self):
        rospy.Timer(rospy.Duration(1.0 / 10.0), self.sendJointPos)

    def Imu1Callback(self, msg):
        self.saveRecvMsg(msg, 0)

    def Imu2Callback(self, msg):
        self.saveRecvMsg(msg, 1)

    def Imu3Callback(self, msg):
        self.saveRecvMsg(msg, 2)

    def saveRecvMsg(self, msg, idx):
        self.imu_recv_msg[idx] = msg
        # self.imu_pose_msg[idx].position.x = 0
        # self.imu_pose_msg[idx].position.z = 0
        # self.imu_pose_msg[idx].orientation.w = msg.orientation.w
        # self.imu_pose_msg[idx].orientation.x = msg.orientation.x
        # self.imu_pose_msg[idx].orientation.y = msg.orientation.y
        # self.imu_pose_msg[idx].orientation.z = msg.orientation.z
        # if idx == 0 :
        #     self.imu_pose_msg[idx].position.y = -0.3
        #     self.publisher1.publish(self.imu_pose_msg[idx])
        # elif idx == 1:
        #     self.imu_pose_msg[idx].position.y = 0.3
        #     self.publisher2.publish(self.imu_pose_msg[idx]) 

    def sendJointPos(self, event=None):
        # first imu
        q0 = self.imu_pose_msg[0].orientation.w
        q1 = self.imu_pose_msg[0].orientation.x
        q2 = self.imu_pose_msg[0].orientation.y
        q3 = self.imu_pose_msg[0].orientation.z

        imu1_angle = np.arctan2(2*(q0*q1 + q2*q3), (1 - 2*(q1*q1 + q2*q2))) * 180 / np.pi

        # second imu
        q0 = self.imu_pose_msg[1].orientation.w
        q1 = self.imu_pose_msg[1].orientation.x
        q2 = self.imu_pose_msg[1].orientation.y
        q3 = self.imu_pose_msg[1].orientation.z

        imu2_angle = np.arctan2(2*(q0*q1 + q2*q3), (1 - 2*(q1*q1 + q2*q2))) * 180 / np.pi

        self.test_msg.position.x = imu1_angle - imu2_angle

        self.test_publisher.publish(self.test_msg)

if __name__=="__main__":
    rospy.init_node("IMU_to_euler_converter")
    nodePositionComputer = NodePositionComputer()
    rospy.spin()


            
