#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose
import numpy as np

class NodePositionComputer:
    def __init__(self):
        self.subscriber1 = rospy.Subscriber("/imu1/sensordata", Imu, self.Imu1Publish)
        self.subscriber2 = rospy.Subscriber("/imu2/sensordata", Imu, self.Imu2Publish)
        self.publisher1 = rospy.Publisher('/imu1/inclinometer', Pose, queue_size=10)
        self.publisher2 = rospy.Publisher('/imu2/inclinometer', Pose, queue_size=10)
        self.test_publisher = rospy.Publisher('/data_test', Pose, queue_size=10)
        self.imu_pose_msg = [Pose() for i in range(3)]
        self.test_msg = Pose()
        self.setup()

    def setup(self):
        rospy.Timer(rospy.Duration(1.0 / 10.0), self.diffPublisher)


    def Imu1Publish(self, msg):
        self.sendPoseMsg(msg, 0)

    def Imu2Publish(self, msg):
        self.sendPoseMsg(msg, 1)

    def sendPoseMsg(self, msg, idx):
        self.imu_pose_msg[idx].position.x = 0
        self.imu_pose_msg[idx].position.z = 0
        self.imu_pose_msg[idx].orientation.w = msg.orientation.w
        self.imu_pose_msg[idx].orientation.x = msg.orientation.x
        self.imu_pose_msg[idx].orientation.y = msg.orientation.y
        self.imu_pose_msg[idx].orientation.z = msg.orientation.z
        if idx == 0 :
            self.imu_pose_msg[idx].position.y = -0.3
            self.publisher1.publish(self.imu_pose_msg[idx])
        elif idx == 1:
            self.imu_pose_msg[idx].position.y = 0.3
            self.publisher2.publish(self.imu_pose_msg[idx]) 

    def diffPublisher(self, event=None):
        # # first imu
        # imu_q_r = self.imu_pose_msg[0].orientation.w
        # imu_q_i = self.imu_pose_msg[0].orientation.x
        # imu_q_j = self.imu_pose_msg[0].orientation.y
        # imu_q_k = self.imu_pose_msg[0].orientation.z

        # imu1_x = 2*(imu_q_i*imu_q_k + imu_q_j*imu_q_r) 
        # imu1_y = 2*(imu_q_j*imu_q_k - imu_q_i*imu_q_r)
        # imu1_z = 1 - 2*(imu_q_i*imu_q_i + imu_q_j*imu_q_j)

        # # second imu
        # imu_q_r = self.imu_pose_msg[1].orientation.w
        # imu_q_i = self.imu_pose_msg[1].orientation.x
        # imu_q_j = self.imu_pose_msg[1].orientation.y
        # imu_q_k = self.imu_pose_msg[1].orientation.z

        # imu2_x = 2*(imu_q_i*imu_q_k + imu_q_j*imu_q_r) 
        # imu2_y = 2*(imu_q_j*imu_q_k - imu_q_i*imu_q_r)
        # imu2_z = 1 - 2*(imu_q_i*imu_q_i + imu_q_j*imu_q_j)

        # # computing angle between normal vectors
        # numerator = imu1_x*imu2_x + imu1_y*imu2_y + imu1_z*imu2_z
        # denominator = np.sqrt(imu1_x*imu1_x + imu1_y*imu1_y + imu1_z*imu1_z) * np.sqrt(imu2_x*imu2_x + imu2_y*imu2_y + imu2_z*imu2_z)
        # angle = np.arccos( numerator / denominator ) * 180 / np.pi
        # self.test_msg.position.x = angle

        q0 = self.imu_pose_msg[0].orientation.w
        q1 = self.imu_pose_msg[0].orientation.x
        q2 = self.imu_pose_msg[0].orientation.y
        q3 = self.imu_pose_msg[0].orientation.z

        # self.test_msg.position.x = np.atan2(2*(q0*q1 + q2*q3), (1 - 2*(q1*q1 + q2*q2))) * 180 / np.pi
        # self.test_msg.position.y = np.asin(2 * (q0*q2 - q3*q1)) * 180 / np.pi
        # self.test_msg.position.z = np.atan2(2 * (q0*q3 + q2*q1), (1 - 2*(q2*q2 + q3*q3))) * 180 / np.pi
        self.test_publisher.publish(self.test_msg)

if __name__=="__main__":
    rospy.init_node("IMU_to_euler_converter")
    nodePositionComputer = NodePositionComputer()
    rospy.spin()


            
