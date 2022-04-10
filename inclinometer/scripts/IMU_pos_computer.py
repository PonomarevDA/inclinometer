#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose
import numpy as np

class NodePositionComputer:
    def __init__(self):
        self.subscribers = []
        self.publishers  = []
        self.test_publisher = rospy.Publisher('/data_test', Pose, queue_size=10)
        self.imu_recv_pose_msg = [Pose() for i in range(3)]
        self.imu_send_pose_msg = [Pose() for i in range(3)]
        self.test_msg = Pose()
        self.setup()

    def setup(self):
        rospy.Timer(rospy.Duration(1.0 / 10.0), self.diffPublisher)
        imu_prefics = ["/imu1", "/imu2", "/imu3"]
        callbacks = [self.Imu1Publish, self.Imu2Publish, self.Imu3Publish]
        self.publishers  = [ rospy.Subscriber(imu_prefics[idx] + "/sensordata", Imu, callbacks[idx]) for idx in range(3) ]
        self.subscribers = [ rospy.Publisher(prefics + "/inclinometer", Pose, queue_size=10) for prefics in imu_prefics ]

    def Imu1Publish(self, msg):
        self.sendPoseMsg(msg, 0)

    def Imu2Publish(self, msg):
        self.sendPoseMsg(msg, 1)

    def Imu3Publish(self, msg):
        self.sendPoseMsg(msg, 2)

    def sendPoseMsg(self, msg, idx):
        self.imu_recv_pose_msg[idx].position.x = 0
        self.imu_recv_pose_msg[idx].position.z = 0
        self.imu_recv_pose_msg[idx].orientation.w = msg.orientation.w
        self.imu_recv_pose_msg[idx].orientation.x = msg.orientation.x
        self.imu_recv_pose_msg[idx].orientation.y = msg.orientation.y
        self.imu_recv_pose_msg[idx].orientation.z = msg.orientation.z

        if idx == 0 :
            self.imu_recv_pose_msg[idx].position.y = -0.3
        elif idx == 1:
            self.imu_recv_pose_msg[idx].position.y = 0.3

        # self.subscribers[idx].publish(self.imu_pose_msg[idx])

    def diffPublisher(self, event=None):
        # first imu
        q0 = self.imu_recv_pose_msg[0].orientation.w
        q1 = self.imu_recv_pose_msg[0].orientation.x
        q2 = self.imu_recv_pose_msg[0].orientation.y
        q3 = self.imu_recv_pose_msg[0].orientation.z

        imu1_angle = np.arctan2(2*(q0*q1 + q2*q3), (1 - 2*(q1*q1 + q2*q2))) #* 180 / np.pi

        # second imu
        q0 = self.imu_recv_pose_msg[1].orientation.w
        q1 = self.imu_recv_pose_msg[1].orientation.x
        q2 = self.imu_recv_pose_msg[1].orientation.y
        q3 = self.imu_recv_pose_msg[1].orientation.z

        imu2_angle = np.arctan2(2*(q0*q1 + q2*q3), (1 - 2*(q1*q1 + q2*q2))) #* 180 / np.pi
        if imu2_angle > 0:
            imu2_angle = -np.pi - (np.pi - imu2_angle)

        self.test_msg.position.x = imu2_angle - imu1_angle
        self.test_msg.position.y = imu1_angle
        self.test_msg.position.z = imu2_angle

        self.test_publisher.publish(self.test_msg)

        imu2_angle = imu1_angle - imu2_angle
        self.imu_send_pose_msg[1].position.x = 0
        self.imu_send_pose_msg[1].position.y = 0.3 + 0.3*np.cos(imu2_angle)
        self.imu_send_pose_msg[1].position.z = 0.3*np.sin(imu2_angle)

        
        # code part taken from wikipedia 
        # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        yaw, pitch, roll = 0, 0, imu2_angle + np.pi / 2
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5);
        cp = np.cos(pitch * 0.5);
        sp = np.sin(pitch * 0.5);
        cr = np.cos(roll * 0.5);
        sr = np.sin(roll * 0.5);
        self.imu_send_pose_msg[1].orientation.w = cr * cp * cy + sr * sp * sy
        self.imu_send_pose_msg[1].orientation.x = sr * cp * cy - cr * sp * sy
        self.imu_send_pose_msg[1].orientation.y = cr * sp * cy + sr * cp * sy
        self.imu_send_pose_msg[1].orientation.z = cr * cp * sy - sr * sp * cy


        self.subscribers[0].publish(self.imu_recv_pose_msg[0])
        self.subscribers[1].publish(self.imu_send_pose_msg[1])
        

if __name__=="__main__":
    rospy.init_node("IMU_to_euler_converter")
    nodePositionComputer = NodePositionComputer()
    while(True):
        rospy.sleep(1.0)
    #rospy.spin()


            
