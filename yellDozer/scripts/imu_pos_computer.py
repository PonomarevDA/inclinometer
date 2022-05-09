"""
Python module collects data from IMU modes and
    compute angular joints position with frequency 100 Hz
"""
#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
import numpy as np


class NodePositionComputer:
    """ This class subscribes to each imu_topic and
        compute joints position 100 times per second.
        Joints position measured in Euler's angle. """
    def __init__(self):
        self.subscriber1 = rospy.Subscriber("/imu1/sensordata", Imu, self.imu1_callback)
        self.subscriber2 = rospy.Subscriber("/imu2/sensordata", Imu, self.imu2_callback)
        self.subscriber3 = rospy.Subscriber("/imu3/sensordata", Imu, self.imu3_callback)
        self.joint_pub = rospy.Publisher('/yelldozer/joint_states', JointState, queue_size=10)
        self.imu_recv_msg = [Imu() for i in range(3)]
        self.joint_state_msg = JointState()
        self.setup()

    def setup(self):
        self.joint_state_msg.name = ["gripper_shaft_joint", "bucket_shaft_joint"]
        rospy.Timer(rospy.Duration(1.0 / 10.0), self.send_joint_pos)

    def imu1_callback(self, msg):
        self.imu_recv_msg[0] = msg

    def imu2_callback(self, msg):
        self.imu_recv_msg[1] = msg

    def imu3_callback(self, msg):
        self.imu_recv_msg[2] = msg

    def get_angle(self, idx):
        ''' Gettin rotaion around X-axis
            Quaternion to Euler angles conversion
            https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles'''

        q_0 = self.imu_recv_msg[idx].orientation.w
        q_1 = self.imu_recv_msg[idx].orientation.x
        q_2 = self.imu_recv_msg[idx].orientation.y
        q_3 = self.imu_recv_msg[idx].orientation.z

        return np.arctan2(2*(q_0*q_1 + q_2*q_3), (1 - 2*(q_1*q_1 + q_2*q_2)))


    def send_joint_pos(self):
        # first imu
        imu1_angle = self.get_angle(0)

        # second imu
        imu2_angle = self.get_angle(1)
        imu2_angle = imu1_angle - imu2_angle

        # third imu
        imu3_angle = self.get_angle(2)
        imu3_angle = imu1_angle - imu3_angle - imu2_angle

        self.joint_state_msg.header.seq += 1
        self.joint_state_msg.header.stamp = rospy.Time.now()
        self.joint_state_msg.position = [-imu2_angle, -imu3_angle]

        self.joint_pub.publish(self.joint_state_msg)



if __name__=="__main__":
    rospy.init_node("IMU_to_euler_converter")
    nodePositionComputer = NodePositionComputer()
    rospy.spin()
