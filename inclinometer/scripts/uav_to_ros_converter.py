#!/usr/bin/env python3

import time
import sys
import queue
import dronecan
from dronecan import uavcan
import rospy
from sensor_msgs.msg import Imu


DEV_PATH = "/dev/ttyACM0"
CAN_DEVICE_TYPE = "can-slcan"


class ImuPublisher:
    def __init__(self, topic_name):
        self.ros_msg = Imu()
        self.publisher = rospy.Publisher(topic_name, Imu, queue_size=10)

    def publish_imu(self, uavcan_msg):
        """
        Incoming message structure:
        --------------------------
        timestamp:
            usec: 28128
        orientation_xyzw: [0.9888, 0.1210, -0.0034, 0.0872]
        orientation_covariance: []
        angular_velocity: [-0.0083, 0.0043, 0.0112]
        angular_velocity_covariance: []
        linear_acceleration: [-0.3149, 1.8408, -10.5703]
        linear_acceleration_covariance: []
        --------------------------
        """
        self.ros_msg.header.stamp = rospy.Time.now()

        self.ros_msg.orientation.x = uavcan_msg.orientation_xyzw[0]
        self.ros_msg.orientation.y = uavcan_msg.orientation_xyzw[1]
        self.ros_msg.orientation.z = uavcan_msg.orientation_xyzw[2]
        self.ros_msg.orientation.w = uavcan_msg.orientation_xyzw[3]

        self.ros_msg.angular_velocity.x = uavcan_msg.angular_velocity[0]
        self.ros_msg.angular_velocity.y = uavcan_msg.angular_velocity[1]
        self.ros_msg.angular_velocity.z = uavcan_msg.angular_velocity[2]

        self.ros_msg.linear_acceleration.x = uavcan_msg.linear_acceleration[0]
        self.ros_msg.linear_acceleration.y = uavcan_msg.linear_acceleration[1]
        self.ros_msg.linear_acceleration.z = uavcan_msg.linear_acceleration[2]

        self.publisher.publish(self.ros_msg)


class DroneCanCommunicator:
    """
    Simple wrap on droneCan
    Based on example: https://legacy.uavcan.org/Implementations/Pyuavcan/Tutorials/
    """
    def __init__(self, can_device_type, node_id=42, node_name="uavcan communicator"):
        """
        Simply create a node without starting it.
        param can_device_type - could be 'serial' or 'can-slcan'
        """
        self.subs = []
        self.pubs = {
            81 : ImuPublisher("/imu1/sensordata"),
            82 : ImuPublisher("/imu2/sensordata"),
            83 : ImuPublisher("/imu3/sensordata")
        }
        self.node = None

        if can_device_type == "can-slcan":
            kawrgs = {"can_device_name" : "slcan0",
                      "bustype" : "socketcan",
                      "bitrate" : 1000000}
        else:
            rospy.logerr("UavcanCommunicatorV0: Unsupported device type")
            sys.exit()

        node_info = uavcan.protocol.GetNodeInfo.Response()
        node_info.name = node_name
        node_info.software_version.major = 0
        node_info.software_version.minor = 2
        node_info.hardware_version.unique_id = b'12345'

        self.node = dronecan.make_node(node_id=node_id, node_info=node_info, **kawrgs)
        self.subscribe(uavcan.equipment.ahrs.Solution, self.node_ahrs_msg_callback)

    def __del__(self):
        if self.node is not None:
            self.node.close()

    def subscribe(self, data_type, callback):
        """
        param data_type - https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/
        param callback - any function with single parameter - event
        Example:
        data_type = uavcan.protocol.NodeStatus
        callback = lambda event: print(uavcan.to_yaml(event))
        communicator.subscribe(data_type, callback)
        """
        self.subs.append(self.node.add_handler(data_type, callback))

    def publish(self, data_type):
        """
        param data_type - https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/
        Example:
        fix2 = uavcan.equipment.gnss.Fix2(pdop=10)
        communicator.publish(fix2)
        """
        try:
            self.node.broadcast(data_type)
        except uavcan.driver.common.TxQueueFullError as err:
            rospy.logerr(f"tx uavcan.driver.common.TxQueueFullError {err}")
        except queue.Full as err:
            rospy.logerr(f"tx queue.Full {err}")

    def spin(self, period=0.00001):
        """
        period - blocking time, where -1 means infinity, 0 means non-blocking
        """
        try:
            if period == -1:
                self.node.spin()
            else:
                self.node.spin(period)
        except dronecan.transport.TransferError as err:
            rospy.logerr(f"spin uavcan.transport.TransferError {err}")
        except queue.Full as err:
            rospy.logerr(f"spin queue.Full {err}")
        except uavcan.driver.common.TxQueueFullError as err:
            rospy.logerr(f"spin uavcan.driver.common.TxQueueFullError {err}")

    def node_ahrs_msg_callback(self, event):
        """
        Incoming message structure:
        --------------------------
        timestamp:
            usec: 28128
        orientation_xyzw: [0.9888, 0.1210, -0.0034, 0.0872]
        orientation_covariance: []
        angular_velocity: [-0.0083, 0.0043, 0.0112]
        angular_velocity_covariance: []
        linear_acceleration: [-0.3149, 1.8408, -10.5703]
        linear_acceleration_covariance: []
        --------------------------
        """
        node_id = event.transfer.source_node_id
        if node_id in self.pubs:
            self.pubs[node_id].publish_imu(event.message)
        else:
            rospy.logwarn(f"Ahrs message has been received from unknown node id={node_id}")

if __name__=="__main__":
    if len(sys.argv) == 2:
        DEV_PATH = sys.argv[1]

    rospy.init_node("ground_station_communicator")

    communicator = None
    while communicator is None and not rospy.is_shutdown():
        try:
            communicator = DroneCanCommunicator(CAN_DEVICE_TYPE)
        except OSError as e:
            rospy.logerr(f"{e}. Check you device. Trying to reconnect.")
            time.sleep(2)
    rospy.logerr("UavcanCommunicatorV0 has been successfully created")

    try:
        while not rospy.is_shutdown():
            communicator.spin(0.2)
    except KeyboardInterrupt:
        print("Interrupt occurs")
        communicator.__del__()
        sys.exit(0)
