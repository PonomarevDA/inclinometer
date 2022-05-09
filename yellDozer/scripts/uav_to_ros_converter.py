"""
This module contains can handler and ros handler parts.
Can handler class receives imu data from slcan.
Ros handler converts imu data to ros message and publish
each imu to separate topic.
"""
#!/usr/bin/env python3

# Common
import logging
import coloredlogs
import time
import sys

# For uavcan v0.1
import dronecan
from dronecan import uavcan
#import can
import queue

import rospy
from sensor_msgs.msg import Imu

DEV_PATH = "/dev/ttyACM0"
CAN_DEVICE_TYPE = "can-slcan"

class RosPublisher:
    """
    Class used for publishing particular imu data.
    It converts incoming uavcan message to ros Imu message
    """
    def __init__(self, topic_name):
        self.ros_msg = Imu()
        self.publisher = rospy.Publisher(topic_name, Imu, queue_size=10)

    def publish_imu(self, uavcan_msg):
        """
        Function converts incoming uavcan message to ros Imu message
        and publish it.
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
        self.handlers = []
        self.node = None

        self.tx_can_error_counter = 0
        self.tx_full_buffer_error = 0
        self.spin_can_error_counter = 0
        self.spin_transfer_error_counter = 0
        self.imu_1_publisher =  RosPublisher("/imu1/sensordata")
        self.imu_2_publisher =  RosPublisher("/imu2/sensordata")
        self.imu_3_publisher =  RosPublisher("/imu3/sensordata")

        if can_device_type == "serial":
            kawrgs = {"can_device_name" : DEV_PATH,
                      "baudrate" : 1000000}
        elif can_device_type == "can-slcan":
            kawrgs = {"can_device_name" : "slcan0",
                      "bustype" : "socketcan",
                      "bitrate" : 1000000}
        else:
            logging.error("UavcanCommunicatorV0: Wrong can device type")
            sys.exit()

        node_info = uavcan.protocol.GetNodeInfo.Response()
        node_info.name = node_name
        node_info.software_version.major = 0
        node_info.software_version.minor = 2
        node_info.hardware_version.unique_id = b'12345'

        self.node = dronecan.make_node(node_id=node_id, node_info=node_info, **kawrgs)
        self.subscribe(uavcan.equipment.ahrs.Solution, self.node_AHRS_msg_Callback)

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
        self.handlers.append(self.node.add_handler(data_type, callback))

    def publish(self, data_type):
        """
        param data_type - https://legacy.uavcan.org/Specification/7._List_of_standard_data_types/
        Example:
        fix2 = uavcan.equipment.gnss.Fix2(pdop=10)
        communicator.publish(fix2)
        """
        try:
            self.node.broadcast(data_type)
        except can.CanError as e:
            self.tx_can_error_counter += 1
            logging.error("tx can.CanError {}, №{}".format(
                         e, self.tx_can_error_counter))
        except uavcan.driver.common.TxQueueFullError as e:
            self.tx_full_buffer_error += 1
            logging.error("tx uavcan.driver.common.TxQueueFullError {}, №{}".format(
                         e, self.tx_full_buffer_error))
        except queue.Full as e:
            logging.error("tx queue.Full {}".format(e))

    def spin(self, period=0.00001):
        """
        period - blocking time, where -1 means infinity, 0 means non-blocking
        """
        try:
            if period == -1:
                self.node.spin()
            else:
                self.node.spin(period)
        except dronecan.transport.TransferError as e:
            self.spin_transfer_error_counter += 1
            logging.error("spin uavcan.transport.TransferError {}, №{}".format(
                        e, self.spin_transfer_error_counter))
        except can.CanError as e:
            self.spin_can_error_counter += 1
            logging.error("spin can.CanError {}, №{}".format(
                        e, self.spin_can_error_counter))
        except queue.Full as e:
            logging.error("spin queue.Full {}".format(e))
        except uavcan.driver.common.TxQueueFullError as e:
            logging.error("spin uavcan.driver.common.TxQueueFullError {}".format(e))

    def node_AHRS_msg_Callback(self, event):
        """
        Function is called by can handler.
        It sends imu data to the ros topic with respect to node id
        """
        if event.transfer.source_node_id == 81:
            self.imu_1_publisher.publish_imu(event.message)
        elif event.transfer.source_node_id == 82:
            self.imu_2_publisher.publish_imu(event.message)
        elif event.transfer.source_node_id == 83:
            self.imu_3_publisher.publish_imu(event.message)

if __name__=="__main__":
    coloredlogs.install()

    # Init dev path using arguments
    if len(sys.argv) == 2:
        DEV_PATH = sys.argv[1]

    # Init publisher
    rospy.init_node("ground_station_communicator")

    # Init communicator
    communicator = None
    while communicator is None and not rospy.is_shutdown():
        try:
            communicator = DroneCanCommunicator(CAN_DEVICE_TYPE)
        except OSError as os_err:
            rospy.logerr("{}. Check you device. Trying to reconnect.".format(os_err))
            time.sleep(2)
    rospy.logerr("UavcanCommunicatorV0 has been successfully created")

    try:
        while not rospy.is_shutdown():
            communicator.spin(0.2)
    except KeyboardInterrupt:
        print("Interrupt occurs")
        communicator.__del__()
        sys.exit(0)
