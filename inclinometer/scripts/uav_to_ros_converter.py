#!/usr/bin/env python3

# Common
import logging
import coloredlogs, logging
import time
import sys

# For uavcan v0.1
import serial
#import uavcan  - old version 
import dronecan
from dronecan import uavcan
#import can
import queue

import rospy
from sensor_msgs.msg import Imu

DEV_PATH = "/dev/ttyACM0"
CAN_DEVICE_TYPE = "can-slcan"

class RosPublisher:
    def __init__(self, topic_1, topic_2):
        self.ros_msg = Imu()
        self.pub_1 = rospy.Publisher(topic_1, Imu, queue_size=10)
        self.pub_2 = rospy.Publisher(topic_2, Imu, queue_size=10)

    def publishFirstImu(self, uavcan_msg):
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

        
        self.pub_1.publish(self.ros_msg)
        self.pub_2.publish(self.ros_msg)


class DroneCanCommunicator:
    """
    Simple wrap on droneCan
    Based on example: https://legacy.uavcan.org/Implementations/Pyuavcan/Tutorials/
    """
    def __init__(self, can_device_type, rosPublisher, node_id=42, node_name="uavcan communicator"):
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
        self.rosPublisher = rosPublisher

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
            if(period == -1):
                self.node.spin()
            else:
                self.node.spin(period)
        except uavcan.transport.TransferError as e:
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
        #print("Source node id: ", event.transfer.source_node_id)
        #print(event.message.timestamp.usec)
        if event.transfer.source_node_id == 81:
            print("Node id: 81")
            self.rosPublisher.publishFirstImu(event.message)

if __name__=="__main__":
    coloredlogs.install()

    # Init dev path using arguments
    if len(sys.argv) == 2:
        DEV_PATH = sys.argv[1]

    # Init publisher
    rospy.init_node("ground_station_communicator")
    publisher = RosPublisher("/imu1/inclinometer", "/imu2/inclinometer")

    # Init communicator
    communicator = None
    while communicator is None:
        try:
            communicator = DroneCanCommunicator(CAN_DEVICE_TYPE, publisher)
        except OSError as e:
            logging.error("{}. Check you device. Trying to reconnect.".format(e))
            time.sleep(2)
    logging.warning("UavcanCommunicatorV0 has been successfully created")
    
    # Register subscriber just for logging communcation status
    #data_type = uavcan.protocol.NodeStatus
    #callback = lambda event: print(event)
    #communicator.subscribe(data_type, callback)

    # Prepare publisher
    geodetioc_pose = [int(55.7544426 * 100000000),
                      int(48.742684 * 100000000),
                      int(-6.5 * 1000)]
    ned_velocity = [0.0,
                    0.0,
                    0.0]
    msg = uavcan.equipment.gnss.Fix(latitude_deg_1e8=geodetioc_pose[0],
                                    longitude_deg_1e8=geodetioc_pose[1],
                                    height_msl_mm=geodetioc_pose[2],
                                    ned_velocity=ned_velocity,
                                    sats_used=10,
                                    status=3,
                                    pdop=99)

    try:                                
        while True:
            communicator.spin(0.2)
    except KeyboardInterrupt:
        print("Interrupt occurs")
        communicator.__del__()
        sys.exit(0)