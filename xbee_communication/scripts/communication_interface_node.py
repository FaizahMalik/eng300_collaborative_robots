#!/usr/bin/env python3
# could do python3 and install some packages from medium site
import math
import sys  # used for specifying the xbee usb port command line argument when using rosrun
from io import StringIO

import lz4.frame
import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from digi.xbee.devices import DigiMeshDevice
import time

# specify port and baud rate of your xbee device here
# set_port = f"/dev/{sys.argv[1]}"  # temporary - for pc only, might not work on rpi - specify w launch file

# command line: rosrun xbee_communication communication_interface.py ttyUSB0 or ttyUSB1

# get params from launch files, or set the default to command line argument if not using the launch files
robot_name = rospy.get_param('robot_name', 'robert')
set_port = f"/dev/{rospy.get_param('xbee_port', sys.argv[1])}"
set_baud_rate = 9600

"""
Could have a method that tests the communication strength between the different devices - will probably use
a deep discovery process
"""


class XbeeInterface:
    """- listens for data from other xbee devices once instantiated
    - sends data received by the RosRelay class to other xbee devices
    - max message size is the maximum amount of bytes a single packet may contain
    """

    def __init__(self, port: str, baud_rate: int, callback, max_message_size: int = 73):
        self.port = port
        self.baud_rate = baud_rate
        self.xbee = DigiMeshDevice(self.port, self.baud_rate)
        self.xbee.open()
        self.xbee.add_data_received_callback(callback)
        self.xnet = self.xbee.get_network()
        self.max_message_size = max_message_size - 3  # reserve 3 bytes for message length

    # def xbee_discover(self):
    #     """might be implemented in the future"""
    #     self.xnet.start_discovery_process()
    #     while self.xnet.is_discovery_running():
    #         time.sleep(0.5)
    #     nodes = self.xnet.get_devices()
    #     if nodes is not None:
    #         print(map(str, nodes))
    #         for i in map(str, nodes):
    #             print(i)
    #     else:
    #         print("Could not find the remote device")
    #     return

    def local_device_info(self):
        """Returns information about your local xbee device"""
        device_info = f"{self.xbee.get_protocol()}\nPort: {self.port}\nBaud rate: {self.baud_rate}\nMac address: {self.xbee.get_64bit_addr()}\nNode ID: {self.xbee.get_node_id()}\n"
        return device_info

    def xbee_broadcast(self, msg):
        """Checks how many times the data needs to be split, and sends data to be split into the data splitter method"""
        msg = msg.data
        #
        # occupancy_grid.serialize(a)
        # a.seek(0)
        # msg = a.read()
        # msg = lz4.frame.compress(msg)
        # rospy.loginfo("WASAAAPPPP" + grid)

        # Serialize the OccupancyGrid into a byte string.
        # grid = str(grid)
        # rospy.loginfo(grid)
        #
        # # Compress the serialized data using lz4.
        # msg = lz4.frame.compress(str.encode(grid))
        # msg = msg.decode('utf-8')
        # rospy.loginfo(msg)

        # Create a new OccupancyGrid message to hold the compressed data.
        # compressed_grid = OccupancyGrid()
        # compressed_grid.header = grid.header
        # compressed_grid.info = grid.info
        # compressed_grid.data = compressed_data

        iterations_needed = math.ceil(len(msg) / self.max_message_size)

        if iterations_needed > 999:
            rospy.loginfo('Message too long')
            return

        for i in self.data_splitter(msg, iterations_needed):
            self.xbee.send_data_broadcast(i)

        self.xbee.send_data_broadcast(str(iterations_needed).zfill(3))  # send final msg with number of packets that should've been received

    def data_splitter(self, data_to_split, iterations_needed):
        """splits data and adds the message index to the start of the string"""
        i = 0

        while i < iterations_needed:
            start = int(self.max_message_size * i)
            end = int(start + self.max_message_size)
            data_to_send = data_to_split[start:end]
            yield f"{str(i).zfill(3)}{data_to_send}"  # add current message index to the string
            i += 1

    def shutdown(self):
        self.xbee.close()


class RosRelay:
    def __init__(self):
        self.incoming = rospy.Publisher('incoming_data', OccupancyGrid, queue_size=10)
        # self.outgoing = rospy.Publisher('map', String, queue_size=10)  # just for now
        rospy.init_node('communication_interface', anonymous=True)

        # rospy.Subscriber('incoming_data', String, self.incoming_data_subscriber)
        self.received_data = {}

    @staticmethod
    def set_outgoing_data_callback(callback):
        rospy.Subscriber('map', OccupancyGrid, callback)

    def outgoing_data_publisher(self, msg):
        """not needed for main program, but just used now for testing"""
        rospy.loginfo("publishing to the outgoing data topic: " + msg)
        # self.outgoing.publish(msg)

    def incoming_data_publisher(self, msg):
        rospy.loginfo(f"publishing to incoming_data topic: {msg}")
        self.incoming.publish(msg)

    def process_incoming_data(self, unprocessed_msg):
        processed_msg = unprocessed_msg.data.decode()
        sender_name = unprocessed_msg.remote_device.get_node_id()

        if sender_name not in self.received_data:
            self.received_data[sender_name] = []

        msg_index = processed_msg[:3]

        rospy.loginfo(f"received a message: {processed_msg}, indexing...")

        if len(processed_msg) > 3:
            self.received_data[sender_name].insert(int(msg_index), processed_msg[3:])
        elif int(processed_msg) == len(self.received_data[sender_name]):
            rospy.loginfo("Messages received and merged. Now publishing...")
            merged_data = ''.join(self.received_data[sender_name])

            self.incoming_data_publisher(lz4.frame.decompress(merged_data))
            self.received_data[sender_name].clear()
        else:
            rospy.logerr("Missing packet")


# TODO DATA TYPE IS STRING, IT SHOULD BE OCCUPANCY GRID. LOOK INTO HOW TO IMPORT OCCUPANCY GRID DATA TYPE IN PYTHON - IF IT;S DIFF FOR PYTHON


def incoming_data_subscriber(data):
    rospy.loginfo(f"from incoming data topic: {data.data}")


if __name__ == '__main__':
    ros_talker = RosRelay()
    xbee_talker = XbeeInterface(set_port, set_baud_rate, ros_talker.process_incoming_data)
    rospy.Subscriber('incoming_data', OccupancyGrid, incoming_data_subscriber)

    timeout = rospy.Rate(1)
    # payload = "The quick brown fox jumps over the lazy dog. The quick brown fox does a lap and yet again jumps over the lazy dog. The dog is horrified. The fox could not care less, and for a third time, jumps over the now seething dog. 'cope with it' the fox said."

    try:
        # rospy.loginfo(xbee_talker.local_device_info())
        ros_talker.set_outgoing_data_callback(xbee_talker.xbee_broadcast)

        time.sleep(1)  # allow xbees to initialise

        # if robot_name == "meepo":
            # ros_talker.outgoing_data_publisher(payload)

        # xbee_talker.xbee_broadcast("sup")

        while not rospy.is_shutdown():
            # payload = input("insert message to send: ")
            # ros_talker.outgoing_data_publisher(payload)
            timeout.sleep()

    except rospy.ROSInterruptException:
        rospy.logerr('Ros Interrupt Exception')

    finally:
        print("closing serial connection")
        xbee_talker.shutdown()

