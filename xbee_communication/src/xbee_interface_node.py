#!/usr/bin/env python3
# could do python3 and install some packages from medium site
import math
import sys  # used for specifying the xbee usb port command line argument when using rosrun
import rospy
from digi.xbee.exception import XBeeException

from xbee_communication.msg import NetworkMap
from xbee_communication.msg import LocalMap
from rospy_message_converter import json_message_converter as message_converter
from digi.xbee.devices import DigiMeshDevice
import time

# specify port and baud rate of your xbee device here
# set_port = f"/dev/{sys.argv[1]}"  # temporary - for pc only, might not work on rpi - specify w launch file

# command line: rosrun xbee_communication xbee_interface.py ttyUSB0 or ttyUSB1

# get params from launch files, or set the default to command line argument if not using the launch files
robot_name = rospy.get_param('robot_name', 'robert')
set_port = f"/dev/{rospy.get_param('xbee_port', sys.argv[1])}"
set_baud_rate = 9600

"""
possible feature: a method that tests the communication strength between the different devices - will probably use
a deep discovery process & is likely to be more power intensive
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
        self.max_message_size = max_message_size - 3
        self.messagesPerSecond = rospy.Rate(10)

    # DEBUG
    def local_device_info(self):
        """Returns information about your local xbee device"""
        device_info = f"{self.xbee.get_protocol()}\nPort: {self.port}\nBaud rate: {self.baud_rate}\nMac address: {self.xbee.get_64bit_addr()}\nNode ID: {self.xbee.get_node_id()}\n"
        return device_info


    def xbee_broadcast(self, msg):
        """Checks how many times the data needs to be split, and sends data to be split into the data splitter method"""
        msg = message_converter.convert_ros_message_to_json(msg)
        msg = msg.replace(' ', '')  # COMMENT - FOR MEASURING DATA SAVED/OTHER STATS - TEST
        iterations_needed = self.calculate_chunks_needed(msg)

        if iterations_needed > 999:
            rospy.logerr(f'Message from {rospy.get_namespace()}/outgoing_local_map is too long for the xbee to broadcast')
            return

        for chunk in self.data_splitter(msg):
            self.messagesPerSecond.sleep()
            self.do_send(chunk)

        self.xbee.send_data_broadcast(str(iterations_needed).zfill(3))  # send final msg with number of packets that should've been received

        rospy.logwarn(f'{robot_name} has broadcast {str(iterations_needed).zfill(3).zfill(3)} packets')

    def calculate_chunks_needed(self, msg):
        return math.ceil(len(msg) / self.max_message_size)

    def data_splitter(self, msg):
        i = 0
        iterations_needed = self.calculate_chunks_needed(msg)

        while i < iterations_needed:
            start = int(self.max_message_size * i)
            end = int(start + self.max_message_size)
            data_to_send = msg[start:end]
            yield f"{str(i).zfill(3)}{data_to_send}"  # add current message index to the string
            i += 1

    def do_send(self, msg):
        try:
            self.xbee.send_data_broadcast(msg)
        except XBeeException as e:
            rospy.logerr(f'{robot_name}: Something went wrong,  sending data again {msg[:3]}, {msg} - {str(e)}')
            self.do_send(msg)

    def shutdown(self):
        self.xbee.close()


class RosRelay:
    def __init__(self):
        self.incoming = rospy.Publisher('mrgs/external_map', NetworkMap, queue_size=10)
        self.received_data = {}
        self.mac_addresses = {}
        rospy.init_node('xbee_interface_node', anonymous=True)
        # DEBUG
        # rospy.Subscriber('incoming_data', String, self.incoming_data_subscriber)
        # self.outgoing = rospy.Publisher('outgoing_data', String, queue_size=10)  # just for now


    @staticmethod
    def set_outgoing_data_callback(callback):
        rospy.Subscriber('outgoing_local_map', NetworkMap, callback, queue_size=1)

    def incoming_data_publisher(self, msg):
        rospy.loginfo(f"publishing to external_map topic: {msg}")
        rospy.loginfo(type(msg))
        self.incoming.publish(msg)


    def process_incoming_data(self, unprocessed_msg):

        processed_msg = unprocessed_msg.data.decode()  # chunk of string
        # TODO Change to mac addr sometime soon, the data interface node needs a list of mac addresses
        sender_mac = str(unprocessed_msg.remote_device.get_64bit_addr())
        if sender_mac not in self.received_data:
            rospy.logwarn('New mac address')
            self.received_data[sender_mac] = {}

        # msg_index = processed_msg[:3]
        # msg_length = len(processed_msg)
        # rospy.loginfo(f"received a message: {processed_msg}, indexing...")


        msg_index = processed_msg[:3]
        msg_data = processed_msg[3:]
        data_length = len(msg_data)
        # rospy.logwarn(f"received a message: {processed_msg}, indexing...")

        # if there is data, process it
        if data_length != 0:
            # purely for logging purposes
            if msg_index in self.received_data[sender_mac] and msg_index == '000':
                rospy.logwarn(f'{robot_name}: Received {msg_index} again... ignoring whatever was sent before')
                self.received_data[sender_mac].clear()

            self.received_data[sender_mac][msg_index] = msg_data
        # it's the final message
        else:
            # self.received_data[sender_mac]['all_data_sent'] = True
            self.received_data[sender_mac]['expected_msg_count'] = int(msg_index)
            rospy.logwarn(f"I, {robot_name}, should have received all my messages! {len(self.received_data[sender_mac]) - 1} / {msg_index}")

        # done manipulating data so assign it to a variable
        sender_data = self.received_data[sender_mac]
        #
        # if robot_name == 'bobert':
        #     rospy.logwarn(f'{robot_name}:Checking if data is complete - {len(sender_data) - 1} / {sender_data.get("expected_msg_count", -2)}')

        # rospy.logwarn()

        if len(sender_data) - 1 == sender_data.get('expected_msg_count', -2):
            rospy.logwarn(f'{robot_name}: Got all me data {"expected_msg_count" in sender_data}')

            # remove this key before joining the data
            # del self.received_data[sender_mac]['all_data_sent']
            del sender_data['expected_msg_count']

            rospy.logwarn(f"{robot_name} Messages received and merged. Now publishing...")
            json_data = ''.join(sender_data.values())
            self.publishJSON(json_data, sender_mac)
            self.received_data[sender_mac].clear()


    def publishJSON(self, json_data, sender_mac):
        # pass

        # convert the received and joined json data into a network map message type
        incoming_network_map = message_converter.convert_json_to_ros_message('xbee_communication/NetworkMap',
                                                                             json_data)
        # replace randomly generated mac address with the actual mac address of the sender
        # incoming_network_map.mac = sender_mac
        # not possible to update the mac address due to it not fitting the expected data type on the network map msg
        # publish to extenal_map

        try:
            incoming_network_map.mac = sender_mac
            self.incoming_data_publisher(incoming_network_map)
            rospy.loginfo(f'{robot_name} just sent some data with mac {incoming_network_map.mac}')
        except Exception as e:
            rospy.logerr(f"Something went wrong publishing the external map topic :( {e}")
        # finally:
        #     self.received_data[sender_mac].clear()

        rospy.logdebug("Just finished sending a message")

if __name__ == '__main__':
    ros_talker = RosRelay()
    xbee_talker = XbeeInterface(set_port, set_baud_rate, ros_talker.process_incoming_data)
    # rospy.Subscriber('incoming_data', String, incoming_data_subscriber)

    rospy.loginfo("Starting XBEE interface")
    timeout = rospy.Rate(1)

    try:
        rospy.logerr(xbee_talker.local_device_info())
        ros_talker.set_outgoing_data_callback(xbee_talker.xbee_broadcast)

        time.sleep(5)  # allow xbees to initialise

        while not rospy.is_shutdown():
            timeout.sleep()

    except rospy.ROSInterruptException:
        rospy.logerr('Ros Interrupt Exception')

    finally:
        print("closing serial connection")
        xbee_talker.shutdown()

