#!/usr/bin/env python3
# could do python3 and install some packages from medium site
import math
import sys
# temporary - for pc only, might not work on rpi

import rospy
from digi.xbee.models.message import XBeeMessage
from std_msgs.msg import String
from digi.xbee.devices import DigiMeshDevice
import time


# specify port and baud rate of your xbee device here
# set_port = f"/dev/{sys.argv[1]}"  # temporary - for pc only, might not work on rpi - specify w launch file
# ttyUSB0 or ttyUSB1


print(rospy.get_param('xbee_port', 'does_not_exist'))
print(rospy.get_param('robot_name', 'does_not_exist'))

robot_name = rospy.get_param('robot_name', 'does_not_exist')
set_port = f"/dev/{rospy.get_param('xbee_port', 'does_not_exist')}"
set_baud_rate = 9600

"""
Could have a method that tests the communication strength between the different devices - will probably use
a deep discovery process
"""


class XbeeInterface:
    """- listens for data from other xbee devices once instantiated
    - sends data received by the RosRelay class to other xbee devices
    - MAX_MESSAGE_SIZE is the maximum amount of bytes a single packet may contain
    """
    def __init__(self, port: str, baud_rate: int, callback, max_message_size: int = 73):
        self.port = port
        self.baud_rate = baud_rate
        self.xbee = DigiMeshDevice(self.port, self.baud_rate)
        self.xbee.open()
        self.xbee.add_data_received_callback(callback)
        self.xnet = self.xbee.get_network()
        self.max_message_size = max_message_size - 2 # reserve 2 bytes for message length


    def xbee_broadcast(self, msg):
        rospy.loginfo('In broadcaster')


        msg = msg.data
        iterations_needed = math.ceil(len(msg) / self.max_message_size)

        if iterations_needed > 99:
            rospy.loginfo('Message too long')
            return

        rospy.loginfo(f"Okay for real, going to send this message now: {msg}")

        # let the receiver know there is no more data to send
        for i in self.data_splitter(msg, iterations_needed):
            rospy.loginfo(f"Sending chunk - {i}")
            self.xbee.send_data_broadcast(i)

        self.xbee.send_data_broadcast(str(iterations_needed))



    def xbee_discover(self):
        """not a happy method"""
        self.xnet.start_discovery_process()
        while self.xnet.is_discovery_running():
            time.sleep(0.5)
        nodes = self.xnet.get_devices()
        if nodes is not None:
            print(map(str, nodes))
            for i in map(str, nodes):
                print(i)
        else:
            print("Could not find the remote device")
        return


    def local_device_info(self):
        """Returns information about your local xbee device"""
        device_info = f"{self.xbee.get_protocol()}\nPort: {self.port}\nBaud rate: {self.baud_rate}\nMac address: {self.xbee.get_64bit_addr()}\nNode ID: {self.xbee.get_node_id()}\n"
        return device_info


    def data_splitter(self, data_to_split, iterations_needed):
        i = 0

        while i < iterations_needed:
            start = self.max_message_size * i
            data_to_send = data_to_split[start:start + self.max_message_size]
            # add current message index to the string
            data_to_send = f"{i}{data_to_send}".zfill(len(data_to_send) + 2)

            yield data_to_send

            i += 1

    def relay_received_data(self):
        return

    def shutdown(self):
        self.xbee.close()

class RosRelay:
    def __init__(self):
        self.incoming = rospy.Publisher('incoming_data', String, queue_size=10)
        self.outgoing = rospy.Publisher('outgoing_data', String, queue_size=10) # just for now
        rospy.init_node('communication_interface', anonymous=True)

        self.data_per_xbee = {}

    def set_outgoing_data_callback(self, callback):
        rospy.Subscriber('outgoing_data', String, callback)

    def incoming_data_publisher(self, msg: XBeeMessage):
        remote_mac = msg.remote_device.get_node_id()

        if not remote_mac in self.data_per_xbee:
            self.data_per_xbee[remote_mac] = []

        # should be reference
        data = msg.data.decode()
        msg_index = data[0:2]
        rospy.loginfo(data)

        # only the last msg is 2 bytes long
        if len(data) > 3:
            self.data_per_xbee[remote_mac].insert(int(msg_index), data)
        elif int(data) == len(self.data_per_xbee[remote_mac]):
            self.incoming.publish(''.join(self.data_per_xbee[remote_mac]))
            self.data_per_xbee[remote_mac].clear()
        else:
            rospy.loginfo("Houston, we are missing a packet")


        # rospy.loginfo(f"Hi from inside RosRelay::incoming_data_published - {msg_processed}")
        # self.incoming.publish(msg.data.decode())
        # print(msg_processed)


    def outgoing_data_publisher(self, msg): # just for now "lol"
        """not needed for main program, but just used now for testing"""
        rospy.loginfo("publishing to the outgoing data topic: " + msg)
        self.outgoing.publish(msg)
        print(msg)

def da_random_incoming_data_subscriber(data):
    rospy.loginfo(data.data)


if __name__ == '__main__':
    ros_talker = RosRelay()
    xbee_talker = XbeeInterface(set_port, set_baud_rate, ros_talker.incoming_data_publisher)
    rospy.Subscriber('incoming_data', String, da_random_incoming_data_subscriber)
    timeout = rospy.Rate(1)

    try:
        # topic returns an object formatted as {data: "published data"}, that's why lambda
        ros_talker.set_outgoing_data_callback(xbee_talker.xbee_broadcast)

        da_data = "001010011101-1000101-1010101010001-11110100110101001101010101111111111111111111111000000000001010100101100101010101-10010101010101010010010101000110001111111"
        ros_talker.outgoing_data_publisher(da_data)

        print(xbee_talker.local_device_info())
        rospy.loginfo(xbee_talker.local_device_info())

        ros_talker.outgoing_data_publisher(da_data)

        while not rospy.is_shutdown():
            # user_payload = input("insert message to send: ")
            # status not ok at 74 byte / 74-character message
            # maximum payload is 73. I need to split large data

            # ros_talker.outgoing_data_publisher(da_data)

            timeout.sleep()

    except rospy.ROSInterruptException:
        rospy.logerr('Ros Interrupt Exception')
    finally:
        print("closing serial connection")
        xbee_talker.shutdown()

