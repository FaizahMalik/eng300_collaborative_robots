#!/usr/bin/env python3
import rospy
from digi.xbee.models.status import NetworkDiscoveryStatus
from std_msgs.msg import String
from digi.xbee.devices import DigiMeshDevice
from digi.xbee.devices import DigiMeshNetwork
import time


# This node subscribes to the topic xbee_to_send_topic and broadcasts that data received via the xbee

PORT = "/dev/ttyUSB1"
BAUD_RATE = 9600

DATA_TO_SEND = "Hello XBee! WOW"
REMOTE_NODE_ID = "reciever"

def main():
    print(" +---------------------------------------------+")
    print(" | XBee Python Library Discover Devices Sample |")
    print(" +---------------------------------------------+\n")

    device = DigiMeshDevice(PORT, BAUD_RATE)

    try:
        device.open()

        xbee_network = device.get_network()

        xbee_network.set_discovery_timeout(15)  # 15 seconds.

        xbee_network.clear()

        # Callback for discovered devices.
        def callback_device_discovered(remote):
            print("Device discovered: %s" % remote)

        # Callback for discovery finished.
        def callback_discovery_finished(status):
            if status == NetworkDiscoveryStatus.SUCCESS:
                print("Discovery process finished successfully.")
            else:
                print("There was an error discovering devices: %s" % status.description)

        xbee_network.add_device_discovered_callback(callback_device_discovered)

        xbee_network.add_discovery_process_finished_callback(callback_discovery_finished)

        xbee_network.start_discovery_process()

        print("Discovering remote XBee devices...")

        while xbee_network.is_discovery_running():
            time.sleep(0.1)

    finally:
        if device is not None and device.is_open():
            device.close()


if __name__ == '__main__':
    main()


# def xbee_send_data(msg):
    # xnet.set_deep_discovery_options(deep_mode=NeighborDiscoveryMode.CASCADE, del_not_discovered_nodes_in_last_scan=True)
    # xnet.set_deep_discovery_timeout(node_timeout=30, time_bw_requests=10,
    #                                 time_bw_scans=20)

    # remote_device = xnet.discover_device(REMOTE_NODE_ID)
    # if remote_device is None:
    #     print("Could not find the remote device")
    #     exit(1)
    # print("Sending data to %s >> %s..." % (remote_device.get_64bit_addr(), msg))
    #
    # xbee.send_data(remote_device, msg)

    # print("Success")
    # xbee.send_data_broadcast(msg)
    # print("data sent")
    # xbee.close()
    # exit()


# if __name__ == '__main__':
#     xbee = DigiMeshDevice(PORT, BAUD_RATE)
#     xbee.open()
#
#     xnet = xbee.get_network()
#
#
#     def callback(status):
#         if status == NetworkDiscoveryStatus.ERROR_READ_TIMEOUT:
#             print("finished")
#
#
#     # Add the discovery process finished callback.
#     xnet.add_discovery_process_finished_callback(callback)
#
#     # Start the discovery process and wait for it to be over.
#     xnet.start_discovery_process(deep=True, n_deep_scans=1)
#     # xnet.start_discovery_process()
#     while xnet.is_discovery_running():
#         time.sleep(0.5)
#
#     # Get the list of the nodes in the network.
#     nodes = xnet.get_devices()
#
#     # get_devices() and get_connections() methods return a pointer, so you need to print it like below
#     print("%s" % '\n'.join(map(str, xnet.get_devices())))
#     print("%s" % '\n'.join(map(str, xnet.get_connections())))
#
#
#     if xbee is not None and xbee.is_open():
#         xbee.close()


# def callback(data):
#     rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data)
#
# def listener():
#     rospy.init_node('listener', anonymous=False)
#     rospy.Subscriber("xbee_to_send_topic", String, xbee_send_data)
#     # spin() simply keeps python from exiting until this node is stopped
#     rospy.spin()
#
# def xbee_send_data(data):
#     rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data)
#     device = XBeeDevice(PORT, BAUD_RATE)
#     device.open()
#     print("sending")
#     device.send_data_broadcast(rospy.get_caller_id() + " I heard %s", data.data)
#     print("data sent")
#     device.close()
#     exit()
