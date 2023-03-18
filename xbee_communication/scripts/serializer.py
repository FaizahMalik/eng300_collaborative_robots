#!/usr/bin/env python2
# could do python3 and install some packages from medium site
import codecs
import math
import sys  # used for specifying the xbee usb port command line argument when using rosrun
from io import StringIO, BytesIO

import rospy
from std_msgs.msg import String
from xbee_communication.msg import NetworkMap
from digi.xbee.devices import DigiMeshDevice
import time


def serializer(msg):
    s = StringIO()
    msg.serialize(s)
    return s