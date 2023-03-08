#!/usr/bin/env python3
# maybe it will work on the rpi

import rospy
from std_msgs.msg import String

TOPIC="xbee_topic"

def callback(da_message):
    rospy.loginfo(f"I heard '{da_message.data}' from flipping {rospy.get_caller_id()}")


def listener_talker():
    pub = rospy.Publisher(TOPIC, String, queue_size=10)
    rospy.init_node(f"listener_talker", anonymous=False)
    # rospy.init_node('listener', anonymous=False)
    rate = rospy.Rate(10)  # 10hz
    rospy.Subscriber(TOPIC, String, callback)

    while not rospy.is_shutdown():
        hello_str = "hello ros i eat ros for breakfast not really because its hard - sent from my iphone"
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()
        # rospy.spin()


if __name__ == '__main__':
    try:
        listener_talker()
    except rospy.ROSInterruptException:
        pass

