#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String

class MySubscriberNode(DTROS):
    def __init__(self, node_name):
        super(MySubscriberNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.sub = rospy.Subscriber('Chatter', String, self.callback)
    
    def callback(self, msg):
        rospy.loginfo(f'I heard {msg.data}')


if __name__ == '__main__':
    node = MySubscriberNode('my_subscriber_node')
    rospy.spin()