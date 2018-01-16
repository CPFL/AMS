#!/usr/bin/env python
# coding: utf-8

import rospy
from ams import Topic
from ams.nodes import EventLoop, Autoware
from ams.messages import autoware_message

import message_filters
from std_msgs.msg import Int32
from time import time

import pprint
pp = pprint.PrettyPrinter(indent=2)


class ClosestWaypointSubscriber(EventLoop):
    def __init__(self, name, host, port):
        super(ClosestWaypointSubscriber, self).__init__()

        self.autowareSubscribeTopic = Topic()
        self.autowareSubscribeTopic.set_id(name)
        self.autowareSubscribeTopic.set_root(Autoware.TOPIC.SUBSCRIBE)
        self.autowareSubscribeTopic.set_message(autoware_message)

        self.__previous_time = time()
        self.__period = 1.0  # [sec]

        self.connect(host, port)
        self.set_main_loop(rospy.spin)

        rospy.init_node("ams_closest_waypoint_subscriber", anonymous=True)
        self.__ROSSubscriber = message_filters.Subscriber(Autoware.ROSTOPIC.CLOSEST_WAYPOINT, Int32)
        self.__ROSSubscriber.registerCallback(self.on_message_from_ros, self.publish)

    def on_message_from_ros(self, messageData, publish):
        current_time = time()
        if self.__period < current_time - self.__previous_time:
            self.__previous_time += (1+int((current_time - self.__previous_time)/self.__period)) * self.__period

            message = self.autowareSubscribeTopic.get_template()["closest_waypoint"]
            message["index"] = messageData.data
            payload = self.autowareSubscribeTopic.serialize(message)
            publish(self.autowareSubscribeTopic.private+"/closest_waypoint", payload)

