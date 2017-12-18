#!/usr/bin/env python
# coding: utf-8

import rospy
import json
from eventLoop import EventLoop
from const import CONST

import message_filters
from std_msgs.msg import Int32
from time import time

import pprint
pp = pprint.PrettyPrinter(indent=2)


class ClosestWaypointSubscriber(EventLoop):
    def __init__(self):
        super(ClosestWaypointSubscriber, self).__init__()
        self.__previous_time = time()
        self.__period = 1.0  # [sec]

        self.connect()
        self.setMainLoop(rospy.spin)

        rospy.init_node("ams_closest_waypoint_subscriber", anonymous=True)
        self.__ROSSubscriber = message_filters.Subscriber('/closest_waypoint', Int32)
        self.__ROSSubscriber.registerCallback(self.onMessageFromROS, self.publish)

    def onMessageFromROS(self, messageData, publish):
        current_time = time()
        if self.__period < current_time - self.__previous_time:
            self.__previous_time += (1+int((current_time - self.__previous_time)/self.__period)) * self.__period
            message = json.dumps({"index": messageData.data})
            publish(CONST.TOPICS.AUTOWARE.CLOSEST_WAYPOINT, message)


if __name__ == '__main__':
    closestWaypointSubscriber = ClosestWaypointSubscriber()
    print("closestWaypointSubscriber {} on {}".format(closestWaypointSubscriber.getEventLoopID(), closestWaypointSubscriber.getPid()))
    closestWaypointSubscriber.start()
