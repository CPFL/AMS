#!/usr/bin/env python
# coding: utf-8

import rospy
from eventLoop import EventLoop
from topic import Topic

import message_filters
from std_msgs.msg import Int32
from time import time
from const.autoware import AUTOWARE

import pprint
pp = pprint.PrettyPrinter(indent=2)


class ClosestWaypointSubscriber(EventLoop):
    def __init__(self, name):
        super(ClosestWaypointSubscriber, self).__init__()

        self.autowareSubscribeTopic = Topic()
        self.autowareSubscribeTopic.setID(name)
        self.autowareSubscribeTopic.setRoot(AUTOWARE.TOPIC.SUBSCRIBE)
        self.autowareSubscribeTopic.load(AUTOWARE.TOPIC.MESSAGE_FILE)

        self.__previous_time = time()
        self.__period = 1.0  # [sec]

        self.connect()
        self.setMainLoop(rospy.spin)

        rospy.init_node("ams_closest_waypoint_subscriber", anonymous=True)
        self.__ROSSubscriber = message_filters.Subscriber(AUTOWARE.ROSTOPIC.SUBSCRIBE, Int32)
        self.__ROSSubscriber.registerCallback(self.onMessageFromROS, self.publish)

    def onMessageFromROS(self, messageData, publish):
        current_time = time()
        if self.__period < current_time - self.__previous_time:
            self.__previous_time += (1+int((current_time - self.__previous_time)/self.__period)) * self.__period

            message = self.autowareSubscribeTopic.getTemplate()["closest_waypoint"]
            message["index"] = messageData.data
            payload = self.autowareSubscribeTopic.serialize(message)
            publish(self.autowareSubscribeTopic.private+"/closest_waypoint", payload)


if __name__ == '__main__':
    import sys
    closestWaypointSubscriber = ClosestWaypointSubscriber(name=sys.argv[1])
    print("closestWaypointSubscriber {} on {}".format(closestWaypointSubscriber.getEventLoopID(), closestWaypointSubscriber.getPid()))
    closestWaypointSubscriber.start()
