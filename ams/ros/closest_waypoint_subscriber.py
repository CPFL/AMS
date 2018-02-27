#!/usr/bin/env python
# coding: utf-8

from time import time

import rospy
import message_filters
from std_msgs.msg import Int32

from ams import Topic
from ams.nodes import EventLoop, Autoware
from ams.messages import ClosestWaypoint

import pprint
pp = pprint.PrettyPrinter(indent=2)


class ClosestWaypointSubscriber(EventLoop):
    def __init__(self, name, period):
        super(ClosestWaypointSubscriber, self).__init__()

        self.topicSubClosestWaypoint = Topic()
        self.topicSubClosestWaypoint.set_id(name)
        self.topicSubClosestWaypoint.set_root(Autoware.CONST.TOPIC.SUBSCRIBE)

        self.__name = name
        self.__previous_time = time()
        self.__period = period

        rospy.init_node(Autoware.CONST.ROSNODE.AMS_CLOSEST_WAYPOINT_SUBSCRIBER, anonymous=True)

        if self.__period < 0.1:
            self.__publish_mqtt = self.publish
            self.set_main_loop(rospy.spin)

            self.__ROSSubscriber = message_filters.Subscriber(Autoware.CONST.ROSTOPIC.CLOSEST_WAYPOINT, Int32, queue_size=1)
            self.__ROSSubscriber.registerCallback(self.on_message_from_ros)
        else:
            self.set_main_loop(self.__main_loop)

    def on_message_from_ros(self, message_data, _user_data):
        current_time = time()
        if self.__period < current_time - self.__previous_time:
            self.__previous_time += (1+int((current_time - self.__previous_time)/self.__period)) * self.__period

            closest_waypoint = ClosestWaypoint.new_data(
                name=self.__name,
                time=time(),
                index=message_data.data
            )
            payload = self.topicSubClosestWaypoint.serialize(closest_waypoint)
            self.__publish_mqtt(self.topicSubClosestWaypoint.private + Autoware.CONST.TOPIC.CLOSEST_WAYPOINT, payload)

    def __main_loop(self):
        r = rospy.Rate(1.0/self.__period)
        while not rospy.is_shutdown():
            try:
                message_data = rospy.wait_for_message(
                    Autoware.CONST.ROSTOPIC.CLOSEST_WAYPOINT, Int32, timeout=self.__period)

                closest_waypoint = ClosestWaypoint.new_data(
                    name=self.__name,
                    time=time(),
                    index=message_data.data
                )
                payload = self.topicSubClosestWaypoint.serialize(closest_waypoint)
                self.publish(self.topicSubClosestWaypoint.private + Autoware.CONST.TOPIC.CLOSEST_WAYPOINT, payload)
            except rospy.ROSException as e:
                # print(e)
                pass
            except rospy.ROSInterruptException:
                break

            r.sleep()
