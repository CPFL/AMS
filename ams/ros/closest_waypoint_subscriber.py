#!/usr/bin/env python
# coding: utf-8

from time import time

import rospy
import message_filters
from std_msgs.msg import Int32

from ams import Topic, Target
from ams.nodes import EventLoop
from ams.messages import ClosestWaypoint
from ams.structures import CLOSEST_WAYPOINT_SUBSCRIBER, AUTOWARE

import pprint
pp = pprint.PrettyPrinter(indent=2)


class ClosestWaypointSubscriber(EventLoop):
    def __init__(self, _id, period):
        super(ClosestWaypointSubscriber, self).__init__(_id)

        self.__previous_time = time()
        self.__period = period

        self.__topicPubClosestWaypoint = Topic()
        self.__topicPubClosestWaypoint.set_targets(
            self.target, Target.new_target(self.target.id, AUTOWARE.NODE_NAME)
        )
        self.__topicPubClosestWaypoint.set_categories(CLOSEST_WAYPOINT_SUBSCRIBER.TOPIC_CATEGORIES)

        rospy.init_node(CLOSEST_WAYPOINT_SUBSCRIBER.ROSNODE, anonymous=True)

        if self.__period < 0.1:
            self.__publish_mqtt = self.publish
            self.set_main_loop(rospy.spin)

            self.__ROSSubscriber = message_filters.Subscriber(
                CLOSEST_WAYPOINT_SUBSCRIBER.ROSTOPIC, Int32, queue_size=1)
            self.__ROSSubscriber.registerCallback(self.on_message_from_ros)
        else:
            self.set_main_loop(self.__main_loop)

    def on_message_from_ros(self, message_data, _user_data):
        current_time = time()
        if self.__period < current_time - self.__previous_time:
            self.__previous_time += (1+int((current_time - self.__previous_time)/self.__period)) * self.__period

            closest_waypoint = ClosestWaypoint.new_data(
                time=time(),
                index=message_data.data
            )
            payload = self.__topicPubClosestWaypoint.serialize(closest_waypoint)
            self.__publish_mqtt(self.__topicPubClosestWaypoint, payload)

    def __main_loop(self):
        r = rospy.Rate(1.0/self.__period)
        while not rospy.is_shutdown():
            try:
                message_data = rospy.wait_for_message(
                    CLOSEST_WAYPOINT_SUBSCRIBER.ROSTOPIC, Int32, timeout=self.__period)

                closest_waypoint = ClosestWaypoint.new_data(
                    time=time(),
                    index=message_data.data
                )
                payload = self.__topicPubClosestWaypoint.serialize(closest_waypoint)
                self.publish(self.__topicPubClosestWaypoint, payload)
            except rospy.ROSException as e:
                # print(e)
                pass
            except rospy.ROSInterruptException:
                break

            r.sleep()
