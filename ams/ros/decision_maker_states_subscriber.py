#!/usr/bin/env python
# coding: utf-8

from time import time

import rospy
import message_filters
from autoware_msgs.msg import state as DecisionMakerStates

from ams import Topic
from ams.nodes import EventLoop, Autoware
from ams.messages import DecisionMakerStates as AMSDecisionMakerStates

import pprint
pp = pprint.PrettyPrinter(indent=2)


class DecisionMakerStatesSubscriber(EventLoop):
    def __init__(self, name, period):
        super(DecisionMakerStatesSubscriber, self).__init__()

        self.topicSubDecisionMakerStates = Topic()
        self.topicSubDecisionMakerStates.set_id(name)
        self.topicSubDecisionMakerStates.set_root(Autoware.CONST.TOPIC.SUBSCRIBE)

        self.__name = name
        self.__previous_time = time()
        self.__period = period

        rospy.init_node(Autoware.CONST.ROSNODE.AMS_DECISION_MAKER_STATES_SUBSCRIBER, anonymous=True)

        if self.__period < 0.1:
            self.__publish_mqtt = self.publish
            self.set_main_loop(rospy.spin)

            self.__ROSSubscriber = message_filters.Subscriber(
                Autoware.CONST.ROSTOPIC.DECISION_MAKER_STATES, DecisionMakerStates, queue_size=1)
            self.__ROSSubscriber.registerCallback(self.on_message_from_ros)
        else:
            self.set_main_loop(self.__main_loop)

    def get_data_from_message(self, message_data):
        return AMSDecisionMakerStates.new_data(
            name=self.__name,
            time=message_data.header.stamp.secs + 0.000000001*message_data.header.stamp.nsecs,
            main=message_data.main_state,
            accel=message_data.acc_state,
            steer=message_data.str_state,
            behavior=message_data.behavior_state
        )

    def on_message_from_ros(self, message_data, _user_data):
        current_time = time()
        if self.__period < current_time - self.__previous_time:
            self.__previous_time += (1+int((current_time - self.__previous_time)/self.__period)) * self.__period

            decision_maker_states = self.get_data_from_message(message_data)
            payload = self.topicSubDecisionMakerStates.serialize(decision_maker_states)
            self.__publish_mqtt(
                self.topicSubDecisionMakerStates.private + Autoware.CONST.TOPIC.DECISION_MAKER_STATES, payload)

    def __main_loop(self):
        r = rospy.Rate(1.0/self.__period)
        while not rospy.is_shutdown():
            try:
                message_data = rospy.wait_for_message(
                    Autoware.CONST.ROSTOPIC.DECISION_MAKER_STATES, DecisionMakerStates, timeout=self.__period)

                decision_maker_states = self.get_data_from_message(message_data)
                payload = self.topicSubDecisionMakerStates.serialize(decision_maker_states)
                self.publish(
                    self.topicSubDecisionMakerStates.private + Autoware.CONST.TOPIC.DECISION_MAKER_STATES, payload)
            except rospy.ROSException as e:
                # print(e)
                pass
            except rospy.ROSInterruptException:
                break

            r.sleep()
