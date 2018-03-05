#!/usr/bin/env python
# coding: utf-8

from time import time

import rospy
import message_filters
from autoware_msgs.msg import state as AWDecisionMakerStates

from ams import Topic, Target
from ams.nodes import EventLoop
from ams.messages import DecisionMakerStates
from ams.structures import DECISION_MAKER_STATES_SUBSCRIBER, AUTOWARE

import pprint
pp = pprint.PrettyPrinter(indent=2)


class DecisionMakerStatesSubscriber(EventLoop):
    
    CONST = DECISION_MAKER_STATES_SUBSCRIBER

    def __init__(self, _id, period):
        super(DecisionMakerStatesSubscriber, self).__init__(_id)

        self.__previous_time = time()
        self.__period = period

        self.__topicPubDecisionMakerState = Topic()
        self.__topicPubDecisionMakerState.set_targets(
            self.target, Target.new_target(self.target.id, AUTOWARE.NODE_NAME))
        self.__topicPubDecisionMakerState.set_categories(DECISION_MAKER_STATES_SUBSCRIBER.TOPIC_CATEGORIES)

        rospy.init_node(DECISION_MAKER_STATES_SUBSCRIBER.ROSNODE, anonymous=True)

        if self.__period < 0.1:
            self.__publish_mqtt = self.publish
            self.set_main_loop(rospy.spin)

            self.__ROSSubscriber = message_filters.Subscriber(
                AUTOWARE.ROSTOPIC.DECISION_MAKER_STATES, AWDecisionMakerStates, queue_size=1)
            self.__ROSSubscriber.registerCallback(self.on_message_from_ros)
        else:
            self.set_main_loop(self.__main_loop)

    def get_data_from_message(self, message_data):
        return DecisionMakerStates.new_data(
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
            payload = self.__topicPubDecisionMakerState.serialize(decision_maker_states)
            self.__publish_mqtt(self.__topicPubDecisionMakerState, payload)

    def __main_loop(self):
        r = rospy.Rate(1.0/self.__period)
        while not rospy.is_shutdown():
            try:
                message_data = rospy.wait_for_message(
                    AUTOWARE.ROSTOPIC.DECISION_MAKER_STATES, AWDecisionMakerStates, timeout=self.__period)

                decision_maker_states = self.get_data_from_message(message_data)
                payload = self.__topicPubDecisionMakerState.serialize(decision_maker_states)
                self.publish(
                    self.__topicPubDecisionMakerState, payload)
            except rospy.ROSException as e:
                # print(e)
                pass
            except rospy.ROSInterruptException:
                break

            r.sleep()
