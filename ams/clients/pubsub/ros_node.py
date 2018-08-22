#!/usr/bin/env python
# coding: utf-8

import rospy
from multiprocessing import Manager

from ams import AttrDict, logger
from ams.structures import CLIENT


class PubSubClient(object):
    CONST = CLIENT.PUBSUB.BASE_CLIENTS.ROS_NODE

    def __init__(self):
        self.__client = rospy
        self.__manager = Manager()
        self.__subscribers_lock = self.__manager.Lock()
        self.__subscribers = {}
        self.__publishers = {}

    def connect(self):
        self.__client.init_node(name="ros_ams_node", anonymous=True)

    def disconnect(self):
        pass

    def publish(self, topic, message, structure, qos=0, retaion=False):
        if topic not in self.__publishers:
            self.__publishers[topic] = self.__client.Publisher(topic, structure)
        attr_dict = AttrDict.set_recursively(message)
        self.__publishers[topic].publish(structure(**attr_dict))

    def subscribe(self, topic, callback, qos=0, user_data=None, structure=None):
        def on_message(message_data):
            self.__subscribers_lock.acquire()
            callback(self.__client, user_data, topic, message_data)
            self.__subscribers_lock.release()

        self.__subscribers[topic] = {
            "topic": topic,
            "callback": callback,
            "qos": qos,
            "user_data": user_data
        }

        if self.__client is not None:
            self.__client.Subscriber(
                topic,
                structure,
                on_message
            )

    def unsubscribe(self):
        pass
