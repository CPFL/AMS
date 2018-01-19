#!/usr/bin/env python
# coding: utf-8

from uuid import uuid1 as uuid
import paho.mqtt.client as mqtt
import os
from signal import SIGKILL
from time import time

from ams import Topic
from ams.messages import event_loop_message


class EventLoop(object):
    KEEP_ALIVE = 60

    class TOPIC(object):
        PUBLISH = "pub_event_loop"
        SUBSCRIBE = "sub_event_loop"

    def __init__(self, _id=None):
        self.event_loop_id = _id
        if _id is None:
            self.event_loop_id = str(uuid())

        self.__subscribeTopic = Topic()
        self.__subscribeTopic.set_id(self.event_loop_id)
        self.__subscribeTopic.set_root(EventLoop.TOPIC.SUBSCRIBE)
        self.__subscribeTopic.set_message(event_loop_message)

        self.__publishTopic = Topic()
        self.__publishTopic.set_id(self.event_loop_id)
        self.__publishTopic.set_root(EventLoop.TOPIC.PUBLISH)
        self.__publishTopic.set_message(event_loop_message)

        self.__subscribers = {}
        self.__publishers = {}
        self.__client = None
        self.__main_loop = None
        self.__pid = os.getpid()
        self.set_subscriber(self.__subscribeTopic.private)
        self.__on_message_functions = []
        self.__user_data = None
        self.__user_will = None

    def __del__(self):
        if self.__client is not None:
            message = self.__publishTopic.get_template()
            message["action"] = "disconnect"
            message["pid"] = self.__pid
            payload = self.__publishTopic.serialize(message)
            self.publish(self.__publishTopic.private, payload)

            self.__client.disconnect()

    def set_subscriber(self, topic):
        self.__subscribers[str(uuid())] = {"topic": topic}

    def set_user_data(self, user_data):
        self.__user_data = user_data

    def add_on_message_function(self, on_message_function):
        self.__on_message_functions.append(on_message_function)

    def set_main_loop(self, main_loop):
        self.__main_loop = main_loop

    def set_will(self, topic, payload):
        self.__user_will = {
            "topic": topic,
            "payload": payload
        }

    def publish(self, topic, payload, qos=0, retain=False):
        self.__client.publish(topic, payload=payload, qos=qos, retain=retain)
        if 0 < qos:
            self.__client.loop_start()

    def __on_message(self, client, userdata, message_data):
        payload = message_data.payload.decode("utf-8")
        if self.__subscribeTopic.root in message_data.topic and \
                self.__subscribeTopic.get_id(message_data.topic) == self.event_loop_id:
            message = self.__subscribeTopic.unserialize(payload)
            if message["action"] == "start":
                print(self.__subscribeTopic.root, message)
            if message["action"] == "kill":
                print(self.__subscribeTopic.root, message)
                self.end()
            if message["action"] == "check":
                print(self.__subscribeTopic.root, message)
                self.__check()
        for onMessageFunction in self.__on_message_functions:
            onMessageFunction(client, userdata, message_data.topic, payload)
        return True

    def connect(self, host, port):
        self.__client = mqtt.Client(protocol=mqtt.MQTTv311, userdata=self.__user_data)

        will = self.__user_will
        print(will)
        if will is None:
            message = self.__publishTopic.get_template()
            message["action"] = "will"
            message["pid"] = self.__pid
            payload = self.__publishTopic.serialize(message)
            will = {"topic": self.__publishTopic.private, "payload": payload}
        self.__client.will_set(will["topic"], payload=will["payload"], qos=2, retain=False)

        self.__client.on_message = self.__on_message
        self.__client.connect(host=host, port=port, keepalive=EventLoop.KEEP_ALIVE)

    def start(self, host="localhost", port=1883):
        self.connect(host, port)

        for subscriber in self.__subscribers.values():
            self.__client.subscribe(subscriber["topic"])

        message = self.__publishTopic.get_template()
        message["time"] = str(int(time()))
        message["action"] = "start"
        message["pid"] = self.__pid
        payload = self.__publishTopic.serialize(message)
        self.publish(self.__publishTopic.private, payload)

        if self.__main_loop is None:
            self.__client.loop_forever()
        else:
            self.__client.loop_start()
            self.__main_loop()

    def end(self):
        self.__client.loop_stop()
        self.__client.disconnect()
        os.kill(self.__pid, SIGKILL)

    def __check(self):
        # todo: main_loop zombie
        message = self.__publishTopic.get_template()
        message["time"] = str(int(time()))
        message["action"] = "ok"
        message["pid"] = self.__pid
        payload = self.__publishTopic.serialize(message)
        self.publish(self.__publishTopic.private, payload)

    def get_pid(self):
        return self.__pid
