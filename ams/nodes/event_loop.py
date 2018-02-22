#!/usr/bin/env python
# coding: utf-8

from uuid import uuid1 as uuid
import paho.mqtt.client as mqtt
import os
from signal import SIGKILL
from time import time

from ams import Topic
from ams.messages import EventLoopMessage
from ams.structures import EVENT_LOOP


class EventLoop(object):

    CONST = EVENT_LOOP

    def __init__(self, _id=None):
        self.event_loop_id = _id
        if _id is None:
            self.event_loop_id = str(uuid())

        self.__topicSub = Topic()
        self.__topicSub.set_id(self.event_loop_id)
        self.__topicSub.set_root(EVENT_LOOP.TOPIC.SUBSCRIBE)

        self.__topicPub = Topic()
        self.__topicPub.set_id(self.event_loop_id)
        self.__topicPub.set_root(EVENT_LOOP.TOPIC.PUBLISH)

        self.__subscribers = {}
        self.__publishers = {}
        self.__client = None
        self.__main_loop = None
        self.__pid = os.getpid()
        self.set_subscriber(self.__topicSub.private)
        self.__on_message_functions = []
        self.__user_data = None
        self.__user_will = None

    def __del__(self):
        if self.__client is not None:
            event_loop_message = EventLoop.get_message(EVENT_LOOP.STATE.DISCONNECT, self.__pid)
            payload = self.__topicPub.serialize(event_loop_message)
            self.publish(self.__topicPub.private, payload)
            self.end()

    @staticmethod
    def get_message(event, pid):
        return EventLoopMessage.new_data(
            time=time(),
            event=event,
            pid=pid
        )

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
        # print("publish", topic, payload)
        self.__client.publish(topic, payload=payload, qos=qos, retain=retain)

    def subscribe(self):
        for subscriber in self.__subscribers.values():
            self.__client.subscribe(subscriber["topic"])

    def __on_connect(self, _client, _userdata, _flags, response_code):
        if response_code == 0:
            self.subscribe()
        else:
            print('connect status {0}'.format(response_code))

    def __on_message(self, client, userdata, message_data):
        payload = message_data.payload.decode("utf-8")
        if self.__topicSub.root in message_data.topic and \
                self.__topicSub.get_id(message_data.topic) == self.event_loop_id:
            event_loop_message = EventLoopMessage.new_data(**self.__topicSub.unserialize(payload))
            if event_loop_message.event == EVENT_LOOP.STATE.START:
                print(self.__topicSub.root, event_loop_message)
            if event_loop_message.event == EVENT_LOOP.ACTION.KILL:
                print(self.__topicSub.root, event_loop_message)
                self.end()
            if event_loop_message.event == EVENT_LOOP.ACTION.CHECK:
                print(self.__topicSub.root, event_loop_message)
                self.__check()
        for onMessageFunction in self.__on_message_functions:
            onMessageFunction(client, userdata, message_data.topic, payload)
        return True

    def connect(self, host, port):
        self.__client = mqtt.Client(protocol=mqtt.MQTTv311, userdata=self.__user_data)

        will = self.__user_will
        # print(will)
        if will is None:
            event_loop_message = EventLoop.get_message(EVENT_LOOP.STATE.WILL, self.__pid)
            payload = self.__topicPub.serialize(event_loop_message)
            will = {"topic": self.__topicPub.private, "payload": payload}
        self.__client.will_set(will["topic"], payload=will["payload"], qos=2, retain=False)

        self.__client.on_connect = self.__on_connect
        self.__client.on_message = self.__on_message
        self.__client.connect(host=host, port=port, keepalive=EVENT_LOOP.KEEP_ALIVE)

    def start(self, host="localhost", port=1883):
        self.connect(host, port)

        event_loop_message = EventLoop.get_message(EVENT_LOOP.STATE.START, self.__pid)
        payload = self.__topicPub.serialize(event_loop_message)
        self.publish(self.__topicPub.private, payload)

        if self.__main_loop is None:
            self.__client.loop_forever()
        else:
            self.__client.loop_start()
            self.__main_loop()

    def end(self):
        self.__client.loop_stop()
        self.__client.disconnect()
        self.__client = None
        os.kill(self.__pid, SIGKILL)

    def __check(self):
        # todo: main_loop zombie
        event_loop_message = EventLoop.get_message(EVENT_LOOP.RESPONSE.OK, self.__pid)
        payload = self.__topicPub.serialize(event_loop_message)
        self.publish(self.__topicPub.private, payload)

    def get_pid(self):
        return self.__pid
