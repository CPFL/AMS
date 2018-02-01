#!/usr/bin/env python
# coding: utf-8

from uuid import uuid1 as uuid
import paho.mqtt.client as mqtt
import os
from signal import SIGKILL
from time import time

from ams import Topic
from ams.messages import EventLoopMessage


class EventLoop(object):

    KEEP_ALIVE = 60

    class TOPIC(object):
        PUBLISH = "pub_event_loop"
        SUBSCRIBE = "sub_event_loop"

    class ACTION(object):
        CHECK = "check"
        KILL = "kill"

    class STATE(object):
        START = "start"
        WILL = "will"
        DISCONNECT = "disconnect"

    class RESPONSE(object):
        OK = "ok"

    def __init__(self, _id=None):
        self.event_loop_id = _id
        if _id is None:
            self.event_loop_id = str(uuid())

        self.__topicSub = Topic()
        self.__topicSub.set_id(self.event_loop_id)
        self.__topicSub.set_root(EventLoop.TOPIC.SUBSCRIBE)

        self.__topicPub = Topic()
        self.__topicPub.set_id(self.event_loop_id)
        self.__topicPub.set_root(EventLoop.TOPIC.PUBLISH)

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
            evennt_loop_message = EventLoop.get_message(EventLoop.STATE.DISCONNECT, self.__pid)
            payload = self.__topicPub.serialize(evennt_loop_message)
            self.publish(self.__topicPub.private, payload)

            self.__client.disconnect()

    @staticmethod
    def get_message(event, pid):
        return EventLoopMessage.get_data(
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
        self.__client.publish(topic, payload=payload, qos=qos, retain=retain)
        self.__client.loop_start()

    def __on_message(self, client, userdata, message_data):
        payload = message_data.payload.decode("utf-8")
        if self.__topicSub.root in message_data.topic and \
                self.__topicSub.get_id(message_data.topic) == self.event_loop_id:
            event_loop_message = EventLoopMessage.get_data(**self.__topicSub.unserialize(payload))
            if event_loop_message.event == EventLoop.STATE.START:
                print(self.__topicSub.root, event_loop_message)
            if event_loop_message.event == EventLoop.ACTION.KILL:
                print(self.__topicSub.root, event_loop_message)
                self.end()
            if event_loop_message.event == EventLoop.ACTION.CHECK:
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
            event_loop_message = EventLoop.get_message(EventLoop.STATE.WILL, self.__pid)
            payload = self.__topicPub.serialize(event_loop_message)
            will = {"topic": self.__topicPub.private, "payload": payload}
        self.__client.will_set(will["topic"], payload=will["payload"], qos=2, retain=False)

        self.__client.on_message = self.__on_message
        self.__client.connect(host=host, port=port, keepalive=EventLoop.KEEP_ALIVE)

    def start(self, host="localhost", port=1883):
        self.connect(host, port)

        for subscriber in self.__subscribers.values():
            self.__client.subscribe(subscriber["topic"])

        event_loop_message = EventLoop.get_message(EventLoop.STATE.START, self.__pid)
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
        os.kill(self.__pid, SIGKILL)

    def __check(self):
        # todo: main_loop zombie
        event_loop_message = EventLoop.get_message(EventLoop.RESPONSE.OK, self.__pid)
        payload = self.__topicPub.serialize(event_loop_message)
        self.publish(self.__topicPub.private, payload)

    def get_pid(self):
        return self.__pid
