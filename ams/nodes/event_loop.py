#!/usr/bin/env python
# coding: utf-8

import paho.mqtt.client as mqtt
import os
from ssl import PROTOCOL_TLSv1_2
from signal import SIGKILL
from time import time

from ams import Topic, Target
from ams.messages import EventLoopMessage
from ams.structures import EVENT_LOOP


class EventLoop(object):

    CONST = EVENT_LOOP

    def __init__(self, _id):
        self.event_loop_id = _id
        self.target = Target.new_target(self.event_loop_id, self.__class__.__name__)
        self.__subscribers = {}
        self.__publishers = {}
        self.__client = None
        self.__main_loop = None
        self.__pid = os.getpid()

        self.__topicPub = Topic()
        self.__topicPub.set_targets(Target.new_target(self.event_loop_id, EventLoop.__name__))
        self.__topicPub.set_categories(EVENT_LOOP.TOPIC.CATEGORIES.RESPONSE)

        self.__topicSub = Topic()
        self.__topicSub.set_targets(None, Target.new_target(self.event_loop_id, EventLoop.__name__))
        self.__topicSub.set_categories(EVENT_LOOP.TOPIC.CATEGORIES.REQUEST)
        self.__topicSub.set_message(EventLoopMessage)
        self.set_subscriber(self.__topicSub, self.on_event_loop_message)

        self.__user_data = None
        self.__user_will = None

    def __del__(self):
        if self.__client is not None:
            event_loop_message = EventLoop.get_message(EVENT_LOOP.STATE.DISCONNECT, self.__pid)
            payload = self.__topicPub.serialize(event_loop_message)
            self.publish(self.__topicPub, payload)
            self.end()

    @staticmethod
    def get_message(event, pid):
        return EventLoopMessage.new_data(
            time=time(),
            event=event,
            pid=pid
        )

    def set_subscriber(self, topic, callback):
        self.__subscribers[topic.get_path(use_wild_card=True)] = callback

    def set_user_data(self, user_data):
        self.__user_data = user_data

    def set_main_loop(self, main_loop):
        self.__main_loop = main_loop

    def set_will(self, topic, payload):
        self.__user_will = {
            "topic": topic,
            "payload": payload
        }

    def publish(self, topic, payload, qos=0, retain=False):
        self.__client.publish(
            topic.get_path(),
            payload=payload, qos=qos, retain=retain)

    def subscribe(self):
        for topic in self.__subscribers.keys():
            self.__client.subscribe(topic)

    def response(self, request_path, payload, qos=0, retain=False):
        response_topic = Topic()
        response_topic.set_fix_path(self.__topicPub.get_response_path(request_path))
        self.publish(response_topic, payload, qos, retain)

    def __on_connect(self, _client, _userdata, _flags, response_code):
        if response_code == 0:
            self.subscribe()
        else:
            print('connect status {0}'.format(response_code))

    def on_event_loop_message(self, _client, _userdata, topic, payload):
        event_loop_message = self.__topicSub.unserialize(payload)
        if event_loop_message.event == EVENT_LOOP.STATE.START:
            pass
        if event_loop_message.event == EVENT_LOOP.ACTION.KILL:
            self.end()
        if event_loop_message.event == EVENT_LOOP.ACTION.CHECK:
            self.__check(topic)

    def __on_message(self, client, userdata, message_data):
        payload = message_data.payload.decode("utf-8")
        for subscriber_path, onMessageFunction in self.__subscribers.items():
            if Topic.is_path_matched(subscriber_path, message_data.topic):
                onMessageFunction(client, userdata, message_data.topic, payload)
        return True

    def ssl_setting(self, ca_path, client_path, key_path):
        self.__client.tls_set(ca_path,
                              certfile=client_path,
                              keyfile=key_path,
                              tls_version=PROTOCOL_TLSv1_2)
        self.__client.tls_insecure_set(True)

    def connect(self, host, port, ca_path=None, client_path=None, key_path=None):
        self.__client = mqtt.Client(protocol=mqtt.MQTTv311, userdata=self.__user_data)

        if ca_path is not None and client_path is not None and key_path is not None:
            self.ssl_setting(ca_path, client_path, key_path)

        will = self.__user_will
        if will is None:
            event_loop_message = EventLoop.get_message(EVENT_LOOP.STATE.WILL, self.__pid)
            payload = self.__topicPub.serialize(event_loop_message)
            will = {"topic": self.__topicPub.get_path(), "payload": payload}
        self.__client.will_set(will["topic"], payload=will["payload"], qos=2, retain=False)

        self.__client.on_connect = self.__on_connect
        self.__client.on_message = self.__on_message
        self.__client.connect(host=host, port=port, keepalive=EVENT_LOOP.KEEP_ALIVE)

    def start(self, host="localhost", port=1883, ca_path=None, client_path=None, key_path=None):

        self.connect(host, port, ca_path=ca_path, client_path=client_path, key_path=key_path)

        event_loop_message = EventLoop.get_message(EVENT_LOOP.STATE.START, self.__pid)
        payload = self.__topicPub.serialize(event_loop_message)
        self.publish(self.__topicPub, payload)

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

    def __check(self, request_path):
        # todo: main_loop zombie
        event_loop_message = EventLoop.get_message(EVENT_LOOP.RESPONSE.OK, self.__pid)
        payload = self.__topicPub.serialize(event_loop_message)
        self.response(request_path, payload)

    def get_pid(self):
        return self.__pid
