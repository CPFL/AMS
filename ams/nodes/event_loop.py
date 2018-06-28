#!/usr/bin/env python
# coding: utf-8

import os
import traceback
from time import time
from multiprocessing import Manager
from copy import deepcopy

from signal import SIGKILL
import paho.mqtt.client as mqtt
from _ssl import PROTOCOL_TLSv1_2

from ams import logger
from ams.helpers import Topic, Target
from ams.messages import EventLoopMessage
from ams.structures import EVENT_LOOP


class EventLoop(object):

    CONST = EVENT_LOOP

    def __init__(self, _id):
        self.manager = Manager()

        self.event_loop_id = _id
        self.target = Target.new_target(self.__class__.__name__, self.event_loop_id)
        self.__subscribers = {}
        self.__subscribers_lock = self.manager.Lock()
        self.__publishers = {}
        self.__client = None
        self.__main_loop = None
        self.__pid = os.getpid()

        self.__pub_topic = Topic.get_topic(
            from_target=Target.new_target(EventLoop.__name__, self.event_loop_id),
            categories=EVENT_LOOP.TOPIC.CATEGORIES.RESPONSE
        )

        self.set_subscriber(
            Topic.get_topic(
                from_target=None,
                to_target=Target.new_target(EventLoop.__name__, self.event_loop_id),
                categories=EVENT_LOOP.TOPIC.CATEGORIES.REQUEST,
                use_wild_card=True
            ),
            callback=self.on_event_loop_message,
            structure=EventLoopMessage
        )

        self.__user_data = None
        self.__user_will = None

    def __del__(self):
        if self.__client is not None:
            self.end()

    @staticmethod
    def get_message(event, pid):
        return EventLoopMessage.new_data(
            time=time(),
            event=event,
            pid=pid
        )

    def set_subscriber(self, topic, callback, structure):
        def wrapped_callback(_client, _user_data, _topic, payload):
            message = Topic.unserialize(payload, structure)
            callback(_client, _user_data, _topic, message)

        self.__subscribers_lock.acquire()
        self.__subscribers[topic] = wrapped_callback
        self.__subscribers_lock.release()

    def remove_subscriber(self, topic):
        self.__subscribers_lock.acquire()
        self.__subscribers.pop(topic.get_path(use_wild_card=True))
        self.__subscribers_lock.release()
        self.__client.unsubscribe(topic.get_path(use_wild_card=True))

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
        self.__client.publish(topic=topic, payload=payload, qos=qos, retain=retain)

    def subscribe(self):
        self.__subscribers_lock.acquire()
        subscribe_topics = deepcopy(list(self.__subscribers.keys()))
        self.__subscribers_lock.release()

        for topic in subscribe_topics:
            self.__client.subscribe(topic)

    def __on_connect(self, _client, _userdata, _flags, response_code):
        if response_code == 0:
            self.subscribe()
        else:
            logger.warning('connect status {0}'.format(response_code))

    def on_event_loop_message(self, _client, _userdata, topic, message):
        if message.event == EVENT_LOOP.STATE.START:
            pass
        if message.event == EVENT_LOOP.ACTION.KILL:
            self.end()
        if message.event == EVENT_LOOP.ACTION.CHECK:
            self.__check(topic)

    def __on_message(self, client, userdata, message_data):
        try:
            payload = message_data.payload.decode("utf-8")
            self.__subscribers_lock.acquire()
            for topic, on_message_callback in self.__subscribers.items():
                if Topic.compare_topics(topic, message_data.topic):
                    on_message_callback(client, userdata, message_data.topic, payload)
            self.__subscribers_lock.release()
        except KeyboardInterrupt:
            pass
        except:
            logger.error(traceback.format_exc())
        finally:
            return True

    def ssl_setting(self, ca_path, client_path, key_path):
        self.__client.tls_set(
            ca_path,
            certfile=client_path,
            keyfile=key_path,
            tls_version=PROTOCOL_TLSv1_2
        )
        self.__client.tls_insecure_set(True)

    def connect(self, host, port, ca_path=None, client_path=None, key_path=None):
        self.__client = mqtt.Client(protocol=mqtt.MQTTv311, userdata=self.__user_data)

        if ca_path is not None and client_path is not None and key_path is not None:
            self.ssl_setting(ca_path, client_path, key_path)

        will = self.__user_will
        if will is None:
            event_loop_message = EventLoop.get_message(EVENT_LOOP.STATE.WILL, self.__pid)
            payload = Topic.serialize(event_loop_message)
            will = {"topic": self.__pub_topic, "payload": payload}
        self.__client.will_set(will["topic"], payload=will["payload"], qos=2, retain=False)

        self.__client.on_connect = self.__on_connect
        self.__client.on_message = self.__on_message
        self.__client.connect(host=host, port=port, keepalive=EVENT_LOOP.KEEP_ALIVE)

    def start(self, host="localhost", port=1883, ca_path=None, client_path=None, key_path=None):
        try:
            self.connect(host, port, ca_path=ca_path, client_path=client_path, key_path=key_path)

            event_loop_message = EventLoop.get_message(EVENT_LOOP.STATE.START, self.__pid)
            payload = Topic.serialize(event_loop_message)
            self.publish(self.__pub_topic, payload)

            if self.__main_loop is None:
                self.__client.loop_forever()
            else:
                self.__client.loop_start()
                self.__main_loop()
        except KeyboardInterrupt:
            pass
        except:
            logger.error(traceback.format_exc())
        else:
            pass
        finally:
            self.end()

    def end(self):
        event_loop_message = EventLoop.get_message(EVENT_LOOP.STATE.DISCONNECT, self.__pid)
        payload = Topic.serialize(event_loop_message)
        self.publish(self.__pub_topic, payload)

        if self.__main_loop is not None:
            self.__client.loop_stop()
        self.__client.disconnect()
        self.__client = None
        os.kill(self.__pid, SIGKILL)

    def __check(self, sub_topic):
        event_loop_message = EventLoop.get_message(EVENT_LOOP.RESPONSE.OK, self.__pid)
        payload = Topic.serialize(event_loop_message)
        response_topic = sub_topic.get_response_topic(sub_topic, self.target)
        self.publish(response_topic, payload)

    def get_pid(self):
        return self.__pid
