#!/usr/bin/env python
# coding: utf-8

from copy import copy
from multiprocessing import Manager
from _ssl import PROTOCOL_TLSv1_2

from paho.mqtt.client import Client

from ams import AttrDict, logger
from ams.helpers import Topic
from ams.structures import CLIENT


class ArgsSetters(object):

    CONST = CLIENT.PUBSUB.BASE_CLIENTS.PAHO_MQTT

    def __init__(self):
        self.args = AttrDict()

    def set_args_of_Client(
            self, client_id="", clean_session=True, userdata=None,
            protocol=CONST.DEFAULT_PROTOCOL, transport="tcp"):
        self.args.client = copy(locals())
        self.args.client.pop("self")

    def set_args_of_tls_set(
            self, ca_certs=None, certfile=None, keyfile=None, cert_reqs=None, tls_version=PROTOCOL_TLSv1_2,
            ciphers=None):
        self.args.tls_set = copy(locals())
        self.args.tls_set.pop("self")

    def set_args_of_tls_insecure_set(self, value):
        self.args.tls_insecure_set = copy(locals())
        self.args.tls_insecure_set.pop("self")

    def set_args_of_will_set(self, topic, payload=None, qos=0, retain=False):
        self.args.will_set = copy(locals())
        self.args.will_set.pop("self")

    def set_args_of_connect(self, host, port=1883, keepalive=60, bind_address=""):
        self.args.connect = copy(locals())
        self.args.connect.pop("self")


def set_on_message_and_connect(client, args, subscribers, subscribers_lock):
    def on_message(_client, _userdata, message_data):
        payload = message_data.payload.decode("utf-8")
        subscribers_lock.acquire()
        for topic, subscriber in subscribers.items():
            if Topic.compare_topics(topic, message_data.topic):
                subscriber["callback"](_client, subscriber["user_data"], message_data.topic, payload)
        subscribers_lock.release()

    client.on_message = on_message
    client.connect(**args.connect)


class PubSubClient(ArgsSetters):
    def __init__(self):
        super().__init__()
        self.__manager = Manager()
        self.__subscribers = {}
        self.__subscribers_lock = self.__manager.Lock()
        self.__client = None

    def subscribe(self, topic, callback, qos=0, user_data=None, structure=None):
        def wrapped_callback(_client, _user_data, _topic, _payload):
            message = Topic.unserialize(_payload, structure)
            callback(_client, _user_data, _topic, message)

        self.__subscribers_lock.acquire()
        self.__subscribers[topic] = {
            "topic": topic,
            "callback": wrapped_callback,
            "qos": qos,
            "user_data": user_data
        }
        self.__subscribers_lock.release()

        if self.__client is not None:
            set_on_message_and_connect(
                self.__client, self.args, self.__subscribers, self.__subscribers_lock)

    def connect(self):
        def on_connect(client, _userdata, _flags, response_code):
            if response_code == 0:
                for topic, subscriber in self.__subscribers.items():
                    client.subscribe(topic=topic, qos=subscriber["qos"])
            else:
                logger.warning('connect status {0}'.format(response_code))

        self.__client = Client(**self.args.client)
        if "tls_set" in self.args.keys():
            self.__client.tls_set(**self.args.tls_set)
        if "tls_insecure_set" in self.args.keys():
            self.__client.tls_insecure_set(**self.args.tls_insecure_set)
        if "will_set" in self.args.keys():
            self.__client.will_set(**self.args.will_set)
        self.__client.on_connect = on_connect
        set_on_message_and_connect(
            self.__client, self.args, self.__subscribers, self.__subscribers_lock)
        self.__client.loop_start()

    def publish(self, topic, message, qos=0, retain=False):
        self.__client.publish(topic, Topic.serialize(message), qos, retain)

    def unsubscribe(self, topic):
        self.__client.unsubscribe(topic)
        self.__subscribers_lock.acquire()
        self.__subscribers.pop(topic)
        self.__subscribers_lock.release()

    def disconnect(self):
        self.__client.disconnect()
