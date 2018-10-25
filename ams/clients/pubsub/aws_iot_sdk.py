#!/usr/bin/env python
# coding: utf-8

from copy import copy
from multiprocessing import Manager
from pprint import pformat
from time import time
from uuid import uuid4
import json

from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient

from ams import AttrDict, logger
from ams.helpers import Topic
from ams.structures import CLIENT


class ArgsSetters(object):

    CONST = CLIENT.PUBSUB.BASE_CLIENTS.AWS_IOT_SDK

    def __init__(self):
        self.args = AttrDict()

    def set_args_of_AWSIoTMQTTClient(
            self, clientID, protocolType=CONST.DEFAULT_PROTOCOL, useWebsocket=False, cleanSession=True):
        self.args.aws_iot_mqtt_client = copy(locals())
        self.args.aws_iot_mqtt_client.pop("self")

    def set_args_of_configureEndpoint(self, hostName, portNumber):
        self.args.configure_endpoint = copy(locals())
        self.args.configure_endpoint.pop("self")

    def set_args_of_configureCredentials(self, CAFilePath, KeyPath="", CertificatePath=""):
        self.args.configure_credentials = copy(locals())
        self.args.configure_credentials.pop("self")

    def set_args_of_configureIAMCredentials(self, AWSAccessKeyID, AWSSecretAccessKey, AWSSessionToken=""):
        self.args.configure_iam_credentials = copy(locals())
        self.args.configure_iam_credentials.pop("self")

    def set_args_of_configureLastWill(self, topic, payload, QoS, retain=False):
        self.args.configure_last_will = copy(locals())
        self.args.configure_last_will.pop("self")

    def set_args_of_configureMQTTOperationTimeout(self, timeoutSecond=5):
        self.args.configure_mqtt_operation_timeout = copy(locals())
        self.args.configure_mqtt_operation_timeout.pop("self")

    def set_args_of_configureAutoReconnectBackoffTime(
            self, baseReconnectQuietTimeSecond, maxReconnectQuietTimeSecond, stableConnectionTimeSecond):
        self.args.configure_auto_reconnect_back_off_time = copy(locals())
        self.args.configure_auto_reconnect_back_off_time.pop("self")

    def set_args_of_connect(self, keepAliveIntervalSecond=600):
        self.args.connect = copy(locals())
        self.args.connect.pop("self")


def get_enable_pool(pool):
    return dict(filter(lambda x: x[1]["instance"] is not None, pool.items()))


def get_least_subscribers_client(pool):
    return min(pool.values(), key=lambda x: x["num_of_subscribers"])


def get_oldest_client(pool):
    return min(pool.values(), key=lambda x: x["last_time"])


def set_on_message(pool, subscriber, subscribers_lock):
    _client = get_least_subscribers_client(pool)

    def on_message(client, _user_data, message_data):
        _client["last_time"] = time()
        subscribers_lock.acquire()
        subscriber["callback"](client, subscriber["user_data"], message_data.topic, message_data.payload)
        subscribers_lock.release()

    _client["instance"].subscribe(subscriber["topic"], QoS=subscriber["qos"], callback=on_message)
    _client["num_of_subscribers"] += 1
    subscriber["client_id"] = _client["id"]


class PubSubClient(ArgsSetters):
    def __init__(self, num_of_clients=1):
        super(PubSubClient, self).__init__()
        self.__manager = Manager()
        self.__subscribers = {}
        self.__subscribers_lock = self.__manager.Lock()
        self.__client_pool = {}
        for i in range(num_of_clients):
            client_id = str(uuid4())
            self.__client_pool[client_id] = {
                "id": client_id,
                "num_of_subscribers": 0,
                "last_time": time(),
                "instance": None
            }
        self.__dumps = json.dumps
        self.__loads = json.loads

    def set_dumps(self, dumps):
        self.__dumps = dumps

    def set_loads(self, loads):
        self.__loads = loads

    def subscribe(self, topic, callback, qos=0, user_data=None, structure=None):
        loads = self.__loads

        def wrapped_callback(_client, _user_data, _topic, _payload):
            message = Topic.deserialize(_payload, structure, loads)
            callback(_client, _user_data, _topic, message)

        self.__subscribers_lock.acquire()
        self.__subscribers[topic] = {
            "topic": topic,
            "callback": wrapped_callback,
            "qos": qos,
            "user_data": user_data,
            "client_id": None
        }
        self.__subscribers_lock.release()

        pool = get_enable_pool(self.__client_pool)
        if 0 < len(pool):
            set_on_message(pool, self.__subscribers[topic], self.__subscribers_lock)

    def connect(self):
        for client_id, client in self.__client_pool.items():
            self.args.aws_iot_mqtt_client["clientID"] = client_id
            client["instance"] = AWSIoTMQTTClient(**self.args.aws_iot_mqtt_client)
            client["instance"].configureEndpoint(**self.args.configure_endpoint)
            if "configure_credentials" in self.args.keys():
                client["instance"].configureCredentials(**self.args.configure_credentials)
            elif "configure_iam_credentials" in self.args.keys():
                client["instance"].configureIAMCredentials(**self.args.configure_iam_credentials)
            if "configure_last_will" in self.args.keys():
                client["instance"].configureLastWill(**self.args.configure_last_will)
            if "configure_mqtt_operation_timeout" in self.args.keys():
                client["instance"].configureMQTTOperationTimeout(**self.args.configure_mqtt_operation_timeout)
            if "configure_auto_reconnect_back_off_time" in self.args.keys():
                client["instance"].configureAutoReconnectBackoffTime(**self.args.configure_auto_reconnect_back_off_time)

        for topic, subscriber in self.__subscribers.items():
            set_on_message(self.__client_pool, self.__subscribers[topic], self.__subscribers_lock)

        for client_id, client in self.__client_pool.items():
            client["instance"].connect(**self.args.connect)

    def publish(self, topic, message, structure=None, qos=0, retain=False, wait=False):
        if structure is not None:
            if not structure.validate_data(message):
                logger.error(pformat({"errors": structure.get_errors(), "message": message}))
                raise ValueError

        pool = get_enable_pool(self.__client_pool)
        if 0 < len(pool):
            client = get_oldest_client(pool)
            client["last_time"] = time()
            if wait:
                if qos == 0:
                    qos = 1
                client["instance"].publish(topic, Topic.serialize(message, self.__dumps), QoS=qos)
            else:
                client["instance"].publishAsync(topic, Topic.serialize(message, self.__dumps), QoS=qos)

    def unsubscribe(self, topic):
        self.__subscribers_lock.acquire()
        subscriber = self.__subscribers.pop(topic)
        self.__subscribers_lock.release()
        if subscriber["client_id"] in self.__client_pool.keys():
            self.__client_pool[subscriber["client_id"]]["instance"].unsubscribe(topic)
            self.__client_pool[subscriber["client_id"]]["num_of_subscribers"] -= 1

    def disconnect(self):
        for client in get_enable_pool(self.__client_pool):
            client["instance"].disconnect()
