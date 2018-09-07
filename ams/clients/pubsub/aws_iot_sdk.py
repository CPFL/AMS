#!/usr/bin/env python
# coding: utf-8

from copy import copy
from multiprocessing import Manager
from _ssl import PROTOCOL_TLSv1_2

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

    def set_args_of_connect(self, keepAliveIntervalSecond=600):
        self.args.connect = copy(locals())
        self.args.connect.pop("self")


def set_on_message(_client, subscriber, subscribers_lock):
    def on_message(client, _user_data, message_data):
        payload = message_data.payload.decode("utf-8")
        subscribers_lock.acquire()
        subscriber["callback"](client, subscriber["user_data"], message_data.topic, payload)
        subscribers_lock.release()

    _client.subscribe(subscriber["topic"], QoS=subscriber["qos"], callback=on_message)


class PubSubClient(ArgsSetters):
    def __init__(self):
        super(PubSubClient, self).__init__()
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
            set_on_message(self.__client, self.__subscribers[topic], self.__subscribers_lock)

    def connect(self):
        self.__client = AWSIoTMQTTClient(**self.args.aws_iot_mqtt_client)
        self.__client.configureEndpoint(**self.args.configure_endpoint)
        if "configure_credentials" in self.args.keys():
            self.__client.configureCredentials(**self.args.configure_credentials)
        elif "configure_iam_credentials" in self.args.keys():
            self.__client.configureIAMCredentials(**self.args.configure_iam_credentials)
        if "configure_last_will" in self.args.keys():
            self.__client.configureLastWill(**self.args.configure_last_will)
        if "configure_mqtt_operation_timeout" in self.args.keys():
            self.__client.configureMQTTOperationTimeout(**self.args.configure_mqtt_operation_timeout)
        for topic, subscriber in self.__subscribers.items():
            set_on_message(self.__client, self.__subscribers[topic], self.__subscribers_lock)
        self.__client.connect(**self.args.connect)

    def publish(self, topic, message, qos=0, retain=False, wait=False):
        if wait:
            if qos == 0:
                qos = 1
            self.__client.publish(topic, Topic.serialize(message), QoS=qos)
        else:
            self.__client.publishAsync(topic, Topic.serialize(message), QoS=qos)

    def unsubscribe(self, topic):
        self.__client.unsubscribe(topic)
        self.__subscribers_lock.acquire()
        self.__subscribers.pop(topic)
        self.__subscribers_lock.release()

    def disconnect(self):
        self.__client.disconnect()
