#!/usr/bin/env python
# coding: utf-8

from copy import copy
from multiprocessing import Manager
from time import sleep
from _ssl import PROTOCOL_TLSv1_2

import boto3

from ams import AttrDict, logger
from ams.helpers import Topic
from ams.structures import CLIENT


class PubSubClient(object):

    CONST = CLIENT.PUBSUB.BASE_CLIENTS.AWS_IOT_BOTO3

    def __init__(self):
        super().__init__()
        self.__client = None

    def subscribe(self, topic, callback, qos=0, user_data=None, structure=None):
        logger.warning("Not support this method.")

    def connect(self):
        self.__client = boto3.client(CLIENT.PUBSUB.BASE_CLIENTS.AWS_IOT_BOTO3.CLIENT_TYPE)

    def publish(self, topic, message, qos=0, retain=False, wait=False):
        self.__client.publish(topic=topic, qos=qos, payload=Topic.serialize(message))

    def unsubscribe(self, topic):
        logger.warning("Not support this method.")

    def disconnect(self):
        logger.warning("Not support this method.")
