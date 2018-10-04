#!/usr/bin/env python
# coding: utf-8

from pprint import pformat
import json

import boto3

from ams import logger
from ams.helpers import Topic
from ams.structures import CLIENT


class PubSubClient(object):

    CONST = CLIENT.PUBSUB.BASE_CLIENTS.AWS_IOT_BOTO3

    def __init__(self):
        super().__init__()
        self.__client = None
        self.__dumps = json.dumps
        self.__loads = json.loads

    def set_dumps(self, dumps):
        self.__dumps = dumps

    def set_loads(self, loads):
        self.__loads = loads

    def subscribe(self, topic, callback, qos=0, user_data=None, structure=None):
        logger.warning("Not support this method.")

    def connect(self):
        self.__client = boto3.client(CLIENT.PUBSUB.BASE_CLIENTS.AWS_IOT_BOTO3.CLIENT_TYPE)

    def publish(self, topic, message, structure=None, qos=0, retain=False, wait=False):
        if structure is not None:
            if not structure.validate_data(message):
                logger.error(pformat({"errors": structure.get_errors(), "message": message}))
                raise ValueError
        self.__client.publish(topic=topic, qos=qos, payload=Topic.serialize(message, self.__dumps))

    def unsubscribe(self, topic):
        logger.warning("Not support this method.")

    def disconnect(self):
        logger.warning("Not support this method.")
