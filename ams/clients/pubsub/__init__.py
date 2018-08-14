#!/usr/bin/env python
# coding: utf-8

from ams.structures import CLIENT


def get_client_class(base_pubsub_client_module):

    if base_pubsub_client_module.__name__ == CLIENT.PUBSUB.BASE_CLIENTS.PAHO_MQTT.MODULE_NAME:
        from ams.clients.pubsub.paho import PubSubClient

    elif base_pubsub_client_module.__name__ == CLIENT.PUBSUB.BASE_CLIENTS.AWS_IOT_SDK.MODULE_NAME:
        from ams.clients.pubsub.aws_iot_sdk import PubSubClient

    elif base_pubsub_client_module.__name__ == CLIENT.PUBSUB.BASE_CLIENTS.AWS_IOT_BOTO3.MODULE_NAME:
        from ams.clients.pubsub.aws_iot_boto3 import PubSubClient

    else:
        raise AttributeError("Unknown module {}".format(base_pubsub_client_module.__name__))

    return PubSubClient
