#!/usr/bin/env python
# coding: utf-8

import traceback
from uuid import uuid4 as uuid

from ams import logger
from ams.clients import get_kvs_client_class, get_pubsub_client_class


def get_manager_client():
    import multiprocessing
    KVSClient = get_kvs_client_class(multiprocessing)
    kvs_client = KVSClient()
    return kvs_client


def get_redis_client(host):
    try:
        import redis
        KVSClient = get_kvs_client_class(redis)
        kvs_client = KVSClient()
        kvs_client.set_args_of_ConnectionPool(host=host)
        kvs_client.set_args_of_StrictRedis()
        return kvs_client
    except ImportError:
        traceback.print_exc()
        return None


def get_paho_client(host, port):
    try:
        import paho.mqtt.client
        PUBSUBClient = get_pubsub_client_class(paho.mqtt.client)
        pubsub_client = PUBSUBClient()
        pubsub_client.set_args_of_Client()
        pubsub_client.set_args_of_connect(host=host, port=port)
        return pubsub_client
    except ImportError:
        traceback.print_exc()
        return None


def get_aws_iot_client(host, port, path_ca, path_private_key, path_cert):
    try:
        import AWSIoTPythonSDK.MQTTLib
        PUBSUBClient = get_pubsub_client_class(AWSIoTPythonSDK.MQTTLib)
        pubsub_client = PUBSUBClient()
        pubsub_client.set_args_of_AWSIoTMQTTClient(str(uuid()))
        pubsub_client.set_args_of_configureEndpoint(host, port)
        pubsub_client.set_args_of_configureCredentials(
            CAFilePath=path_ca, KeyPath=path_private_key, CertificatePath=path_cert)
        pubsub_client.set_args_of_configureMQTTOperationTimeout(30)
        pubsub_client.set_args_of_connect()
        return pubsub_client
    except ImportError:
        traceback.print_exc()
        return None


def get_ros_client():
    try:
        import rospy
        PUBSUBClient = get_pubsub_client_class(rospy)
        pubsub_client = PUBSUBClient()
        return pubsub_client
    except ImportError:
        traceback.print_exc()
        return None
