ls#!/usr/bin/env python
# coding: utf-8

import unittest
import traceback
import os

from time import time, sleep
from uuid import uuid4 as uuid

from ams.clients import get_pubsub_client_class
from tests.config.env import env


def generate_paho_client(host, port):
    try:
        import paho.mqtt.client
        PUBSUBClient = get_pubsub_client_class(paho.mqtt.client)
        pubsub_client = PUBSUBClient()
        pubsub_client.set_args_of_Client()
        pubsub_client.set_args_of_connect(host=host, port=port)
        return pubsub_client
    except ModuleNotFoundError:
        return None
    except ImportError:
        return None
    except Exception as e:
        print(e)


def generate_aws_iot_client(host, port, path_ca, path_private_key, path_cert):
    try:
        import AWSIoTPythonSDK.MQTTLib
        if not os.path.exists(path_ca):
            print("not found: {}".format(path_ca))
            return None
        if not os.path.exists(path_private_key):
            print("not found: {}".format(path_private_key))
            return None
        if not os.path.exists(path_cert):
            print("not found: {}".format(path_cert))
            return None
        PUBSUBClient = get_pubsub_client_class(AWSIoTPythonSDK.MQTTLib)
        pubsub_client = PUBSUBClient(num_of_clients=6)
        pubsub_client.set_args_of_AWSIoTMQTTClient(str(uuid()))
        pubsub_client.set_args_of_configureEndpoint(host, port)
        pubsub_client.set_args_of_configureCredentials(
            CAFilePath=path_ca, KeyPath=path_private_key, CertificatePath=path_cert)
        pubsub_client.set_args_of_configureMQTTOperationTimeout(30)
        pubsub_client.set_args_of_configureAutoReconnectBackoffTime(1, 128, 10)
        pubsub_client.set_args_of_connect()
        return pubsub_client
    except ModuleNotFoundError:
        return None
    except ImportError:
        return None
    except Exception as e:
        print(e)


def generate_ros_client():
    try:
        import rospy
        PUBSUBClient = get_pubsub_client_class(rospy)
        pubsub_client = PUBSUBClient()
        return pubsub_client
    except ModuleNotFoundError:
        return None
    except ImportError:
        return None
    except Exception as e:
        print(e)


def on_message_set(_client, user_data, _topic, message):
    # print("message: {}".format(message))
    user_data.message = message


class Test(unittest.TestCase):

    def __init__(self, method_name):

        super(Test, self).__init__(method_name)
        self.message = None
        self.paho_client = generate_paho_client("localhost", 1883)
        self.aws_iot_client = generate_aws_iot_client(
            env["AWS_IOT_ENDPOINT"], 8883,
            "./tests/config/aws_iot_keys/root-CA.crt",
            "./tests/config/aws_iot_keys/private.key",
            "./tests/config/aws_iot_keys/cert.pem"
        )
        self.ros_client = generate_ros_client()

    def test_subscribe(self):
        if self.paho_client is not None:
            topic = "/test/subscribe/paho"
            expected = "paho_message"
            timeout = 5.0
            self.message = None
            self.paho_client.subscribe(topic, on_message_set, qos=2, user_data=self, structure=None)
            self.paho_client.connect()
            sleep(1)
            self.paho_client.publish(topic, expected)
            ts = time()
            while self.message is None:
                if timeout < time() - ts:
                    break
                sleep(1)
            # print("paho pubsub time: {} [sec]".format(time() - ts))
            self.assertEqual(expected, self.message)

        if self.aws_iot_client is not None:
            topic = "/test/subscribe/aws_iot"
            expected = "aws_iot_message"
            timeout = 5.0
            self.message = None
            self.aws_iot_client.subscribe(topic, on_message_set, qos=1, user_data=self, structure=None)
            self.aws_iot_client.connect()
            sleep(1)
            self.aws_iot_client.publish(topic, expected)
            ts = time()
            while self.message is None:
                if timeout < time() - ts:
                    break
                sleep(1)
            # print("aws_iot pubsub time: {} [sec]".format(time() - ts))
            self.assertEqual(expected, self.message)

        if self.ros_client is not None:
            from std_msg.msgs import String
            topic = "/test/subscribe/ros"
            expected = "ros_message"
            timeout = 1.0
            self.message = None
            self.ros_client.subscribe(topic, on_message_set, qos=0, user_data=self, structure=String)
            self.ros_client.connect()
            sleep(1)
            self.ros_client.publish(topic, expected)
            ts = time()
            while self.message is None:
                if timeout < time() - ts:
                    break
                sleep(1)
            print("ros pubsub time: {} [sec]".format(time() - ts))
            self.assertEqual(expected, self.message)

    def test_connect(self):
        if self.paho_client is not None:
            self.paho_client.connect()

        if self.aws_iot_client is not None:
            self.aws_iot_client.connect()

        if self.ros_client is not None:
            self.ros_client.connect()

    def test_publish(self):
        if self.paho_client is not None:
            self.paho_client.connect()
            self.paho_client.publish("/test/publish/paho", "message")

        if self.aws_iot_client is not None:
            self.aws_iot_client.connect()
            self.aws_iot_client.publish("/test/publish/aws_iot", "message")

        if self.ros_client is not None:
            from std_msg.msgs import String
            self.ros_client.connect()
            self.ros_client.publish("/test/publish/ros", "message", String)
