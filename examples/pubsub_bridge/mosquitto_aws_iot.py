#!/usr/bin/env python
# coding: utf-8

import traceback
import multiprocessing
from argparse import ArgumentParser
from time import sleep
from uuid import uuid4 as uuid

from ams import logger
from ams.clients import get_pubsub_client_class


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
        pubsub_client.set_args_of_connect()
        return pubsub_client
    except ImportError:
        traceback.print_exc()
        return None


def publish_to(_client, user_data, topic, message):
    user_data["client"].publish(topic, message)


if __name__ == '__main__':

    parser = ArgumentParser()
    parser.add_argument(
        "-FCT", "--from_broker_type", type=str, required=True, help="from broker type", 
        choices=["mosquitto", "aws_iot"])
    parser.add_argument(
        "-TCT", "--to_broker_type", type=str, required=True, help="to broker type",
        choices=["mosquitto", "aws_iot"])

    parser.add_argument("-FH", "--from_host", type=str, required=True, help="from host")
    parser.add_argument("-TH", "--to_host", type=str, required=True, help="to host")

    parser.add_argument("-FP", "--from_port", type=int, required=True, help="from port")
    parser.add_argument("-TP", "--to_port", type=int, required=True, help="to port")

    parser.add_argument("-T", "--topic", type=str, required=True, help="topic")

    parser.add_argument(
        "-CAP", "--path_ca", type=str, default="./key_dev/root-CA.crt", help="root ca file path")
    parser.add_argument(
        "-PKP", "--path_private_key", type=str, default="./key_dev/dev-autoware.private.key",
        help="private key file path")
    parser.add_argument(
        "-CP", "--path_cert", type=str, default="./key_dev/dev-autoware.cert.pem",
        help="certificate file path")
    args = parser.parse_args()

    if args.from_broker_type == "mosquitto":
        paho_client = get_paho_client(args.from_host, args.from_port)
        user_data = {
            "client": get_aws_iot_client(
            args.to_host, args.to_port, args.path_ca, args.path_private_key, args.path_cert)
        }
        user_data["client"].connect()
        paho_client.subscribe(topic=args.topic, callback=publish_to, user_data=user_data)
        paho_client.connect()
    else:
        aws_iot_client = get_aws_iot_client(
            args.from_host, args.from_port, args.path_ca, args.path_private_key, args.path_cert)
        user_data = {
            "client": get_paho_client(args.to_host, args.to_port)
        }
        user_data["client"].connect()
        aws_iot_client.subscribe(topic=args.topic, callback=publish_to, user_data=user_data)
        aws_iot_client.connect()

    while True:
        sleep(2)
