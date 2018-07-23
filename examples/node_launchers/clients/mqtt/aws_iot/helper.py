#!/usr/bin/env python
# coding: utf-8

from uuid import uuid4 as uuid

import AWSIoTPythonSDK.MQTTLib

from ams import get_ams_mqtt_client_class


def get_mqtt_client(args):
    MQTTClient = get_ams_mqtt_client_class(AWSIoTPythonSDK.MQTTLib)
    mqtt_client = MQTTClient()
    mqtt_client.set_args_of_AWSIoTMQTTClient(str(uuid()))
    mqtt_client.set_args_of_configureEndpoint(args.mqtt_host, args.mqtt_port)
    mqtt_client.set_args_of_configureCredentials(
        CAFilePath=args.path_ca, KeyPath=args.path_private_key, CertificatePath=args.path_cert)
    mqtt_client.set_args_of_connect()
    return mqtt_client
