#!/usr/bin/env python
# coding: utf-8

from uuid import uuid4 as uuid

import AWSIoTPythonSDK.MQTTLib

from ams import get_ams_mqtt_client_class


def get_mqtt_client(host, port, path_ca, path_private_key, path_cert):
    MQTTClient = get_ams_mqtt_client_class(AWSIoTPythonSDK.MQTTLib)
    mqtt_client = MQTTClient()
    mqtt_client.set_args_of_AWSIoTMQTTClient(str(uuid()))
    mqtt_client.set_args_of_configureEndpoint(host, port)
    mqtt_client.set_args_of_configureCredentials(
        CAFilePath=path_ca, KeyPath=path_private_key, CertificatePath=path_cert)
    mqtt_client.set_args_of_connect()
    return mqtt_client
