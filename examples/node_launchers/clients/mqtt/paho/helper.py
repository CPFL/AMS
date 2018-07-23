#!/usr/bin/env python
# coding: utf-8

import paho.mqtt.client
from ams import get_ams_mqtt_client_class


def get_mqtt_client(args):
    MQTTClient = get_ams_mqtt_client_class(paho.mqtt.client)
    mqtt_client = MQTTClient()
    mqtt_client.set_args_of_Client()
    mqtt_client.set_args_of_connect(host=args.mqtt_host, port=args.mqtt_port)
    return mqtt_client
