#!/usr/bin/env python
# coding: utf-8

from argparse import ArgumentParser

import paho.mqtt.client

from ams.clients import get_pubsub_client_class
from ams.nodes import Autoware


if __name__ == '__main__':

    parser = ArgumentParser()
    parser.add_argument("-MH", "--mqtt_host", type=str, default="localhost", help="mqtt host")
    parser.add_argument("-MP", "--mqtt_port", type=str, default=1883, help="mqtt port")
    parser.add_argument("-MT", "--mqtt_topic", type=str, required=True, help="mqtt topic")
    args = parser.parse_args()

    MQTTClient = get_pubsub_client_class(paho.mqtt.client)
    mqtt_client = MQTTClient()
    mqtt_client.set_args_of_Client()
    mqtt_client.set_args_of_connect(host=args.mqtt_host, port=args.mqtt_port)
    mqtt_client.connect()
    mqtt_client.publish(args.mqtt_topic, Autoware.ROSMessage.StateCMD.new_data(**{
        "data": "engage"
    }))
