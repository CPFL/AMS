#!/usr/bin/env python
# coding: utf-8

import json
from argparse import ArgumentParser

import paho.mqtt.client

from ams.helpers import Hook, Publisher, Topic
from ams.clients import get_pubsub_client_class
from ams.nodes import Dispatcher


if __name__ == '__main__':

    parser = ArgumentParser()
    parser.add_argument(
        "-DIFP", "--dispatcher_initials_file_path", type=str, required=True, help="dispatcher_initials_file_path")
    parser.add_argument("-MH", "--mqtt_host", type=str, default="localhost", help="mqtt host")
    parser.add_argument("-MP", "--mqtt_port", type=int, default=1883, help="mqtt port")
    parser.add_argument("-MT", "--mqtt_topic", type=str, required=True, help="mqtt topic")
    args = parser.parse_args()

    with open(args.dispatcher_initials_file_path, "r") as f:
        transportation_config = Dispatcher.TransportationConfig.new_data(**json.load(f)["transportation_config"])

    schedule = Hook.generate_vehicle_schedule(transportation_config)

    Topic.domain = Topic.get_domain(args.mqtt_topic)

    PubSubClient = get_pubsub_client_class(paho.mqtt.client)
    pubsub_client = PubSubClient()
    pubsub_client.set_args_of_Client()
    pubsub_client.set_args_of_connect(host=args.mqtt_host, port=args.mqtt_port)
    pubsub_client.connect()
    print("publish schedule: {}".format(schedule.id))
    Publisher.publish_schedule_message(
        pubsub_client,
        Topic.get_from_target(args.mqtt_topic),
        Topic.get_to_target(args.mqtt_topic),
        schedule
    )
