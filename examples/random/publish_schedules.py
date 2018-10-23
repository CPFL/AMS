#!/usr/bin/env python
# coding: utf-8

import json
from argparse import ArgumentParser
import multiprocessing

import paho.mqtt.client

from ams import logger
from ams.helpers import Hook, Publisher, Topic
from ams.clients import get_pubsub_client_class, get_kvs_client_class
from ams.nodes import Dispatcher


if __name__ == '__main__':

    parser = ArgumentParser()
    parser.add_argument("-DIFP", "--dispatcher_initials_file_path", type=str, required=True, help="dispatcher_initials_file_path")
    parser.add_argument("-MH", "--mqtt_host", type=str, default="localhost", help="mqtt host")
    parser.add_argument("-MP", "--mqtt_port", type=int, default=1883, help="mqtt port")
    parser.add_argument("-MT", "--mqtt_topic", type=str, required=True, help="mqtt topic")
    args = parser.parse_args()

    with open(args.dispatcher_initials_file_path, "r") as f:
        transportation_config = Dispatcher.TransportationConfig.new_data(**json.load(f)["transportation_config"])

    schedules = Hook.generate_vehicle_schedules(transportation_config)

    Topic.domain = Topic.get_domain(args.mqtt_topic)
    # logger.pp(schedules)

    PubSubClient = get_pubsub_client_class(paho.mqtt.client)
    pubsub_client = PubSubClient()
    pubsub_client.set_args_of_Client()
    pubsub_client.set_args_of_connect(host=args.mqtt_host, port=args.mqtt_port)
    pubsub_client.connect()
    Publisher.publish_schedules_message(
        pubsub_client,
        Topic.get_from_target(args.mqtt_topic),
        Topic.get_to_target(args.mqtt_topic),
        schedules
    )



# {
#     "topic": "/sim/vehicle/v_random1/autoware/a_random1/route_code",
#     "route_code": "10472:10471>9686:9686:9686<9673:9673:9673>9988:10333",
#     "topic2": "/sim/vehicle/v_random1/autoware/a_random1/state_cmd"
# }