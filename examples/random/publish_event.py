#!/usr/bin/env python
# coding: utf-8

from argparse import ArgumentParser

import paho.mqtt.client

from ams import VERSION
from ams.helpers import Topic, Event
from ams.clients import get_pubsub_client_class
from ams.nodes import Dispatcher


if __name__ == '__main__':

    parser = ArgumentParser()
    parser.add_argument("-EN", "--event_name", type=str, required=True, help="event_name")
    parser.add_argument("-RC", "--route_code", type=str, default=None, help="route_code")
    parser.add_argument("-MH", "--mqtt_host", type=str, default="localhost", help="mqtt host")
    parser.add_argument("-MP", "--mqtt_port", type=int, default=1883, help="mqtt port")
    parser.add_argument("-MT", "--mqtt_topic", type=str, required=True, help="mqtt topic")
    args = parser.parse_args()

    Topic.domain = Topic.get_domain(args.mqtt_topic)

    PubSubClient = get_pubsub_client_class(paho.mqtt.client)
    pubsub_client = PubSubClient()
    pubsub_client.set_args_of_Client()
    pubsub_client.set_args_of_connect(host=args.mqtt_host, port=args.mqtt_port)
    pubsub_client.connect()
    pubsub_client.publish(
        args.mqtt_topic,
        Dispatcher.Message.Event.new_data(**{
            "header": {
                "id": Event.get_id(),
                "time": Event.get_time(),
                "version": VERSION
            },
            "body": {
                "target": Topic.get_to_target(args.mqtt_topic),
                "event": Event.new_event(
                    targets=[Topic.get_to_target(args.mqtt_topic)],
                    name=args.event_name,
                    route_code=args.route_code
                )
            }
        }),
        Dispatcher.Message.Event
    )
