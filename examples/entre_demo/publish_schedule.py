#!/usr/bin/env python
# coding: utf-8

import json
from argparse import ArgumentParser

import paho.mqtt.client

from ams.helpers import Hook, Publisher, Topic
from ams.clients import get_pubsub_client_class, get_kvs_client_class
from ams.nodes import Dispatcher


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


if __name__ == '__main__':

    parser = ArgumentParser()
    parser.add_argument(
        "-DIFP", "--dispatcher_initials_file_path", type=str, required=True, help="dispatcher_initials_file_path")
    parser.add_argument("-MH", "--mqtt_host", type=str, default="localhost", help="mqtt host")
    parser.add_argument("-MP", "--mqtt_port", type=int, default=1883, help="mqtt port")
    parser.add_argument("-MT", "--mqtt_topic", type=str, required=True, help="mqtt topic")
    parser.add_argument("-SID", "--schedule_id", type=str, required=True, help="schedule id")
    args = parser.parse_args()

    with open(args.dispatcher_initials_file_path, "r") as f:
        config = Dispatcher.Config.new_data(**json.load(f)["config"])

    target_dispatcher = Topic.get_from_target(args.mqtt_topic)
    target_vehicle = Topic.get_to_target(args.mqtt_topic)

    kvs_client = get_manager_client()
    # kvs_client = get_redis_client()
    kvs_client.connect()
    Hook.set_config(kvs_client, target_dispatcher, config)
    Hook.generate_vehicle_schedule_from_config(
        kvs_client, target_dispatcher, target_vehicle, args.schedule_id)

    Topic.domain = Topic.get_domain(args.mqtt_topic)

    pubsub_client = get_pubsub_client_class(paho.mqtt.client)()
    pubsub_client.set_args_of_Client()
    pubsub_client.set_args_of_connect(host=args.mqtt_host, port=args.mqtt_port)
    pubsub_client.connect()
    print("publish schedule: {}".format(args.schedule_id))
    Publisher.publish_schedule_message(
        pubsub_client,
        Topic.get_from_target(args.mqtt_topic),
        Topic.get_to_target(args.mqtt_topic),
        Hook.get_generated_schedule(kvs_client, target_dispatcher, target_vehicle)
    )
