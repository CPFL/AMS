#!/usr/bin/env python
# coding: utf-8

from time import time, sleep
from argparse import ArgumentParser
from uuid import uuid1 as uuid

from clients.kvs.redis.helper import get_kvs_client as get_redis_client
from clients.kvs.manager.helper import get_kvs_client as get_manager_client
from clients.kvs.helper import print_all

from clients.mqtt.aws_iot.helper import get_mqtt_client as get_aws_iot_client
from clients.mqtt.paho.helper import get_mqtt_client as get_paho_client

from ams import MapsClient
from ams.helpers import Topic
from ams.nodes import Autoware


if __name__ == '__main__':

    parser = ArgumentParser()

    parser.add_argument("-ID", "--id", type=str, default=str(uuid()), help="node id")
    parser.add_argument("-RID", "--ros_id", type=str, default=str(uuid()), help="ros id")
    parser.add_argument("-DID", "--dispatcher_id", type=str, default=str(uuid()), help="dispatcher id")
    parser.add_argument("-TD", "--topic_domain", type=str, default="ams", help="topic domain")

    parser.add_argument("-AK", "--path_activation_key", type=str,
                        default="../../res/activation.key", help="activation.key path")
    parser.add_argument("-W", "--path_waypoint_json", type=str,
                        default="../../res/waypoint.json", help="waypoint.json path")
    parser.add_argument("-A", "--path_arrow_json", type=str,
                        default="../../res/arrow.json", help="arrow.json path")

    parser.add_argument("-KH", "--kvs_host", type=str, default="localhost", help="kvs host")
    parser.add_argument("-KP", "--kvs_port", type=int, default=6379, help="kvs port")
    parser.add_argument("-KCT", "--kvs_client_type", type=str, default="manager", help="manager or redis")

    parser.add_argument("-MH", "--mqtt_host", type=str, default="localhost", help="mqtt broker host")
    parser.add_argument("-MP", "--mqtt_port", type=int, default=1883, help="mqtt broker port")
    parser.add_argument("-MCT", "--mqtt_client_type", type=str, default="paho", help="paho or aws_iot",
                        choices=["pahp", "aws_iot"])
    parser.add_argument(
        "-CAP", "--path_ca", type=str, default="./clients/aws_iot/key_dev/root-CA.crt", help="root ca file path")
    parser.add_argument(
        "-PKP", "--path_private_key", type=str, default="./clients/aws_iot/key_dev/dev-autoware.private.key",
        help="private key file path")
    parser.add_argument(
        "-CP", "--path_cert", type=str, default="./clients/aws_iot/key_dev/dev-autoware.cert.pem",
        help="certificate file path")

    args = parser.parse_args()

    with open(args.path_activation_key, "r") as f:
        activation_key = f.readline()[:-1]

    if args.kvs_client_type == "manager":
        kvs_client = get_manager_client()
    elif args.kvs_client_type == "redis":
        kvs_client = get_redis_client(args)
    else:
        raise ValueError("Unknown kvs client type: {}".format(args.kvs_client_type))

    if args.mqtt_client_type == "paho":
        mqtt_client = get_paho_client(args)
    elif args.mqtt_client_type == "aws_iot":
        mqtt_client = get_aws_iot_client(args)
    else:
        raise ValueError("Unknown mqtt client type: {}".format(args.mqtt_client_type))

    Topic.domain = args.topic_domain

    maps_client = MapsClient()
    maps_client.load_waypoint_json_file(args.path_waypoint_json)
    maps_client.load_arrow_json_file(args.path_arrow_json)

    autoware = Autoware(group=Autoware.CONST.NODE_NAME, _id=args.id)
    autoware.set_kvs_client(kvs_client)
    autoware.set_mqtt_client(mqtt_client)
    autoware.set_maps_client(maps_client)

    start_time = time()

    autoware.set_initial_config(
        target_ros_id=args.ros_id,
        target_dispatcher_id=args.dispatcher_id,
        activation=activation_key
    )
    autoware.set_initial_status(
        state=Autoware.CONST.STATE.START_PROCESSING,
    )
    autoware.start()
    while True:
        sleep(2)
        # print_all(kvs_client)
