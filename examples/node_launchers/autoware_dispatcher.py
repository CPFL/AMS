#!/usr/bin/env python
# coding: utf-8

import json
from argparse import ArgumentParser
from uuid import uuid1 as uuid
from time import sleep

from ams import MapsClient
from ams.helpers import Target
from ams.nodes import AutowareDispatcher, Autoware

from clients.kvs.redis.helper import get_kvs_client as get_redis_client
from clients.kvs.manager.helper import get_kvs_client as get_manager_client
from clients.kvs.helper import print_all

from clients.mqtt.aws_iot.helper import get_mqtt_client as get_aws_iot_client
from clients.mqtt.paho.helper import get_mqtt_client as get_paho_client

parser = ArgumentParser()

parser.add_argument("-ID", "--id", type=str, default=str(uuid()), help="node id")
parser.add_argument("-VIC", "--vehicle_id_csv", type=str, default=None, help="vehicle ids as csv")
# parser.add_argument("-UIC", "--user_id_csv", type=str, default=None, help="user ids as csv")

parser.add_argument(
    "-W", "--path_waypoint_json", type=str, default="../../res/waypoint.json", help="waypoint.json path")
parser.add_argument(
    "-A", "--path_arrow_json", type=str, default="../../res/arrow.json", help="arrow.json path")
parser.add_argument(
    "-SW", "--path_stop_waypoint_ids_json", type=str, default="../../res/stop_waypoint_ids.json",
    help="stop_waypoint_ids.json path")

parser.add_argument("-KH", "--kvs_host", type=str, default="localhost", help="kvs host")
parser.add_argument("-KP", "--kvs_port", type=int, default=6379, help="kvs port")
parser.add_argument("-KCT", "--kvs_client_type", type=str, default="manager", help="manager or redis")

parser.add_argument("-MH", "--mqtt_host", type=str, default="localhost", help="mqtt broker host")
parser.add_argument("-MP", "--mqtt_port", type=int, default=1883, help="mqtt broker port")
parser.add_argument("-MCT", "--mqtt_client_type", type=str, default="paho", help="paho or aws_iot")
parser.add_argument("-AWS", "--use_aws_iot", type=bool, default=False, help="use aws iot flag")
parser.add_argument(
    "-CAP", "--path_ca", type=str, default="./clients/aws_iot/key_dev/root-CA.crt", help="root ca file path")
parser.add_argument(
    "-PKP", "--path_private_key", type=str, default="./clients/aws_iot/key_dev/dev-autoware.private.key",
    help="private key file path")
parser.add_argument(
    "-CP", "--path_cert", type=str, default="./clients/aws_iot/key_dev/dev-autoware.cert.pem",
    help="certificate file path")

args = parser.parse_args()


if __name__ == '__main__':

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

    maps_client = MapsClient()
    maps_client.load_waypoint_json_file(args.path_waypoint_json)
    maps_client.load_arrow_json_file(args.path_arrow_json)

    stop_waypoint_ids = []
    with open(args.path_stop_waypoint_ids_json, "r") as f:
        stop_waypoint_ids = json.load(f)

    autoware_dispatcher = AutowareDispatcher(group=AutowareDispatcher.CONST.NODE_NAME, _id=args.id)
    autoware_dispatcher.set_kvs_client(kvs_client)
    autoware_dispatcher.set_mqtt_client(mqtt_client)
    autoware_dispatcher.set_maps_client(maps_client)

    targets = []
    if args.vehicle_id_csv is not None:
        for vehicle_id in args.vehicle_id_csv.split(","):
            targets.append(Target.new_target(
                group=Autoware.CONST.NODE_NAME,
                _id=vehicle_id
            ))

    autoware_dispatcher.set_initial_config(
        targets,
        inactive_api_keys=["test_api_key"],
        stop_waypoint_ids=stop_waypoint_ids
    )
    autoware_dispatcher.set_initial_state()
    autoware_dispatcher.start()
    while True:
        sleep(2)
        # print_all(kvs_client)
