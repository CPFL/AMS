#!/usr/bin/env python
# coding: utf-8

import json
import yaml

from setproctitle import setproctitle

from ams import logger
from ams.helpers import Topic
from ams.clients import MapsClient
from ams.nodes import AutowareInterface

from argument_parser import parser
from clients.helper import get_manager_client, get_redis_client, get_paho_client, get_aws_iot_client


if __name__ == '__main__':

    setproctitle("ams_autoware_interface")

    args = parser.parse_args()

    if args.kvs_client_type == "manager":
        kvs_client = get_manager_client()
    elif args.kvs_client_type == "redis":
        kvs_client = get_redis_client(args.kvs_host)
    else:
        raise ValueError("Unknown kvs client type: {}".format(args.kvs_client_type))

    if args.pubsub_client_type == "paho":
        pubsub_client = get_paho_client(args.pubsub_host, args.pubsub_port)
    elif args.pubsub_client_type == "aws_iot":
        pubsub_client = get_aws_iot_client(
            args.pubsub_host, args.pubsub_port, args.path_ca, args.path_private_key, args.path_cert)
    else:
        raise ValueError("Unknown pubsub client type: {}".format(args.pubsub_client_type))

    if args.use_ros_flag:
        from std_msgs.msg import String, Int32
        from geometry_msgs.msg import PoseStamped
        from autoware_msgs.msg import VehicleLocation, LaneArray

        from clients.helper import get_ros_client

        ros_msgs = {
            "VehicleLocation": VehicleLocation,
            "String": String,
            "PoseStamped": PoseStamped,
            "LaneArray": LaneArray,
            "Int32": Int32
        }
        ros_client = get_ros_client()
    else:
        ros_msgs = {
            "VehicleLocation": None,
            "String": None,
            "PoseStamped": None,
            "LaneArray": None,
            "Int32": None
        }
        ros_client = get_paho_client(args.pubsub_host, args.pubsub_port)
        ros_client.set_dumps(yaml.dump)
        ros_client.set_loads(yaml.load)

    maps_client = MapsClient()
    maps_client.load_waypoint_json_file(args.waypoint_json_path)
    maps_client.load_lane_json_file(args.lane_json_path)

    with open(args.initials_file_path, "r") as f:
        initials = json.load(f)

    Topic.domain = args.topic_domain

    autoware_interface = AutowareInterface(initials["config"], ros_msgs)
    autoware_interface.set_rate(args.event_loop_rate)
    autoware_interface.set_clients(kvs_client, pubsub_client, maps_client, ros_client)
    try:
        autoware_interface.start()
    except KeyboardInterrupt:
        logger.info("autoware_interface node: KeyboardInterrupt")
    except Exception as e:
        logger.error(logger.pformat(e))
    finally:
        del kvs_client
        del pubsub_client
        del maps_client
        del ros_client
        del autoware_interface
