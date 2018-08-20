#!/usr/bin/env python
# coding: utf-8

from time import time, sleep
from argparse import ArgumentParser

from ams.helpers import Topic, Target
from ams.nodes import SimAutoware

from clients.helper import get_manager_client, get_redis_client, get_paho_client, get_aws_iot_client


if __name__ == '__main__':

    parser = ArgumentParser()
    
    parser.add_argument("-NN", "--name", type=str, required=True, help="node name")
    parser.add_argument("-ID", "--id", type=str, required=True, help="node id")
    parser.add_argument("-VNN", "--target_vehicle_node_name", type=str, required=True, help="target vehicle node name")
    parser.add_argument("-VID", "--target_vehicle_node_id", type=str, required=True, help="target vehicle node id")
    parser.add_argument(
        "-IP", "--initial_pose", type=float, default=None, nargs=7,
        help="pose(x,y,z,x,y,z,w) as csv ex: -53.6064338684,-85.6038742065,0,0,0,-0.409678852825,0.912229816191")

    parser.add_argument("-TD", "--topic_domain", type=str, default="ams", help="topic domain")

    parser.add_argument("-KH", "--kvs_host", type=str, default="localhost", help="kvs host")
    parser.add_argument("-KP", "--kvs_port", type=int, default=6379, help="kvs port")
    parser.add_argument("-KCT", "--kvs_client_type", type=str, default="manager", help="manager or redis")

    parser.add_argument("-PSH", "--pubsub_host", type=str, default="localhost", help="pubsub broker host")
    parser.add_argument("-PSP", "--pubsub_port", type=int, default=1883, help="pubsub broker port")
    parser.add_argument(
        "-PSCT", "--pubsub_client_type", type=str, default="paho", help="paho or aws_iot", choices=["paho", "aws_iot"])
    parser.add_argument(
        "-CAP", "--path_ca", type=str, default="./clients/key_dev/root-CA.crt", help="root ca file path")
    parser.add_argument(
        "-PKP", "--path_private_key", type=str, default="./clients/key_dev/dev-autoware.private.key",
        help="private key file path")
    parser.add_argument(
        "-CP", "--path_cert", type=str, default="./clients/key_dev/dev-autoware.cert.pem",
        help="certificate file path")

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

    Topic.domain = args.topic_domain

    sim_autoware = SimAutoware(group=args.name, _id=args.id)
    sim_autoware.set_kvs_client(kvs_client)
    sim_autoware.set_pubsub_client(pubsub_client)

    start_time = time()

    sim_autoware.set_initial_config(
        target_vehicle=Target.new_target(
            args.target_vehicle_node_name,
            args.target_vehicle_node_id
        )
    )
    pose = None
    if args.initial_pose is not None:
        pose = {
            "position": {
                "x": args.initial_pose[0],
                "y": args.initial_pose[1],
                "z": args.initial_pose[2],
            },
            "orientation": {
                "x": args.initial_pose[3],
                "y": args.initial_pose[4],
                "z": args.initial_pose[5],
                "w": args.initial_pose[6],
            }
        }
    sim_autoware.set_initial_status(
        decision_maker_state=SimAutoware.CONST.DECISION_MAKER_STATE.WAIT_MISSION_ORDER,
        pose=pose
    )
    sim_autoware.start()
    while True:
        sleep(2)
