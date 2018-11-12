#!/usr/bin/env python
# coding: utf-8
from argparse import ArgumentParser

parser = ArgumentParser()
parser.add_argument("-IFP", "--initials_file_path", type=str, required=True, help="initials file path")

parser.add_argument("-TD", "--topic_domain", type=str, default="ams", help="topic domain")

parser.add_argument("-URF", "--use_ros_flag", type=bool, default=False, help="use ros flag")
parser.add_argument("-ELR", "--event_loop_rate", type=float, default=1.0, help="event_loop rate (Hz)")

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

parser.add_argument(
    "-WJP", "--waypoint_json_path", type=str, default="./static/maps/waypoint.json", help="waypoint.json file path")
parser.add_argument(
    "-LJP", "--lane_json_path", type=str, default="./static/maps/lane.json", help="lane.json file path")
