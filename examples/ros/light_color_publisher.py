#!/usr/bin/env python
# coding: utf-8

from argparse import ArgumentParser

from ams.ros import LightColorPublisher


parser = ArgumentParser()
parser.add_argument("-H", "--host", type=str, default="localhost", help="host")
parser.add_argument("-P", "--port", type=int, default=1883, help="port")
parser.add_argument("-ID", "--id", type=str, required=True, help="node id")
args = parser.parse_args()


if __name__ == '__main__':
    lightColorPublisher = LightColorPublisher(_id=args.id)

    print("lightColorPublisher {} on {}".format(
        lightColorPublisher.event_loop_id, lightColorPublisher.get_pid()))

    lightColorPublisher.start(host=args.host, port=args.port)
