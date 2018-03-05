#!/usr/bin/env python
# coding: utf-8

from argparse import ArgumentParser

from ams.ros import StateCommandPublisher


parser = ArgumentParser()
parser.add_argument("-H", "--host", type=str, default="localhost", help="host")
parser.add_argument("-P", "--port", type=int, default=1883, help="port")
parser.add_argument("-ID", "--id", type=str, required=True, help="node id")
args = parser.parse_args()


if __name__ == '__main__':

    stateCommandPublisher = StateCommandPublisher(_id=args.id)

    print("stateCommandPublisher {} on {}".format(
        stateCommandPublisher.event_loop_id, stateCommandPublisher.get_pid()))

    stateCommandPublisher.start(host=args.host, port=args.port)
