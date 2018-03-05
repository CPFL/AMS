#!/usr/bin/env python
# coding: utf-8

from argparse import ArgumentParser

from ams.ros import DecisionMakerStatesSubscriber


parser = ArgumentParser()
parser.add_argument("-H", "--host", type=str, default="localhost", help="host")
parser.add_argument("-P", "--port", type=int, default=1883, help="port")
parser.add_argument("-ID", "--id", type=str, required=True, help="node id")
parser.add_argument("-PS", "--period", type=float, default=1.0, help="period second")
args = parser.parse_args()


if __name__ == '__main__':

    decisionMakerStatesPublisher = DecisionMakerStatesSubscriber(_id=args.id, period=args.period)

    print("decisionMakerStatesPublisher {} on {}".format(
        decisionMakerStatesPublisher.event_loop_id, decisionMakerStatesPublisher.get_pid()))

    decisionMakerStatesPublisher.start(host=args.host, port=args.port)
