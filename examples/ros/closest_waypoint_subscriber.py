#!/usr/bin/env python
# coding: utf-8

from argparse import ArgumentParser

from ams.ros import ClosestWaypointSubscriber


parser = ArgumentParser()
parser.add_argument("-H", "--host", type=str, default="localhost", help="host")
parser.add_argument("-P", "--port", type=int, default=1883, help="port")
parser.add_argument("-N", "--name", type=str, default="sim_car 1", help="name")
parser.add_argument("-PS", "--period", type=float, default=1.0, help="period second")
args = parser.parse_args()


if __name__ == '__main__':

    closestWaypointSubscriber = ClosestWaypointSubscriber(
        name=args.name, host=args.host, port=args.port, period=args.period)

    print("closestWaypointSubscriber {} on {}".format(
        closestWaypointSubscriber.event_loop_id, closestWaypointSubscriber.get_pid()))

    closestWaypointSubscriber.start(host=args.host, port=args.port)
