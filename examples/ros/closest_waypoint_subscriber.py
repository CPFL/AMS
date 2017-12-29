#!/usr/bin/env python
# coding: utf-8

import sys
from ams.ros import ClosestWaypointSubscriber
from config.env import env


if __name__ == '__main__':
    closestWaypointSubscriber = ClosestWaypointSubscriber(name=sys.argv[1], host=env["MQTT_BROKER_HOST"], port=int(env["MQTT_BROKER_PORT"]))
    print("closestWaypointSubscriber {} on {}".format(closestWaypointSubscriber.event_loop_id, closestWaypointSubscriber.get_pid()))
    closestWaypointSubscriber.start(host=env["MQTT_BROKER_HOST"], port=int(env["MQTT_BROKER_PORT"]))
