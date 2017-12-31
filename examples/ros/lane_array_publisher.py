#!/usr/bin/env python
# coding: utf-8

import sys
from ams.ros import LaneArrayPublisher
from config.env import env


if __name__ == '__main__':
    laneArrayPublisher = LaneArrayPublisher(name=sys.argv[1])
    print("laneArrayPublisher {} on {}".format(laneArrayPublisher.event_loop_id, laneArrayPublisher.get_pid()))
    laneArrayPublisher.start(env["MQTT_BROKER_HOST"], int(env["MQTT_BROKER_PORT"]))

