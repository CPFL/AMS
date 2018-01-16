#!/usr/bin/env python
# coding: utf-8

import sys
from ams.ros import LightColorManagedPublisher
from config.env import env


if __name__ == '__main__':
    lightColorManagedPublisher = LightColorManagedPublisher(name=sys.argv[1])
    print("lightColorManagedPublisher {} on {}".format(lightColorManagedPublisher.event_loop_id, lightColorManagedPublisher.get_pid()))
    lightColorManagedPublisher.start(env["MQTT_BROKER_HOST"], int(env["MQTT_BROKER_PORT"]))

