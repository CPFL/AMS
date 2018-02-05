#!/usr/bin/env python
# coding: utf-8

import sys
from time import time, sleep
from copy import deepcopy
import paho.mqtt.client as mqtt
from multiprocessing import Process
import json

from ams import Topic
from ams.nodes import TrafficSignal
from ams.messages import traffic_signal_message


class MQTTClient(object):
    def __init__(self, host, port):
        self.__client = mqtt.Client(protocol=mqtt.MQTTv311)
        self.__client.connect(host, port=port, keepalive=60)

    def __del__(self):
        self.__client.disconnect()

    def publish(self, topic, message):
        self.__client.publish(topic, message)


TRAFFIC_SIGNAL_FILE = "../../res/trafficSignal.json"

if __name__ == '__main__':
    host = "localhost"
    port = 1883
    if 1 < len(sys.argv):
        host = sys.argv[1]
    if 2 < len(sys.argv):
        port = int(sys.argv[2])
    mqtt_client = MQTTClient(host, port)

    topicTrafficSignalSubscribe = Topic()
    topicTrafficSignalSubscribe.set_root(TrafficSignal.TOPIC.SUBSCRIBE)
    topicTrafficSignalSubscribe.set_message(traffic_signal_message)

    with open(TRAFFIC_SIGNAL_FILE, "r") as f:
        traffic_signal_configs = json.load(f)

    cycle_set = traffic_signal_configs["cycles"]
    cycle_groups = traffic_signal_configs["cycleGroups"]

    traffic_signals = {}
    routes = {}
    cycles = {}
    schedules = {}
    for intersection_id, intersection_cycle_groups in cycle_groups.items():
        instance = TrafficSignal(name=intersection_id)
        process = Process(target=instance.start)
        traffic_signals[intersection_id] = {
            "instance": instance,
            "process": process
        }
        process.start()
        routes[intersection_id] = []
        cycles[intersection_id] = []
        schedules[intersection_id] = []
        for group_name, group in intersection_cycle_groups.items():
            for route_code in group["routeCodes"]:
                traffic_signal_route_message = deepcopy(topicTrafficSignalSubscribe.get_template()["routes"][0])
                traffic_signal_route_message["route_code"] = route_code
                traffic_signal_route_message["action"] = None
                traffic_signal_route_message["state"] = None
                routes[intersection_id].append(traffic_signal_route_message)

                traffic_signal_schedule_message = deepcopy(topicTrafficSignalSubscribe.get_template()["schedules"][0])
                traffic_signal_schedule_message["route_code"] = route_code
                traffic_signal_schedule_message["state"] = "red"
                traffic_signal_schedule_message["start_time"] = time()
                traffic_signal_schedule_message["duration"] = 20
                schedules[intersection_id].append(traffic_signal_schedule_message)

            traffic_signal_cycle_message = deepcopy(topicTrafficSignalSubscribe.get_template()["cycles"][0])
            traffic_signal_cycle_message["route_codes"] = group["routeCodes"]
            traffic_signal_cycle_message["base_time"] = cycle_set[group_name]["baseTime"]
            traffic_signal_cycle_message["period"] = cycle_set[group_name]["period"]
            traffic_signal_cycle_message["phases"] = cycle_set[group_name]["phases"]
            cycles[intersection_id].append(traffic_signal_cycle_message)

    sleep(1)

    print("publish routes")
    for intersection_id, traffic_signal in traffic_signals.items():
        # print(topicTrafficSignalSubscribe.root+"/"+traffic_signal["instance"].event_loop_id()+"/routes")
        mqtt_client.publish(
            topicTrafficSignalSubscribe.root+"/"+traffic_signal["instance"].event_loop_id+"/routes",
            topicTrafficSignalSubscribe.serialize(routes[intersection_id])
        )

    sleep(5)

    print("publish cycles")
    for intersection_id, traffic_signal in traffic_signals.items():
        # print(topicTrafficSignalSubscribe.root+"/"+traffic_signal["instance"].event_loop_id()+"/cycles")
        mqtt_client.publish(
            topicTrafficSignalSubscribe.root+"/"+traffic_signal["instance"].event_loop_id+"/cycles",
            topicTrafficSignalSubscribe.serialize(cycles[intersection_id])
        )

    sleep(5)

    print("publish schedules")
    mqtt_client.publish(
        topicTrafficSignalSubscribe.root+"/"+traffic_signals["555"]["instance"].event_loop_id+"/schedules",
        topicTrafficSignalSubscribe.serialize(schedules["555"])
    )

    print("wait join")
    for traffic_signal in traffic_signals.values():
        traffic_signal["process"].join()
