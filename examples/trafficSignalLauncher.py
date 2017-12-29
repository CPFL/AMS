#!/usr/bin/env python
# coding: utf-8
from time import time, sleep
from copy import deepcopy
import paho.mqtt.client as mqtt
from config.env import env
import json
from trafficSignal import TrafficSignal
from const.traffic_signal import TRAFFIC_SIGNAL
from topic import Topic
from multiprocessing import Process


class MQTTClient(object):
    def __init__(self, host=env["MQTT_BROKER_HOST"], port=int(env["MQTT_BROKER_PORT"])):
        self.__client = mqtt.Client(protocol=mqtt.MQTTv311)
        self.__client.on_message = self.__onMessagePrint
        self.__client.connect(host, port=port, keepalive=60)

    def __del__(self):
        self.__client.disconnect()

    def subscribe(self, topic):
        self.__client.subscribe(topic)
        self.__client.loop_forever()

    def __onMessagePrint(self, client, userdata, messageData):
        message = messageData.payload.decode("utf-8")

    def publish(self, topic, message):
        self.__client.publish(topic, message)


if __name__ == '__main__':
    mqtt_client = MQTTClient()

    topicTrafficSignalSubscribe = Topic()
    topicTrafficSignalSubscribe.setRoot(TRAFFIC_SIGNAL.TOPIC.SUBSCRIBE)
    topicTrafficSignalSubscribe.load(TRAFFIC_SIGNAL.TOPIC.MESSAGE_FILE)

    with open("../res/trafficSignal.json", "r") as f:
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
                traffic_signal_route_message = deepcopy(topicTrafficSignalSubscribe.getTemplate()["routes"][0])
                traffic_signal_route_message["route_code"] = route_code
                traffic_signal_route_message["action"] = None
                traffic_signal_route_message["state"] = None
                routes[intersection_id].append(traffic_signal_route_message)

                traffic_signal_schedule_message = deepcopy(topicTrafficSignalSubscribe.getTemplate()["schedules"][0])
                traffic_signal_schedule_message["route_code"] = route_code
                traffic_signal_schedule_message["state"] = "red"
                traffic_signal_schedule_message["start_time"] = time()
                traffic_signal_schedule_message["duration"] = 20
                schedules[intersection_id].append(traffic_signal_schedule_message)

            traffic_signal_cycle_message = deepcopy(topicTrafficSignalSubscribe.getTemplate()["cycles"][0])
            traffic_signal_cycle_message["route_codes"] = group["routeCodes"]
            traffic_signal_cycle_message["base_time"] = cycle_set[group_name]["baseTime"]
            traffic_signal_cycle_message["period"] = cycle_set[group_name]["period"]
            traffic_signal_cycle_message["phases"] = cycle_set[group_name]["phases"]
            cycles[intersection_id].append(traffic_signal_cycle_message)

    sleep(1)

    print("publish routes")
    for intersection_id, traffic_signal in traffic_signals.items():
        # print(topicTrafficSignalSubscribe.root+"/"+traffic_signal["instance"].getEventLoopID()+"/routes")
        mqtt_client.publish(
            topicTrafficSignalSubscribe.root+"/"+traffic_signal["instance"].getEventLoopID()+"/routes",
            topicTrafficSignalSubscribe.serialize(routes[intersection_id])
        )

    sleep(5)

    print("publish cycles")
    for intersection_id, traffic_signal in traffic_signals.items():
        # print(topicTrafficSignalSubscribe.root+"/"+traffic_signal["instance"].getEventLoopID()+"/cycles")
        mqtt_client.publish(
            topicTrafficSignalSubscribe.root+"/"+traffic_signal["instance"].getEventLoopID()+"/cycles",
            topicTrafficSignalSubscribe.serialize(cycles[intersection_id])
        )

    sleep(5)

    print("publish schedules")
    mqtt_client.publish(
        topicTrafficSignalSubscribe.root+"/"+traffic_signals["555"]["instance"].getEventLoopID()+"/schedules",
        topicTrafficSignalSubscribe.serialize(schedules["555"])
    )

    print("wait join")
    for traffic_signal in traffic_signals.values():
        traffic_signal["process"].join()
