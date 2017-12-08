#!/usr/bin/env python
# coding: utf-8

from uuid import uuid1 as uuid
import paho.mqtt.client as mqtt
import os
from signal import SIGKILL

from config.env import env
from const import CONST


class EventLoop(object):
    def __init__(self):
        self.__eventLoopID = str(uuid())
        self.__subscribers = {}
        self.__publishers = {}
        self.__client = None
        self.__mainLoop = None
        self.__pid = os.getpid()
        self.setSubscriber(CONST.TOPICS.EVENTLOOP)
        self.__onMessageFunction = lambda topic, message, userdata: print("no onMessageFunction")
        self.__userData = None

    def __del__(self):
        if self.__client is not None:
            self.__client.disconnect()

    def setSubscriber(self, topic):
        self.__subscribers[str(uuid())] = {"topic": topic}

    def setOnMessageFunction(self, onMessageFunction, userData=None):
        self.__onMessageFunction = onMessageFunction
        self.__userData = userData

    def setMainLoop(self, mainLoop):
        self.__mainLoop = mainLoop

    def publish(self, topic, message):
        if topic == "eventLoop":
            print("publish", id(self.__client), topic, message)
        self.__client.publish(topic, message)

    def __onMessage(self, client, userdata, messageData):
        message = messageData.payload.decode("utf-8")
        if messageData.topic == "eventLoop":
            if message == " ".join(["kill", self.__eventLoopID]):
                print("eventLoop", message)
                self.end()
            if message == " ".join(["check", self.__eventLoopID]):
                print("eventLoop", message)
                self.__check()
        self.__onMessageFunction(client, userdata, messageData.topic, message)
        return True

    def connect(self):
        self.__client = mqtt.Client(protocol=mqtt.MQTTv311, userdata=self.__userData)
        self.__client.on_message = self.__onMessage
        self.__client.connect(env["MQTT_BROKER_HOST"], port=env["MQTT_BROKER_PORT"], keepalive=CONST.KEEPALIVE)

    def start(self):
        self.connect()
        for subscriber in self.__subscribers.values():
            self.__client.subscribe(subscriber["topic"])
        self.publish(CONST.TOPICS.EVENTLOOP, " ".join(["start", self.__eventLoopID, str(self.__pid)]))

        if self.__mainLoop is None:
            self.__client.loop_forever()
        else:
            self.__client.loop_start()
            self.__mainLoop()

    def end(self):
        self.__client.loop_stop()
        self.__client.disconnect()
        os.kill(self.__pid, SIGKILL)

    def __check(self):
        # todo: mainLoop zombie
        self.publish(CONST.TOPICS.EVENTLOOP, " ".join([self.__eventLoopID, str(self.__pid), "ok"]))

    def getEventLoopID(self):
        return self.__eventLoopID

    def getPid(self):
        return self.__pid


if __name__ == '__main__':
    eventLoop = EventLoop()
    eventLoop.start()
    print("eventLoopID {} on {}".format(eventLoop.getEventLoopID(), eventLoop.getPid()))