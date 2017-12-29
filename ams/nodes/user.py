#!/usr/bin/env python
# coding: utf-8

import random
from time import sleep

from ams import Topic
from ams.nodes import EventLoop
from ams.messages import user_message


class User(EventLoop):
    class TOPIC(object):
        PUBLISH = "pubUser"
        SUBSCRIBE = "subUser"

    class STATE(object):
        LOGIN = "login"
        WAITING = "waiting"
        GETTING_ON = "gettingOn"
        GOT_ON = "gotOn"
        MOVING = "moving"
        GETTING_OUT = "gettingOut"
        GOT_OUT = "gotOut"

    class ACTION(object):
        WAIT = "wait"
        GET_ON = "getOn"
        GET_OUT = "getOut"

    class EVENT(object):
        MOVE_VEHICLE = "moveVehicle"

    def __init__(self, name, waypoint, dt=1.0):
        super().__init__()

        self.topicUserPublish = Topic()
        self.topicUserPublish.set_id(self.event_loop_id)
        self.topicUserPublish.set_root(User.TOPIC.PUBLISH)
        self.topicUserPublish.set_message(user_message)

        self.topicUserSubscribe = Topic()
        self.topicUserSubscribe.set_id(self.event_loop_id)
        self.topicUserSubscribe.set_root(User.TOPIC.SUBSCRIBE)
        self.topicUserSubscribe.set_message(user_message)

        self.name = name
        self.id = self.event_loop_id
        self.state = User.STATE.LOGIN
        self.event = None
        self.action = None
        self.dt = dt
        self.__start_waypoint_id = None
        self.__goal_waypoint_id = None
        self.__vehicleID = None

        self.__waypoint = waypoint

        self.add_on_message_function(self.update_action)
        self.add_on_message_function(self.update_event)

        self.set_subscriber(self.topicUserSubscribe.private+"/schedules")
        self.set_subscriber(self.topicUserSubscribe.private+"/event")
        self.set_main_loop(self.__main_loop)

    def publish_status(self):
        message = self.topicUserPublish.get_template()
        message["name"] = self.name
        message["state"] = self.state
        message["event"] = self.event
        message["schedules"][0]["action"] = self.action
        message["schedules"][0]["start"]["waypoint_id"] = self.__start_waypoint_id
        message["schedules"][0]["goal"]["waypoint_id"] = self.__goal_waypoint_id
        payload = self.topicUserPublish.serialize(message)
        self.publish(self.topicUserPublish.private, payload)

    def set_waypoint_at_random(self, waypoint_ids=None):
        if waypoint_ids is None:
            waypoint_ids = self.__waypoint.get_waypoint_ids()
        start_waypoint_id = random.choice(waypoint_ids)
        waypoint_ids.remove(start_waypoint_id)
        goal_waypoint_id = random.choice(waypoint_ids)
        self.set_waypoint(start_waypoint_id, goal_waypoint_id)

    def set_waypoint(self, start_waypoint_id, goal_waypoint_id):
        self.set_start_waypoint(start_waypoint_id)
        self.set_goal_waypoint(goal_waypoint_id)

    def set_start_waypoint(self, start_waypoint_id):
        self.__start_waypoint_id = start_waypoint_id

    def set_goal_waypoint(self, goal_waypoint_id):
        self.__goal_waypoint_id = goal_waypoint_id

    def update_action(self, _client, _userdata, topic, payload):
        print(topic)
        if topic == self.topicUserSubscribe.private+"/schedules":
            message = self.topicUserSubscribe.unserialize(payload)
            self.action = message["schedules"][0]["action"]

    def update_event(self, _client, _userdata, topic, payload):
        if topic == self.topicUserSubscribe.private+"/event":
            message = self.topicUserSubscribe.unserialize(payload)
            self.event = message["event"]

    def update_status(self):
        print(self.state, self.event, self.action)
        if self.state == User.STATE.LOGIN:
            if self.action == User.ACTION.WAIT:
                self.state = User.STATE.WAITING
                self.action = None
        elif self.state == User.STATE.WAITING:
            if self.action == User.ACTION.GET_ON:
                self.state = User.STATE.GETTING_ON
                self.action = None
        elif self.state == User.STATE.GETTING_ON:
            self.state = User.STATE.GOT_ON
        elif self.state == User.STATE.GOT_ON:
            if self.event == User.EVENT.MOVE_VEHICLE:
                self.state = User.STATE.MOVING
                self.event = None
        elif self.state == User.STATE.MOVING:
            if self.action == User.ACTION.GET_OUT:
                self.state = User.STATE.GETTING_OUT
                self.action = None
        elif self.state == User.STATE.GETTING_OUT:
            self.state = User.STATE.GOT_OUT

    def __main_loop(self):
        self.publish_status()
        while self.state != User.STATE.GOT_OUT:
            sleep(self.dt)
            self.update_status()
            self.publish_status()
