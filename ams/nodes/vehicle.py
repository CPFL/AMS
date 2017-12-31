#!/usr/bin/env python
# coding: utf-8

from time import sleep
import Geohash

from ams import Topic
from ams.nodes import EventLoop
from ams.messages import vehicle_message, geo_vehicle_message

from pprint import PrettyPrinter
pp = PrettyPrinter(indent=2).pprint


class Vehicle(EventLoop):

    class TOPIC(object):
        PUBLISH = "pub_vehicle"
        SUBSCRIBE = "sub_vehicle"

        class GEO(object):
            PUBLISH = "pub_geo_vehicle"
            SUBSCRIBE = "sub_geo_vehicle"

    class STATE(object):
        MOVE = "move"
        STOP = "stop"
        WILL = "will"

    class ACTION(object):
        MOVE = "move"
        STOP = "stop"

    def __init__(self, name, waypoint, arrow, route, waypoint_id, velocity, schedules=None, dt=1.0):
        super().__init__()

        self.topicVehiclePublish = Topic()
        self.topicVehiclePublish.set_id(self.event_loop_id)
        self.topicVehiclePublish.set_root(Vehicle.TOPIC.PUBLISH)
        self.topicVehiclePublish.set_message(vehicle_message)

        self.topicVehicleSubscribe = Topic()
        self.topicVehicleSubscribe.set_id(self.event_loop_id)
        self.topicVehicleSubscribe.set_root(Vehicle.TOPIC.SUBSCRIBE)
        self.topicVehicleSubscribe.set_message(vehicle_message)

        self.topicGeoVehiclePublish = Topic()
        self.topicGeoVehiclePublish.set_id(self.event_loop_id)
        self.topicGeoVehiclePublish.set_root(Vehicle.TOPIC.GEO.PUBLISH)
        self.topicGeoVehiclePublish.set_message(geo_vehicle_message)

        self.name = name
        self.state = Vehicle.STATE.STOP
        self.event = None
        self.action = None
        self.waypoint = waypoint
        self.arrow = arrow
        self.route = route
        self.waypoint_id = waypoint_id
        self.position = self.waypoint.get_position(self.waypoint_id)
        self.velocity = velocity
        self.schedules = schedules
        self.dt = dt

        self.arrow_code = self.arrow.get_arrow_codes_from_waypoint_id(waypoint_id)[0]
        self.position = self.waypoint.get_position(self.waypoint_id)
        self.yaw = self.arrow.get_heading(self.arrow_code, self.waypoint_id)

        self.add_on_message_function(self.update_schedules)
        self.add_on_message_function(self.update_event)
        self.set_subscriber(self.topicVehicleSubscribe.private+"/schedules")
        self.set_subscriber(self.topicVehicleSubscribe.private+"/event")
        self.set_main_loop(self.__main_loop)

    def publish_status(self):
        xyz = self.position.data[:]
        message = self.topicVehiclePublish.get_template()
        message["name"] = self.name
        message["state"] = self.state
        message["event"] = self.event
        message["location"]["waypoint_id"] = self.waypoint_id
        message["location"]["geohash"] = self.waypoint.get_geohash(self.waypoint_id)
        message["location"]["arrow_code"] = self.arrow_code
        message["pose"]["position"]["x"] = xyz[0]
        message["pose"]["position"]["y"] = xyz[1]
        message["pose"]["position"]["z"] = xyz[2]
        message["pose"]["orientation"]["yaw"] = self.yaw
        message["schedules"] = self.schedules
        payload = self.topicVehiclePublish.serialize(message)
        self.publish(self.topicVehiclePublish.private, payload)
        self.publish(
            self.topicGeoVehiclePublish.root+"/"+"/".join(self.waypoint.get_geohash(self.waypoint_id)),
            self.event_loop_id)

    def update_schedules(self, _client, _userdata, topic, payload):
        if topic == self.topicVehicleSubscribe.private+"/schedules":
            message = self.topicVehicleSubscribe.unserialize(payload)
            self.schedules = message["schedules"]

    def update_event(self, _client, _userdata, topic, payload):
        if topic == self.topicVehicleSubscribe.private+"/event":
            message = self.topicVehicleSubscribe.unserialize(payload)
            self.event = message["event"]

    def update_status(self):
        return

    def __main_loop(self):
        sleep(1)

        self.publish_status()

        while self.schedules is not None:
                sleep(self.dt)
                self.update_status()
                self.publish_status()

        return True
