#!/usr/bin/env python
# coding: utf-8

from time import sleep

from ams import Topic, Location, Position
from ams.nodes import EventLoop
from ams.messages import VehicleStatus
from ams.structures import Pose, Orientation, Rpy, Target, Schedules, VEHICLE

from pprint import PrettyPrinter
pp = PrettyPrinter(indent=2).pprint


class Vehicle(EventLoop):

    CONST = VEHICLE

    def __init__(self, name, waypoint, arrow, route, waypoint_id, arrow_code, velocity, dt=1.0):
        super().__init__()

        self.topicStatus = Topic()
        self.topicStatus.set_id(self.event_loop_id)
        self.topicStatus.set_root(VEHICLE.TOPIC.PUBLISH)

        self.topicSchedules = Topic()
        self.topicSchedules.set_id(self.event_loop_id)
        self.topicSchedules.set_root(VEHICLE.TOPIC.SUBSCRIBE)

        self.topicGeo = Topic()
        self.topicGeo.set_id(self.event_loop_id)
        self.topicGeo.set_root(VEHICLE.TOPIC.GEO.PUBLISH)

        self.name = name
        self.state = VEHICLE.STATE.STOP
        self.waypoint = waypoint
        self.arrow = arrow
        self.route = route
        self.velocity = velocity
        self.schedules = None
        self.dt = dt
        self.waypoint_id = waypoint_id
        self.arrow_code = arrow_code
        self.np_position = self.waypoint.get_np_position(self.waypoint_id)
        self.yaw = self.arrow.get_yaw(self.arrow_code, self.waypoint_id)

        self.add_on_message_function(self.update_schedules)
        self.set_subscriber(self.topicSchedules.private+"/schedules")
        self.set_main_loop(self.__main_loop)

    def set_schedules(self, schedules):
        self.schedules = schedules

    def new_location(self):
        return Location.new_location(self.waypoint_id, self.arrow_code, self.waypoint.get_geohash(self.waypoint_id))

    def get_pose(self):
        return Pose.new_data(
            position=Position.new_position_from_np_position(self.np_position),
            orientation=Orientation.new_data(
                rpy=Rpy.new_data(
                    yaw=self.yaw
                )
            )
        )

    def get_status(self):
        return VehicleStatus.new_data(
            name=self.name,
            state=self.state,
            schedule=self.schedules[0],
            location=self.new_location(),
            pose=self.get_pose()
        )

    def publish_status(self):
        payload = self.topicStatus.serialize(self.get_status())
        self.publish(self.topicStatus.private, payload)

    def publish_geo_topic(self):
        self.publish(
            self.topicGeo.root+"/"+"/".join(self.waypoint.get_geohash(self.waypoint_id)),
            self.topicGeo.serialize(Target.new_data(id=self.event_loop_id, group="Vehicle"))
        )

    def update_schedules(self, _client, _userdata, topic, payload):
        if topic == self.topicSchedules.private+"/schedules":
            # print("update_schedules")
            message = self.topicSchedules.unserialize(payload)
            self.schedules = Schedules.new_data(message)

    def update_status(self):
        return

    def __main_loop(self):
        sleep(1)

        self.publish_status()
        self.publish_geo_topic()

        while self.schedules is not None:
                sleep(self.dt)
                self.update_status()
                self.publish_status()
                self.publish_geo_topic()

        return True
