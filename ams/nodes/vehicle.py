#!/usr/bin/env python
# coding: utf-8

from time import sleep

from ams import Topic, Location, Position
from ams.nodes import EventLoop
from ams.messages import VehicleStatus
from ams.structures import Pose, Orientation, Rpy, Schedules, VEHICLE, FLEET_MANAGER

from pprint import PrettyPrinter
pp = PrettyPrinter(indent=2).pprint


class Vehicle(EventLoop):

    CONST = VEHICLE

    def __init__(self, _id, name, waypoint, arrow, route, dt=1.0):
        super().__init__(_id)

        self.name = name
        self.state = VEHICLE.STATE.LOG_IN
        self.waypoint = waypoint
        self.arrow = arrow
        self.route = route
        self.schedules = None
        self.dt = dt

        self.waypoint_id = None
        self.arrow_code = None
        self.np_position = None
        self.yaw = None
        self.velocity = None

        self.__topicPubStatus = Topic()
        self.__topicPubStatus.set_targets(self.target)
        self.__topicPubStatus.set_categories(VEHICLE.TOPIC.CATEGORIES.STATUS)

        self.__topicSubSchedules = Topic()
        self.__topicSubSchedules.set_targets(None, self.target)
        self.__topicSubSchedules.set_categories(FLEET_MANAGER.TOPIC.CATEGORIES.SCHEDULES)
        self.__topicSubSchedules.set_message(Schedules)
        self.set_subscriber(self.__topicSubSchedules, self.update_schedules)

        self.__topicPubGeotopic = Topic()
        self.__topicPubGeotopic.set_targets(self.target, None)

        self.set_main_loop(self.__main_loop)

    def set_waypoint_id_and_arrow_code(self, waypoint_id, arrow_code):
        self.waypoint_id = waypoint_id
        self.arrow_code = arrow_code
        self.update_np_position()
        self.update_yaw_from_map()

    def set_velocity(self, velocity):
        self.velocity = velocity

    def update_np_position(self):
        self.np_position = self.waypoint.get_np_position(self.waypoint_id)

    def update_yaw_from_map(self):
        self.yaw = self.arrow.get_yaw(self.arrow_code, self.waypoint_id)

    def set_schedules(self, schedules):
        self.schedules = schedules

    def get_schedule(self):
        if self.schedules is None:
            return None
        return self.schedules[0]

    def get_location(self):
        if None in [self.waypoint_id, self.arrow_code]:
            return None
        return Location.new_location(self.waypoint_id, self.arrow_code, self.waypoint.get_geohash(self.waypoint_id))

    def get_pose(self):
        if self.np_position is None:
            return None
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
            schedule=self.get_schedule(),
            location=self.get_location(),
            pose=self.get_pose()
        )

    def publish_status(self):
        payload = self.__topicPubStatus.serialize(self.get_status())
        self.publish(self.__topicPubStatus, payload)

    def publish_geotopic(self):
        if self.waypoint_id is not None:
            self.__topicPubGeotopic.set_categories(
                VEHICLE.TOPIC.CATEGORIES.GEOTOPIC + list(self.waypoint.get_geohash(self.waypoint_id))
            )
            self.publish(self.__topicPubGeotopic, self.__topicPubGeotopic.serialize(self.target))

    def update_schedules(self, _client, _userdata, _topic, payload):
        self.schedules = self.__topicSubSchedules.unserialize(payload)

    def update_status(self):
        return

    def __main_loop(self):

        while self.state != VEHICLE.STATE.LOG_OUT:
            sleep(self.dt)
            self.update_status()
            self.publish_status()
            self.publish_geotopic()

        return True
