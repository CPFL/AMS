#!/usr/bin/env python
# coding: utf-8

from time import time, sleep
from copy import deepcopy

from ams import Topic, Location, Position, Schedule
from ams.nodes import EventLoop
from ams.messages import VehicleStatus
from ams.structures import Pose, Orientation, Rpy, Schedules, VEHICLE, FLEET_MANAGER


class Vehicle(EventLoop):

    CONST = VEHICLE

    def __init__(self, _id, name, waypoint, arrow, route, dt=1.0):
        super().__init__(_id)

        self.status = VehicleStatus.new_data(
            name=name,
            time=time(),
            state=VEHICLE.STATE.LOG_IN,
            schedule=None,
            location=None,
            pose=None
        )

        self.waypoint = waypoint
        self.arrow = arrow
        self.route = route
        self.state_machine = None
        self.np_position = None
        self.dt = dt

        self.schedules = self.manager.list()
        self.schedules_lock = self.manager.Lock()

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
        self.set_location(Location.new_location(waypoint_id, arrow_code, self.waypoint.get_geohash(waypoint_id)))

    def set_location(self, location):
        self.status.location = location
        self.update_np_position()
        self.status.pose = Pose.new_data(
            position=Position.new_position_from_np_position(self.np_position),
            orientation=Orientation.new_data(
                rpy=Rpy.new_data(
                    yaw=self.arrow.get_yaw(self.status.location.arrow_code, self.status.location.waypoint_id)
                )
            )
        )

    def update_np_position(self):
        self.np_position = self.waypoint.get_np_position(self.status.location.waypoint_id)

    def set_schedules(self, schedules):
        self.schedules[:] = schedules
        self.status.schedule = deepcopy(self.schedules[0])

    def publish_status(self):
        self.status.time = time()
        self.status.state = self.state_machine.state
        if self.status.location is not None:
            self.status.location.geohash = self.waypoint.get_geohash(self.status.location.waypoint_id)
        if self.np_position is not None:
            self.status.pose.position = Position.new_position_from_np_position(self.np_position)
        payload = self.__topicPubStatus.serialize(self.status)
        self.publish(self.__topicPubStatus, payload)

    def publish_geotopic(self):
        if self.status.location is not None:
            self.__topicPubGeotopic.set_categories(
                VEHICLE.TOPIC.CATEGORIES.GEOTOPIC + list(self.status.location.geohash)
            )
            self.publish(self.__topicPubGeotopic, self.__topicPubGeotopic.serialize(self.target))

    def update_status_schedule(self):
        pass

    def update_schedules(self, _client, _userdata, _topic, payload):
        new_schedules = self.__topicSubSchedules.unserialize(payload)
        index = list(map(lambda x: x.id, new_schedules)).index(self.status.schedule.id)

        self.schedules_lock.acquire()
        self.schedules[:] = new_schedules[index:]
        self.update_status_schedule()
        self.schedules_lock.release()

    def get_next_schedules(self, schedules, current_time):
        schedules.pop(0)
        dif_time = current_time - schedules[0].period.start
        schedules = Schedule.get_shifted_schedules(schedules, dif_time)
        self.status.schedule = schedules[0]
        return schedules

    def get_schedules_and_lock(self):
        self.schedules_lock.acquire()
        return deepcopy(self.schedules)

    def set_schedules_and_unlock(self, schedules):
        self.schedules[:] = schedules
        self.schedules_lock.release()

    def update_status(self):
        schedules = self.get_schedules_and_lock()

        self.set_schedules_and_unlock(schedules)
        return

    def __main_loop(self):

        while self.status.state != VEHICLE.STATE.LOG_OUT:
            sleep(self.dt)
            self.update_status()
            self.publish_status()
            self.publish_geotopic()

        return True
