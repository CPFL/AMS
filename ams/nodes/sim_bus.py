#!/usr/bin/env python
# coding: utf-8

from time import time
from ams import Schedule, Topic, Target
from ams.nodes import SimCar
from ams.messages import UserStatus
from ams.structures import SIM_BUS, SIM_BUS_USER, USER

from pprint import PrettyPrinter
pp = PrettyPrinter(indent=2).pprint


class SimBus(SimCar):

    CONST = SIM_BUS

    def __init__(
            self, _id, name, waypoint, arrow, route, intersection, dt=1.0):
        super().__init__(_id, name, waypoint, arrow, route, intersection, dt=dt)
        self.state = SIM_BUS.SCHEDULE.STAND_BY
        self.user_statuses = {}

        self.__topicSubUserStatus = Topic()
        self.__topicSubUserStatus.set_targets(Target.new_target(None, SIM_BUS_USER.NODE_NAME), None)
        self.__topicSubUserStatus.set_categories(USER.TOPIC.CATEGORIES.STATUS)
        self.__topicSubUserStatus.set_message(UserStatus)
        self.set_subscriber(self.__topicSubUserStatus, self.update_user_status)

    def update_user_status(self, _client, _userdata, topic, payload):
        user_id = self.__topicSubUserStatus.get_from_id(topic)
        user_status = self.__topicSubUserStatus.unserialize(payload)
        if user_id in self.user_statuses and user_status.state == USER.STATE.LOG_OUT:
            self.user_statuses.pop(user_id)
        else:
            self.user_statuses[user_id] = user_status

    def is_needed_to_via(self):
        target_bus_stops = Target.get_same_group_targets_in_targets("spot", self.schedules[0].targets)
        for target_bus_stop in target_bus_stops:
            waiting_user_statuses = list(
                filter(lambda x: x.state == SIM_BUS_USER.STATE.WAITING, self.user_statuses.values()))
            for user_status in waiting_user_statuses:
                target_waiting_bus_stop = Target.get_same_group_targets_in_targets(
                    SIM_BUS_USER.TARGET_GROUP.START_BUS_STOP, user_status.trip_schedules[0].targets)[0]
                if target_bus_stop.id == target_waiting_bus_stop.id:
                    return True
            stop_requested_user_statuses = list(
                filter(lambda x: x.schedule.event == SIM_BUS_USER.ACTION.REQUEST_STOP, self.user_statuses.values()))
            for user_status in stop_requested_user_statuses:
                target_waiting_bus_stop = Target.get_same_group_targets_in_targets(
                    SIM_BUS_USER.TARGET_GROUP.GOAL_BUS_STOP, user_status.trip_schedules[0].targets)[0]
                if target_bus_stop.id == target_waiting_bus_stop.id:
                    return True
        return False

    def update_status(self):
        current_time = time()
        schedule_event = self.schedules[0].event
        # print("SimBus:", self.state, schedule_event)
        if schedule_event == SIM_BUS.SCHEDULE.STAND_BY:
            if 1 < len(self.schedules):
                self.schedules.pop(0)

                # update next schedule
                dif_time = current_time - self.schedules[0].period.start
                self.schedules = Schedule.get_shifted_schedules(self.schedules, dif_time)

                self.state = SIM_BUS.STATE.MOVE
                self.publish_status()
        elif schedule_event == SIM_BUS.SCHEDULE.MOVE_TO_BRANCH_POINT:
            self.update_pose()
            self.update_velocity()
            if 1 == len(self.schedules):
                if self.is_needed_to_via():
                    self.state = SIM_BUS.STATE.VIA
                else:
                    self.state = SIM_BUS.STATE.THROUGH
                self.publish_status()
            else:
                if self.is_achieved():
                    self.schedules.pop(0)

                    # update next schedule
                    new_start_time = time()
                    dif_time = new_start_time - self.schedules[0].period.start
                    self.schedules = Schedule.get_shifted_schedules(self.schedules, dif_time)

                    self.state = SIM_BUS.STATE.MOVE
                    self.publish_status()
        elif schedule_event in [
            SIM_BUS.SCHEDULE.MOVE_TO_CIRCULAR_ROUTE,
            SIM_BUS.SCHEDULE.MOVE_TO_SELECT_POINT,
            SIM_BUS.SCHEDULE.MOVE_TO_JUNCTION
        ]:
            self.update_pose()
            self.update_velocity()
            if self.is_achieved():
                self.schedules.pop(0)

                # update next schedule
                new_start_time = time()
                dif_time = new_start_time - self.schedules[0].period.start
                self.schedules = Schedule.get_shifted_schedules(self.schedules, dif_time)

                self.state = SIM_BUS.STATE.MOVE
                self.publish_status()

        elif schedule_event == SIM_BUS.SCHEDULE.MOVE_TO_BUS_STOP:
            self.update_pose()
            self.update_velocity()
            if self.is_achieved():
                self.waypoint_id = self.schedules[0].route.goal_waypoint_id
                self.arrow_code = self.schedules[0].route.arrow_codes[-1]
                self.np_position = self.waypoint.get_np_position(self.waypoint_id)
                self.yaw = self.arrow.get_yaw(self.arrow_code, self.waypoint_id)
                self.schedules.pop(0)

                # update next schedule
                new_start_time = time()
                dif_time = new_start_time - self.schedules[0].period.start
                self.schedules = Schedule.get_shifted_schedules(self.schedules, dif_time)

                self.state = SIM_BUS.STATE.STOP
                self.publish_status()

        elif schedule_event == SIM_BUS.SCHEDULE.STOP_TO_DISCHARGE:
            if self.schedules[0].period.end < current_time:
                self.schedules.pop(0)

                # update next schedule
                dif_time = current_time - self.schedules[0].period.start
                self.schedules = Schedule.get_shifted_schedules(self.schedules, dif_time)

                self.state = SIM_BUS.STATE.STOP
                self.publish_status()

        elif schedule_event in [SIM_BUS.SCHEDULE.STOP_TO_TAKE_UP, SIM_BUS.SCHEDULE.STOP_TO_DISCHARGE_AND_TAKE_UP]:
            if self.schedules[0].period.end < current_time:
                self.schedules.pop(0)

                # update next schedule
                dif_time = current_time - self.schedules[0].period.start
                self.schedules = Schedule.get_shifted_schedules(self.schedules, dif_time)

                self.state = SIM_BUS.STATE.MOVE
                self.publish_status()
