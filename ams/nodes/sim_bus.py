#!/usr/bin/env python
# coding: utf-8

from time import time
from ams import Schedule
from ams.nodes import SimCar, Vehicle

from pprint import PrettyPrinter
pp = PrettyPrinter(indent=2).pprint


class SimBus(SimCar):

    class STATE(object):
        STANDBY = "standBy"
        MOVE_TO_CIRCULAR_ROUTE = "moveToCircularRoute"
        MOVE_TO_BUS_STOP = "moveToNextStop"
        MOVE_TO_PARKING = "moveToParking"
        STOP_TO_TAKE_UP = "StopToTakeUp"
        STOP_TO_DISCHARGE = "StopToDischarge"
        STOP_TO_DISCHARGING_AND_TAKE_UP = "StopToDischargeAndTakeUp"

    def __init__(
            self, name, waypoint, arrow, route, intersection, waypoint_id, arrow_code, velocity, dt=1.0):
        super().__init__(name, waypoint, arrow, route, intersection, waypoint_id, arrow_code, velocity, dt)
        self.state = SimBus.STATE.STANDBY
        self.circular_route = None
        self.circular_route_waypoints = None

    def set_circular_route(self, circular_route):
        self.circular_route = circular_route
        self.circular_route_waypoints = self.route.get_route_waypoint_ids(self.circular_route)

    def on_circular_route(self):
        return self.waypoint_id in self.circular_route_waypoints

    def update_status(self):
        current_time = time()
        # print("SimBus.update_status", self.state, self.schedules[0].event)
        if self.state == SimBus.STATE.STANDBY:
            # print(SimBus.STATE.STANDBY, len(self.schedules))
            if 1 < len(self.schedules) and self.schedules[0].event == SimBus.STATE.MOVE_TO_CIRCULAR_ROUTE:
                self.schedules.pop(0)

                # update next schedule
                dif_time = current_time - self.schedules[0].period.start
                self.schedules = Schedule.get_shifted_schedules(self.schedules, dif_time)

                # print(self.schedules[0])
                self.state = self.schedules[0].event
                self.publish_status()

        elif self.state == SimBus.STATE.MOVE_TO_CIRCULAR_ROUTE:
            self.update_pose()
            self.update_velocity()
            if self.is_achieved():
                self.waypoint_id = self.schedules[0].route.goal_waypoint_id
                self.arrow_code = self.schedules[0].route.arrow_codes[-1]
                self.position = self.waypoint.get_position(self.waypoint_id)
                self.yaw = self.arrow.get_yaw(self.arrow_code, self.waypoint_id)
                self.schedules.pop(0)

                # update next schedule
                new_start_time = time()
                dif_time = new_start_time - self.schedules[0].period.start
                self.schedules = Schedule.get_shifted_schedules(self.schedules, dif_time)

                self.state = SimBus.STATE.STOP_TO_TAKE_UP
                self.publish_status()
            else:
                arrow_codes = self.schedules[0].route.arrow_codes
                self.schedules[0].route.arrow_codes = arrow_codes[arrow_codes.index(self.arrow_code):]

        elif self.state == SimBus.STATE.MOVE_TO_BUS_STOP:
            self.update_pose()
            self.update_velocity()
            if self.is_achieved():
                self.waypoint_id = self.schedules[0].route.goal_waypoint_id
                self.arrow_code = self.schedules[0].route.arrow_codes[-1]
                self.position = self.waypoint.get_position(self.waypoint_id)
                self.yaw = self.arrow.get_yaw(self.arrow_code, self.waypoint_id)
                self.schedules.pop(0)

                # update next schedule
                new_start_time = time()
                dif_time = new_start_time - self.schedules[0].period.start
                self.schedules = Schedule.get_shifted_schedules(self.schedules, dif_time)

                self.state = SimBus.STATE.STOP_TO_TAKE_UP
                self.publish_status()
            else:
                arrow_codes = self.schedules[0].route.arrow_codes
                self.schedules[0].route.arrow_codes = arrow_codes[arrow_codes.index(self.arrow_code):]

        elif self.state == SimBus.STATE.STOP_TO_DISCHARGE:
            if self.schedules[0].period.end < current_time:
                self.schedules.pop(0)

                # update next schedule
                dif_time = current_time - self.schedules[0].period.start
                self.schedules = Schedule.get_shifted_schedules(self.schedules, dif_time)

                self.state = SimBus.STATE.STOP_TO_TAKE_UP
                self.publish_status()

        elif self.state in [SimBus.STATE.STOP_TO_TAKE_UP, SimBus.STATE.STOP_TO_DISCHARGING_AND_TAKE_UP]:
            if self.schedules[0].event == Vehicle.ACTION.MOVE or \
                    self.schedules[0].period.end < current_time:
                self.schedules.pop(0)

                # update next schedule
                dif_time = current_time - self.schedules[0].period.start
                self.schedules = Schedule.get_shifted_schedules(self.schedules, dif_time)

                self.state = SimBus.STATE.MOVE_TO_BUS_STOP
                self.publish_status()
