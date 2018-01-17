#!/usr/bin/env python
# coding: utf-8

from time import time
from ams.nodes import SimCar, Vehicle

from pprint import PrettyPrinter
pp = PrettyPrinter(indent=2).pprint


class SimTaxi(SimCar):
    class ACTION(object):
        STANDBY = "standBy"

    class STATE(object):
        STANDBY = "standBy"
        MOVE_TO_USER = "moveToUser"
        STOP_FOR_PICKING_UP = "pickingUp"
        MOVE_TO_USER_DESTINATION = "moveToUserDestination"
        STOP_FOR_DISCHARGING = "discharging"
        MOVE_TO_STANDBY = "moveToDeploy"

    def __init__(self, name, waypoint, arrow, route, intersection, waypoint_id, velocity, schedules=None, dt=1.0):
        super().__init__(name, waypoint, arrow, route, intersection, waypoint_id, velocity, schedules, dt)
        self.state = SimTaxi.STATE.STANDBY

    def update_status(self):
        current_time = time()
        # print("SimTaxi.update_status", self.state)
        if self.state == SimTaxi.STATE.STANDBY:
            # print(SimTaxi.STATE.STANDBY, len(self.schedules))
            if 1 < len(self.schedules):
                self.schedules.pop(0)

                # update next schedule
                dif_time = current_time - self.schedules[0]["start_time"]
                self.schedules[0]["start_time"] += dif_time
                self.schedules[0]["duration"] = dif_time

                # print(self.schedules[0])
                self.state = SimTaxi.STATE.MOVE_TO_USER

        elif self.state == SimTaxi.STATE.MOVE_TO_USER:
            self.update_pose()
            if self.is_achieved():
                # print("*** arrival ***")
                self.waypoint_id = self.schedules[0]["route"]["goal"]["waypoint_id"]
                self.position = self.waypoint.get_position(self.waypoint_id)
                self.yaw = self.arrow.get_heading(self.arrow_code, self.waypoint_id)
                self.schedules.pop(0)

                # update next schedule
                new_start_time = time()
                dif_time = new_start_time - self.schedules[0]["start_time"]
                self.schedules[0]["start_time"] += dif_time
                self.schedules[0]["duration"] = dif_time

                self.state = SimTaxi.STATE.STOP_FOR_PICKING_UP
            else:
                arrow_codes = self.schedules[0]["route"]["arrow_codes"]
                self.schedules[0]["route"]["arrow_codes"] = arrow_codes[arrow_codes.index(self.arrow_code):]

        elif self.state == SimTaxi.STATE.STOP_FOR_PICKING_UP:
            if self.schedules[0]["action"] == Vehicle.ACTION.MOVE or \
                    self.schedules[0]["start_time"] + self.schedules[0]["duration"] < current_time:
                self.schedules.pop(0)

                # update next schedule
                dif_time = current_time - self.schedules[0]["start_time"]
                self.schedules[0]["start_time"] += dif_time
                self.schedules[0]["duration"] = dif_time

                self.state = SimTaxi.STATE.MOVE_TO_USER_DESTINATION
        elif self.state == SimTaxi.STATE.MOVE_TO_USER_DESTINATION:
            self.update_pose()
            if self.is_achieved():
                # print("*** arrival ***")
                self.waypoint_id = self.schedules[0]["route"]["goal"]["waypoint_id"]
                self.position = self.waypoint.get_position(self.waypoint_id)
                self.yaw = self.arrow.get_heading(self.arrow_code, self.waypoint_id)
                self.schedules.pop(0)

                # update next schedule
                new_start_time = time()
                dif_time = new_start_time - self.schedules[0]["start_time"]
                self.schedules[0]["start_time"] += dif_time
                self.schedules[0]["duration"] = dif_time

                self.state = SimTaxi.STATE.STOP_FOR_DISCHARGING
            else:
                arrow_codes = self.schedules[0]["route"]["arrow_codes"]
                self.schedules[0]["route"]["arrow_codes"] = arrow_codes[arrow_codes.index(self.arrow_code):]

        elif self.state == SimTaxi.STATE.STOP_FOR_DISCHARGING:
            if self.schedules[0]["start_time"] + self.schedules[0]["duration"] < current_time:
                self.schedules.pop(0)

                # update next schedule
                dif_time = current_time - self.schedules[0]["start_time"]
                self.schedules[0]["start_time"] += dif_time
                self.schedules[0]["duration"] = dif_time

                self.state = SimTaxi.STATE.STANDBY

        elif self.state == SimTaxi.STATE.MOVE_TO_STANDBY:
            pass
