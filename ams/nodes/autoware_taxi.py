#!/usr/bin/env python
# coding: utf-8

from time import time
from ams import Schedule
from ams.nodes import Vehicle, Autoware
from ams.structures import AUTOWARE_TAXI
from pprint import PrettyPrinter
pp = PrettyPrinter(indent=2).pprint


class AutowareTaxi(Autoware):

    CONST = AUTOWARE_TAXI

    def __init__(self, name, waypoint, arrow, route, waypoint_id, arrow_code, velocity, dt=1.0):
        super().__init__(name, waypoint, arrow, route, waypoint_id, arrow_code, velocity, dt)
        self.state = AUTOWARE_TAXI.STATE.STANDBY

    def is_achieved(self):
        if len(self.current_poses) - self.pose_index <= 3:
            return True
        else:
            return False

    def update_status(self):
        current_time = time()
        if self.state == AUTOWARE_TAXI.STATE.STANDBY:
            if 1 < len(self.schedules):
                self.schedules.pop(0)

                self.on_start_moving()

                # update next schedule
                dif_time = current_time - self.schedules[0]["route"]["start"]["time"]
                self.schedules = Schedule.get_shifted_schedules(self.schedules, dif_time)

                self.state = AUTOWARE_TAXI.STATE.MOVE_TO_USER

        elif self.state == AUTOWARE_TAXI.STATE.MOVE_TO_USER:
            if self.is_achieved():
                self.schedules.pop(0)

                # update next schedule
                new_start_time = time()
                dif_time = new_start_time - self.schedules[0]["route"]["start"]["time"]
                self.schedules = Schedule.get_shifted_schedules(self.schedules, dif_time)

                self.state = AUTOWARE_TAXI.STATE.STOP_FOR_PICKING_UP
            else:
                arrow_codes = self.schedules[0]["route"]["arrow_codes"]
                i_s = 0
                if self.arrow_code in arrow_codes:
                    i_s = arrow_codes.index(self.arrow_code)
                self.schedules[0]["route"]["arrow_codes"] = arrow_codes[i_s:]

        elif self.state == AUTOWARE_TAXI.STATE.STOP_FOR_PICKING_UP:
            if self.schedules[0]["event"] == Vehicle.ACTION.MOVE or \
                    self.schedules[0]["route"]["goal"]["time"] < current_time:
                self.schedules.pop(0)

                self.on_start_moving()

                # update next schedule
                dif_time = current_time - self.schedules[0]["route"]["start"]["time"]
                self.schedules = Schedule.get_shifted_schedules(self.schedules, dif_time)

                self.state = AUTOWARE_TAXI.STATE.MOVE_TO_USER_DESTINATION
        elif self.state == AUTOWARE_TAXI.STATE.MOVE_TO_USER_DESTINATION:
            if self.is_achieved():
                self.schedules.pop(0)

                # update next schedule
                new_start_time = time()
                dif_time = new_start_time - self.schedules[0]["route"]["start"]["time"]
                self.schedules = Schedule.get_shifted_schedules(self.schedules, dif_time)

                self.state = AUTOWARE_TAXI.STATE.STOP_FOR_DISCHARGING
            else:
                arrow_codes = self.schedules[0]["route"]["arrow_codes"]
                i_s = 0
                if self.arrow_code in arrow_codes:
                    i_s = arrow_codes.index(self.arrow_code)
                self.schedules[0]["route"]["arrow_codes"] = arrow_codes[i_s:]

        elif self.state == AUTOWARE_TAXI.STATE.STOP_FOR_DISCHARGING:
            if self.schedules[0]["route"]["goal"]["time"] < current_time:
                self.schedules.pop(0)

                # update next schedule
                dif_time = current_time - self.schedules[0]["route"]["start"]["time"]
                self.schedules = Schedule.get_shifted_schedules(self.schedules, dif_time)

                self.state = AUTOWARE_TAXI.STATE.STANDBY

        elif self.state == AUTOWARE_TAXI.STATE.MOVE_TO_STANDBY:
            pass
