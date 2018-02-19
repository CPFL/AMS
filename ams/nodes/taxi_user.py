#!/usr/bin/env python
# coding: utf-8

from ams.nodes import User
from ams.structures import TAXI_USER


class TaxiUser(User):

    CONST = TAXI_USER

    def __init__(self, name, dt=1.0):
        super().__init__(name, dt)

    def update_status(self):
        # print(self.name, self.state, self.schedules[0].event)
        if self.state == User.CONST.STATE.LOG_IN:
            self.state = TAXI_USER.STATE.CALLING
            self.publish_status()
        if self.state == TAXI_USER.STATE.CALLING:
            if self.schedules[0].event == TAXI_USER.ACTION.WAIT:
                self.state = TAXI_USER.STATE.WAITING
                self.schedules[0].event = None
                self.publish_status()
        elif self.state == TAXI_USER.STATE.WAITING:
            if self.schedules[0].event == TAXI_USER.ACTION.GET_ON:
                self.state = TAXI_USER.STATE.GETTING_ON
                self.schedules[0].event = None
                self.publish_status()
        elif self.state == TAXI_USER.STATE.GETTING_ON:
            self.state = TAXI_USER.STATE.GOT_ON
            self.publish_status()
        elif self.state == TAXI_USER.STATE.GOT_ON:
            if self.schedules[0].event == TAXI_USER.EVENT.MOVE_VEHICLE:
                self.state = TAXI_USER.STATE.MOVING
                self.schedules[0].event = None
                self.publish_status()
            if self.schedules[0].event == TAXI_USER.ACTION.GET_OUT:
                self.state = TAXI_USER.STATE.GETTING_OUT
                self.schedules[0].event = None
                self.publish_status()
        elif self.state == TAXI_USER.STATE.MOVING:
            if self.schedules[0].event == TAXI_USER.ACTION.GET_OUT:
                self.state = TAXI_USER.STATE.GETTING_OUT
                self.schedules[0].event = None
                self.publish_status()
        elif self.state == TAXI_USER.STATE.GETTING_OUT:
            self.state = TAXI_USER.STATE.GOT_OUT
            self.publish_status()
        elif self.state == TAXI_USER.STATE.GOT_OUT:
            self.state = User.CONST.STATE.LOG_OUT
            self.publish_status()
