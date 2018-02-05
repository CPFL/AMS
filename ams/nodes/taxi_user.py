#!/usr/bin/env python
# coding: utf-8

from ams.nodes import User


class TaxiUser(User):

    class STATE(object):
        CALLING = "calling"
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

    def __init__(self, name, dt=1.0):
        super().__init__(name, dt)

    def update_status(self):
        # print(self.name, self.state, self.schedules[0].event)
        if self.state == User.STATE.LOG_IN:
            self.state = TaxiUser.STATE.CALLING
            self.publish_status()
        if self.state == TaxiUser.STATE.CALLING:
            if self.schedules[0].event == TaxiUser.ACTION.WAIT:
                self.state = TaxiUser.STATE.WAITING
                self.schedules[0].event = None
                self.publish_status()
        elif self.state == TaxiUser.STATE.WAITING:
            if self.schedules[0].event == TaxiUser.ACTION.GET_ON:
                self.state = TaxiUser.STATE.GETTING_ON
                self.schedules[0].event = None
                self.publish_status()
        elif self.state == TaxiUser.STATE.GETTING_ON:
            self.state = TaxiUser.STATE.GOT_ON
            self.publish_status()
        elif self.state == TaxiUser.STATE.GOT_ON:
            if self.schedules[0].event == TaxiUser.EVENT.MOVE_VEHICLE:
                self.state = TaxiUser.STATE.MOVING
                self.schedules[0].event = None
                self.publish_status()
            if self.schedules[0].event == TaxiUser.ACTION.GET_OUT:
                self.state = TaxiUser.STATE.GETTING_OUT
                self.schedules[0].event = None
                self.publish_status()
        elif self.state == TaxiUser.STATE.MOVING:
            if self.schedules[0].event == TaxiUser.ACTION.GET_OUT:
                self.state = TaxiUser.STATE.GETTING_OUT
                self.schedules[0].event = None
                self.publish_status()
        elif self.state == TaxiUser.STATE.GETTING_OUT:
            self.state = TaxiUser.STATE.GOT_OUT
            self.publish_status()
        elif self.state == TaxiUser.STATE.GOT_OUT:
            self.state = User.STATE.LOG_OUT
            self.publish_status()
