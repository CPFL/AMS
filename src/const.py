#!/usr/bin/env python
# coding: utf-8
from sys import float_info


class CONST(object):
    HOST = 'localhost'
    PORT = 1883
    KEEPALIVE = 60

    BL_PER_METER = 0.000009925558312655087
    METER_PER_BL = 100749.999999999998464
    FLOAT_MAX = float_info.max

    class TOPICS(object):
        EVENTLOOP = "eventLoop"
        # RIDE_SCHEDULE = "rideSchedule"
        USER_STATUS = "userStatus"  # userID, startSpotID, goalSpotID, state, vehicleID
        VEHICLE_STATUS = "vehicleStatus"  # json
        TRAFFICSIGNAL_STATUS = "trafficSignalStatus"
        FLEET_STATUS = "fleetStatus"  # json(users, vehicles)
        DIRECTIONS = "vehicleDirections"  # json(vehicleID, schedules)
        TRAFFICSIGNAL_DIRECTIONS = "trafficSignalDirections"  # json(traffocSignalID, schedules)
        NOTICE = "notice"  # vehicleID, stoppedSpotID

    class USER_STATE(object):
        WAITING = "waiting"
        # GETTING_ON = "gettingOn"
        MOVING = "moving"
        # GETTING_OUT = "gettingOut"
        GOT_OUT = "gotOut"

    class VEHICLE_STATE(object):
        RUN = "run"
        # STAY = "stay"

        MOVE = "move"
        STOP = "stop"
        STANDBY = "standBy"

    class TRAFFICSIGNAL_STATE(object):
        GREEN = "green"
        YELLOW = "yellow"
        RED = "red"
