#!/usr/bin/env python
# coding: utf-8

from ams.nodes.dispatcher import Subscriber as DispatcherSubscriber
from ams.nodes.sim_car import CONST as SIM_CAR
from ams.nodes.sim_car_dispatcher import Helper, StateMachine


class Subscriber(DispatcherSubscriber):

    VEHICLE = SIM_CAR

    Helper = Helper
    StateMachine = StateMachine
