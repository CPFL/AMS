#!/usr/bin/env python
# coding: utf-8

from ams.nodes.sim_car_dispatcher import Helper, Publisher
from ams.nodes.dispatcher import StateMachine as DispatcherStateMachine
from ams.nodes.sim_car import CONST as SIM_CAR
from ams.nodes.sim_car_dispatcher import CONST


class Condition(DispatcherStateMachine.Transition.Condition):
    pass


class BeforeHook(DispatcherStateMachine.Transition.BeforeHook):

    Helper = Helper


class AfterHook(DispatcherStateMachine.Transition.AfterHook):

    Helper = Helper
    Publisher = Publisher


class Transition(DispatcherStateMachine.Transition):

    VEHICLE = SIM_CAR
    DISPATCHER = CONST

    Helper = Helper
    Condition = Condition
    BeforeHook = BeforeHook
    AfterHook = AfterHook


class StateMachine(DispatcherStateMachine):

    DISPATCHER = CONST

    Helper = Helper
    Transition = Transition
