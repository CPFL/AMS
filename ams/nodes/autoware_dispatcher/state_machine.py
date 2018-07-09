#!/usr/bin/env python
# coding: utf-8

from ams.nodes.dispatcher import StateMachine as DispatcherStateMachine
from ams.nodes.autoware import CONST as AUTOWARE
from ams.nodes.autoware_dispatcher import CONST, Helper, Publisher


class Condition(DispatcherStateMachine.Transition.Condition):
    pass


class BeforeHook(DispatcherStateMachine.Transition.BeforeHook):

    Helper = Helper


class AfterHook(DispatcherStateMachine.Transition.AfterHook):

    Helper = Helper
    Publisher = Publisher


class Transition(DispatcherStateMachine.Transition):

    VEHICLE = AUTOWARE
    DISPATCHER = CONST

    Helper = Helper
    Condition = Condition
    BeforeHook = BeforeHook
    AfterHook = AfterHook


class StateMachine(DispatcherStateMachine):

    DISPATCHER = CONST

    Helper = Helper
    Transition = Transition
