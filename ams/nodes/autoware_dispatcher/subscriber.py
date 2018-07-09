#!/usr/bin/env python
# coding: utf-8

from ams.nodes.dispatcher import Subscriber as DispatcherSubscriber
from ams.nodes.autoware import CONST as AUTOWARE
from ams.nodes.autoware_dispatcher import Helper, StateMachine


class Subscriber(DispatcherSubscriber):

    VEHICLE = AUTOWARE

    Helper = Helper
    StateMachine = StateMachine
