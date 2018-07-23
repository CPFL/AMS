#!/usr/bin/env python
# coding: utf-8

from ams.nodes.dispatcher import Publisher as DispatcherPublisher
from ams.nodes.autoware_dispatcher import CONST, Message, Helper


class Publisher(DispatcherPublisher):

    DISPATCHER = CONST
    Message = Message
    Helper = Helper
