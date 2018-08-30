#!/usr/bin/env python
# coding: utf-8

from time import sleep

from ams.nodes.base import EventLoop as BaseEventLoop
from ams.nodes.infra import CONST, Structure, Message, Helper, Publisher, StateMachine, Subscriber


class EventLoop(BaseEventLoop):

    CONST = CONST
    Structure = Structure
    Message = Message
    Helper = Helper
    Publisher = Publisher
    StateMachine = StateMachine
    Subscriber = Subscriber

    def __init__(self, config, status):
        super(EventLoop, self).__init__(config, status)
        self.user_data["target_roles"]["infra"] = self.user_data["target_roles"]["self"]

    def loop(self):
        while True:
            _ = self.StateMachine.update_state(
                clients=self.user_data["clients"], target_roles=self.user_data["target_roles"])

            status = self.Helper.get_status(self.user_data["clients"], self.user_data["target_roles"])
            self.Publisher.publish_status(self.user_data["clients"], self.user_data["target_roles"], status)

            if status.state == self.CONST.STATE.END_PROCESSING:
                break

            sleep(self.dt)
