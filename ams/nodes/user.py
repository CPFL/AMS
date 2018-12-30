#!/usr/bin/env python
# coding: utf-8

from time import time, sleep

from ams import logger
from ams.helpers import Hook, Condition, Publisher, Subscriber, Event
from ams.helpers import StateMachine as StateMachineHelper
from ams.nodes.event_loop import EventLoop
from ams.structures import User as Structure
from ams.structures import Dispatcher


class User(EventLoop):

    CONST = Structure.CONST
    Config = Structure.Config
    Status = Structure.Status
    Message = Structure.Message

    def __init__(self, config, status, state_machine_path=None):
        super(User, self).__init__(config, status)

        self.user_data["target_user"] = self.config.target_self

        topic = Subscriber.get_user_event_topic(self.config.target_dispatcher, self.config.target_self)
        self.subscribers[topic] = {
            "topic": topic,
            "callback": Subscriber.on_user_event_message,
            "structure": Dispatcher.Message.Event,
            "user_data": self.user_data
        }

        self.state_machine_path = state_machine_path

    def loop(self):
        resource = StateMachineHelper.load_resource(self.state_machine_path)
        state_machine_data = StateMachineHelper.create_data(resource)
        StateMachineHelper.attach(
            state_machine_data,
            [
                Hook.set_user_goal_at_random,
                Publisher.publish_user_status,
                Condition.user_hired_vehicle,
                Condition.user_state_timeout
            ],
            self.user_data
        )

        while True:
            start_time = time()

            status = Hook.get_status(self.user_data["kvs_client"], self.user_data["target_user"], self.Status)
            if status.state == self.CONST.STATE.END:
                break

            event = Hook.get_event(self.user_data["kvs_client"], self.user_data["target_user"])
            updated_flag = False
            if event is not None:
                if Hook.initialize_event(self.user_data["kvs_client"], self.user_data["target_user"]):
                    updated_flag = StateMachineHelper.update_state(state_machine_data, event.name)
            if not updated_flag:
                updated_flag = StateMachineHelper.update_state(state_machine_data, None)

            if updated_flag:
                new_status = Hook.get_status(
                    self.user_data["kvs_client"], self.user_data["target_user"], self.Status)
                new_status.updated_at = Event.get_time()
                new_status.state = StateMachineHelper.get_state(state_machine_data)
                Hook.set_status(self.user_data["kvs_client"], self.user_data["target_user"], new_status)
                logger.info("User Event: {}, State: {} -> {}".format(event, status.state, new_status.state))

            sleep(max(0, self.dt - (time() - start_time)))
