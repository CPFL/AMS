#!/usr/bin/env python
# coding: utf-8

from time import time, sleep

from ams import logger
from ams.helpers import Hook, Condition, Publisher, Subscriber, Event
from ams.helpers import StateMachine as StateMachineHelper
from ams.nodes.event_loop import EventLoop
from ams.structures import TrafficSignal as Structure
from ams.structures import TrafficSignalController


class TrafficSignal(EventLoop):

    CONST = Structure.CONST
    Config = Structure.Config
    Status = Structure.Status
    Message = Structure.Message

    def __init__(self, config, status, state_machine_path=None):
        super(TrafficSignal, self).__init__(config, status)

        self.user_data["target_traffic_signal"] = self.config.target_self

        topic = Subscriber.get_traffic_signal_event_topic(
            self.config.target_traffic_signal_controller, self.config.target_self)
        self.subscribers[topic] = {
            "topic": topic,
            "callback": Subscriber.on_traffic_signal_event_message,
            "structure": TrafficSignalController.Message.Event,
            "user_data": self.user_data
        }

        topic = Subscriber.get_traffic_signal_cycle_topic(
            self.config.target_traffic_signal_controller, self.config.target_self)
        self.subscribers[topic] = {
            "topic": topic,
            "callback": Subscriber.on_traffic_signal_cycle_message,
            "structure": TrafficSignalController.Message.Cycle,
            "user_data": self.user_data
        }

        topic = Subscriber.get_traffic_signal_schedule_topic(
            self.config.target_traffic_signal_controller, self.config.target_self)
        self.subscribers[topic] = {
            "topic": topic,
            "callback": Subscriber.on_traffic_signal_schedule_message,
            "structure": TrafficSignalController.Message.Schedule,
            "user_data": self.user_data
        }

        self.state_machine_path = state_machine_path

    def loop(self):
        resource = StateMachineHelper.load_resource(self.state_machine_path)
        state_machine_data = StateMachineHelper.create_data(resource)
        StateMachineHelper.attach(
            state_machine_data,
            [
                Hook.update_traffic_signal_light_color,
                Hook.update_traffic_signal_next_light_color,
                Hook.remove_old_traffic_signal_event_from_schedule,
                Hook.append_traffic_signal_event_to_schedule_from_cycle,
                Hook.replace_schedule,
                Hook.initialize_received_schedule,
                Publisher.publish_traffic_signal_status,
                Condition.schedule_exists,
                Condition.received_schedule_exists,
                Condition.traffic_signal_cycle_exists,
                Condition.event_close_to_the_end,
                Condition.schedule_close_to_the_end,
                Condition.traffic_signal_light_color_updated,
                Condition.traffic_signal_next_light_color_updated,
                Condition.schedule_replaced,
                Condition.traffic_signal_state_timeout,
                Condition.received_schedule_initialized
            ],
            self.user_data
        )

        while True:
            start_time = time()

            status = Hook.get_status(self.user_data["kvs_client"], self.user_data["target_traffic_signal"], self.Status)
            if status.state == TrafficSignal.CONST.STATE.END:
                break

            event = Hook.get_event(self.user_data["kvs_client"], self.user_data["target_traffic_signal"])
            updated_flag = False
            if event is not None:
                if Hook.initialize_event(self.user_data["kvs_client"], self.user_data["target_traffic_signal"]):
                    updated_flag = StateMachineHelper.update_state(state_machine_data, event.name)
            if not updated_flag:
                updated_flag = StateMachineHelper.update_state(state_machine_data, None)

            if updated_flag:
                new_status = Hook.get_status(
                    self.user_data["kvs_client"], self.user_data["target_traffic_signal"], self.Status)
                new_status.updated_at = Event.get_time()
                new_status.state = StateMachineHelper.get_state(state_machine_data)
                Hook.set_status(self.user_data["kvs_client"], self.user_data["target_traffic_signal"], new_status)
                logger.info("TrafficSignal Event: {}, State: {} -> {}".format(event, status.state, new_status.state))
            else:
                if status.state == TrafficSignal.CONST.STATE.WAITING_EVENT:
                    sleep(max(0, self.dt - (time() - start_time)))
