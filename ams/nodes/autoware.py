#!/usr/bin/env python
# coding: utf-8

from time import time, sleep

from ams.helpers import Hook, Condition, Publisher, Subscriber
from ams.helpers import StateMachine as StateMachineHelper
from ams.nodes.event_loop import EventLoop
from ams.structures import Autoware as Structure


class Autoware(EventLoop):

    Config = Structure.Config
    Status = Structure.Status
    Message = Structure.Message
    ROSMessage = Structure.ROSMessage

    def __init__(self, config, status, state_machine_path=None, identifiable=False):
        super(Autoware, self).__init__(config, status)

        self.user_data["target_autoware"] = self.config.target_self
        self.user_data["identifiable"] = identifiable

        if identifiable:
            topic = Subscriber.get_lane_array_rostopic(self.config.target_self)
        else:
            topic = Subscriber.get_lane_array_rostopic()
        self.subscribers[topic] = {
            "topic": topic,
            "callback": Subscriber.on_lane_array,
            "structure": self.Status.LaneArray,
            "user_data": self.user_data
        }

        if identifiable:
            topic = Subscriber.get_state_cmd_rostopic(self.config.target_self)
        else:
            topic = Subscriber.get_state_cmd_rostopic()
        self.subscribers[topic] = {
            "topic": topic,
            "callback": Subscriber.on_state_cmd,
            "structure": self.Status.StateCMD,
            "user_data": self.user_data
        }

        if identifiable:
            topic = Subscriber.get_stop_waypoint_index_rostopic(self.config.target_self)
        else:
            topic = Subscriber.get_stop_waypoint_index_rostopic()
        self.subscribers[topic] = {
            "topic": topic,
            "callback": Subscriber.on_stop_waypoint_index,
            "structure": self.Status.StopWaypointIndex,
            "user_data": self.user_data
        }

        self.state_machine_path = state_machine_path

    @classmethod
    def set_autoware_status(cls, kvs_client, target, value):
        set_flag = Hook.set_current_pose(kvs_client, target, value.current_pose)
        set_flag *= Hook.set_vehicle_location(kvs_client, target, value.vehicle_location)
        set_flag *= Hook.set_state_cmd(kvs_client, target, value.state_cmd)
        set_flag *= Hook.set_lane_array(kvs_client, target, value.lane_array)
        set_flag *= Hook.set_decision_maker_state(kvs_client, target, value.decision_maker_state)
        set_flag *= Hook.set_stop_waypoint_index(kvs_client, target, value.stop_waypoint_index)
        if not set_flag:
            raise IOError("cannot set autoware status.")

    def loop(self):
        Hook.set_status(self.user_data["kvs_client"], self.user_data["target_autoware"], self.status)
        self.set_autoware_status(self.user_data["kvs_client"], self.user_data["target_autoware"], self.status)

        resource = StateMachineHelper.load_resource(self.state_machine_path)
        state_machine_data = StateMachineHelper.create_data(resource)
        StateMachineHelper.attach(
            state_machine_data,
            [
                Hook.initialize_vehicle_location,
                Hook.initialize_state_cmd,
                Hook.initialize_lane_array,
                Hook.initialize_received_lane_array,
                Hook.update_lane_array,
                Hook.update_vehicle_location,
                Hook.update_current_pose,
                Condition.received_lane_array_exists,
                Condition.lane_array_initialized,
                Condition.received_lane_array_initialized,
                Condition.lane_array_updated,
                Condition.vehicle_location_initialized,
                Condition.state_cmd_is_expected,
                Condition.vehicle_location_is_end_point
            ],
            self.user_data
        )

        while True:
            start_time = time()
            state_cmd = Hook.get_state_cmd(self.user_data["kvs_client"], self.user_data["target_autoware"])
            event = state_cmd.data if state_cmd is not None else None
            updated_flag = False
            if event is not None:
                updated_flag = StateMachineHelper.update_state(state_machine_data, event)
            if not updated_flag:
                StateMachineHelper.update_state(state_machine_data, None)

            Hook.set_decision_maker_state(
                self.user_data["kvs_client"], self.user_data["target_autoware"],
                self.Status.DecisionMakerState.new_data(
                    data=StateMachineHelper.get_state(state_machine_data)))

            Publisher.publish_ros_current_pose(
                self.user_data["pubsub_client"], self.user_data["kvs_client"],
                self.user_data["target_autoware"], wait=True, identifiable=self.user_data["identifiable"])

            Publisher.publish_ros_vehicle_location(
                self.user_data["pubsub_client"], self.user_data["kvs_client"],
                self.user_data["target_autoware"], wait=True, identifiable=self.user_data["identifiable"])

            Publisher.publish_ros_decision_maker_state(
                self.user_data["pubsub_client"], self.user_data["kvs_client"],
                self.user_data["target_autoware"], wait=True, identifiable=self.user_data["identifiable"])

            sleep(max(0, self.dt - (time()-start_time)))
