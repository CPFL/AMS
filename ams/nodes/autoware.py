#!/usr/bin/env python
# coding: utf-8

from time import time, sleep

from ams import logger
from ams.helpers import Hook, Condition, Publisher, Subscriber
from ams.helpers import StateMachine as StateMachineHelper
from ams.nodes.event_loop import EventLoop
from ams.structures import AutowareInterface
from ams.structures import Autoware as Structure


class Autoware(EventLoop):

    Config = Structure.Config
    Status = Structure.Status
    Message = Structure.Message
    ROSMessage = Structure.ROSMessage

    def __init__(self, config, status, state_machine_path=None):
        super(Autoware, self).__init__(config, status)

        self.user_data["target_autoware"] = self.config.target_self

        topic = AutowareInterface.CONST.TOPIC.LANE_ARRAY
        self.subscribers[topic] = {
            "topic": topic,
            "callback": Subscriber.on_lane_array,
            "structure": self.Status.LaneArray,
            "user_data": self.user_data
        }

        topic = AutowareInterface.CONST.TOPIC.STATE_CMD
        self.subscribers[topic] = {
            "topic": topic,
            "callback": Subscriber.on_state_cmd,
            "structure": self.Status.StateCMD,
            "user_data": self.user_data
        }

        topic = AutowareInterface.CONST.TOPIC.LIGHT_COLOR
        self.subscribers[topic] = {
            "topic": topic,
            "callback": Subscriber.on_light_color,
            "structure": self.Status.LightColor,
            "user_data": self.user_data
        }

        self.state_machine_path = state_machine_path

    def loop(self):
        Hook.set_autoware_status(self.user_data["kvs_client"], self.user_data["target_autoware"], self.status)

        resource = StateMachineHelper.load_resource(self.state_machine_path)
        state_machine_data = StateMachineHelper.create_data(resource)
        StateMachineHelper.attach(
            state_machine_data,
            [
                Hook.initialize_vehicle_location,
                Hook.initialize_state_cmd,
                Hook.initialize_lane_array,
                Hook.update_vehicle_location,
                Hook.update_current_pose,
                Condition.lane_array_exists,
                Condition.vehicle_location_initialized,
                Condition.state_cmd_is_engage,
                Condition.state_cmd_initialized,
                Condition.vehicle_location_is_end_point
            ],
            self.user_data
        )

        while True:
            start_time = time()
            state_machine_data["variables"]["config"] = Hook.get_config(
                self.user_data["kvs_client"], self.user_data["target_autoware"], self.Config)
            state_machine_data["variables"]["status"] = Hook.get_autoware_status(
                self.user_data["kvs_client"], self.user_data["target_autoware"])

            state_cmd = Hook.get_state_cmd(self.user_data["kvs_client"], self.user_data["target_autoware"])
            event = state_cmd.data if state_cmd is not None else None
            logger.info("update_state: {}, {}".format(StateMachineHelper.get_state(state_machine_data), event))
            updated_flag = False
            if event is not None:
                updated_flag = StateMachineHelper.update_state(state_machine_data, event)
                if updated_flag:
                    logger.info(
                        Hook.get_status(
                            self.user_data["kvs_client"], self.user_data["target_autoware"], self.Status
                        ).decision_maker_state.data
                    )
            if not updated_flag:
                if StateMachineHelper.update_state(state_machine_data, None):
                    logger.info(
                        Hook.get_status(
                            self.user_data["kvs_client"], self.user_data["target_autoware"], self.Status
                        ).decision_maker_state.data
                    )

            Hook.set_decision_maker_state(
                self.user_data["kvs_client"], self.user_data["target_autoware"],
                self.Status.DecisionMakerState.new_data(
                    data=StateMachineHelper.get_state(state_machine_data)))

            Publisher.publish_ros_current_pose(
                self.user_data["pubsub_client"], self.user_data["kvs_client"],
                self.user_data["target_autoware"], wait=True)

            Publisher.publish_ros_vehicle_location(
                self.user_data["pubsub_client"], self.user_data["kvs_client"],
                self.user_data["target_autoware"], wait=True)

            Publisher.publish_ros_decision_maker_state(
                self.user_data["pubsub_client"], self.user_data["kvs_client"],
                self.user_data["target_autoware"], wait=True)

            sleep(max(0, self.dt - (time()-start_time)))
