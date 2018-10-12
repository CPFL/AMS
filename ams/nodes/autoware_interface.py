#!/usr/bin/env python
# coding: utf-8

from ams import logger
from ams.helpers import Subscriber
from ams.nodes.event_loop import EventLoop
from ams.structures import Vehicle, Autoware
from ams.structures import AutowareInterface as Structure


class AutowareInterface(EventLoop):

    Config = Structure.Config
    Message = Structure.Message

    def __init__(self, config, ros_msgs=None):
        logger.info("AAAA" + logger.pformat(self.Config.get_schema()))
        super(AutowareInterface, self).__init__(config)

        self.user_data["target_autoware"] = self.config.target_self
        self.user_data["target_vehicle"] = self.config.target_vehicle
        self.user_data["lane_array_structure"] = ros_msgs["LaneArray"]
        self.user_data["state_cmd_structure"] = ros_msgs["String"]
        self.user_data["light_color_structure"] = ros_msgs["traffic_light"]

        topic = Subscriber.get_route_code_message_topic(
            self.user_data["target_vehicle"], self.user_data["target_autoware"])
        self.subscribers[topic] = {
            "topic": topic,
            "callback": Subscriber.on_route_code_message_publish_lane_array,
            "structure": Vehicle.Message.RouteCode,
            "user_data": self.user_data
        }

        topic = Subscriber.get_state_cmd_topic(self.user_data["target_vehicle"], self.user_data["target_autoware"])
        self.subscribers[topic] = {
            "topic": topic,
            "callback": Subscriber.on_state_cmd_publish,
            "structure": None,
            "user_data": self.user_data
        }

        topic = Subscriber.get_light_color_topic(self.user_data["target_vehicle"], self.user_data["target_autoware"])
        self.subscribers[topic] = {
            "topic": topic,
            "callback": Subscriber.on_light_color_publish,
            "structure": None,
            "user_data": self.user_data
        }

        topic = Autoware.CONST.TOPIC.CURRENT_POSE
        self.ros_subscribers[topic] = {
            "topic": topic,
            "callback": Subscriber.on_current_pose_publish,
            "structure": ros_msgs["PoseStamped"],
            "user_data": self.user_data
        }

        topic = Autoware.CONST.TOPIC.VEHICLE_LOCATION
        self.ros_subscribers[topic] = {
            "topic": topic,
            "callback": Subscriber.on_vehicle_location_publish_route_point,
            "structure": ros_msgs["Int32"],
            "user_data": self.user_data
        }

        topic = Autoware.CONST.TOPIC.DECISION_MAKER_STATE
        self.ros_subscribers[topic] = {
            "topic": topic,
            "callback": Subscriber.on_decision_maker_state_publish,
            "structure": ros_msgs["String"],
            "user_data": self.user_data
        }
