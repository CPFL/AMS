#!/usr/bin/env python
# coding: utf-8

from time import time
from uuid import uuid4 as uuid

from ams.helpers import Target
from ams.nodes.vehicle import EventLoop as VehicleEventLoop
from ams.nodes.traffic_signal import Message as TrafficSignalMessage
from ams.nodes.autoware_dispatcher import Message as DispatcherMessage
from ams.nodes.autoware import CONST, Structure, Message, Helper, Publisher, StateMachine, Subscriber


class EventLoop(VehicleEventLoop):

    CONST = CONST
    Structure = Structure
    Message = Message
    Helper = Helper
    Publisher = Publisher
    StateMachine = StateMachine
    Subscriber = Subscriber

    DispatcherMessage = DispatcherMessage

    def __init__(self, _id, group=CONST.NODE_NAME):
        super().__init__(_id, group)

        self.upper_distance_from_stopline = CONST.DEFAULT_UPPER_DISTANCE_FROM_STOPLINE

    def __set_autoware_subscriber(self):
        topic = self.Subscriber.get_closest_waypoint_topic(self.initials["config"].target_ros, self.target)
        self.subscribers[topic] = {
            "topic": topic,
            "callback": self.Subscriber.on_closest_waypoint_ros_message,
            "structure": self.Structure.ROSMessage.ClosestWaypoint,
            "user_data": self.user_data
        }

        topic = self.Subscriber.get_current_pose_topic(self.initials["config"].target_ros, self.target)
        self.subscribers[topic] = {
            "topic": topic,
            "callback": self.Subscriber.on_current_pose_ros_message,
            "structure": self.Structure.ROSMessage.CurrentPose,
            "user_data": self.user_data
        }

        topic = self.Subscriber.get_decision_maker_state_topic(self.initials["config"].target_ros, self.target)
        self.subscribers[topic] = {
            "topic": topic,
            "callback": self.Subscriber.on_decision_maker_state_ros_message,
            "structure": self.Structure.ROSMessage.DecisionMakerState,
            "user_data": self.user_data
        }

        topic = self.Subscriber.get_traffic_signal_status_topic()
        self.subscribers[topic] = {
            "topic": topic,
            "callback": self.Subscriber.on_traffic_signal_status_message,
            "structure": TrafficSignalMessage.Status,
            "user_data": self.user_data
        }

    def set_initial_config(
        self, activation, target_ros_id=None,
        upper_distance_from_stopline=CONST.DEFAULT_UPPER_DISTANCE_FROM_STOPLINE
    ):
        self.initials["config"] = self.Structure.Config.new_data(
            target_dispatcher=None,
            activation=activation,
            upper_distance_from_stopline=upper_distance_from_stopline,
            target_ros=Target.new_target(
                CONST.ROS.NODE_NAME,
                target_ros_id if target_ros_id is not None else str(uuid())),
        )

    def set_initial_status(
            self, state=CONST.STATE.START_PROCESSING, schedule_id=None, location=None, pose=None, velocity=0.0,
            route_code=None, current_pose=None, closest_waypoint=None, decision_maker_state=None):
        self.initials["status"] = self.Structure.Status.new_data(
            state=state,
            schedule_id=schedule_id,
            location=location,
            pose=pose,
            velocity=velocity,
            route_code=route_code,
            current_pose=current_pose,
            closest_waypoint=closest_waypoint,
            decision_maker_state=decision_maker_state,
            updated_at=self.Helper.get_current_time()
        )

    def start(self):
        self.__set_vehicle_subscriber()
        self.__set_autoware_subscriber()

        self.__connect_and_subscribe()

        self.Helper.set_vehicle_config(self.user_data["kvs_client"], self.target, self.initials["config"])
        self.Helper.set_vehicle_status(self.user_data["kvs_client"], self.target, self.initials["status"])

    def stop(self):
        self.user_data["mqtt_client"].disconnect()
