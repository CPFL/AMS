#!/usr/bin/env python
# coding: utf-8

from time import sleep

from ams.helpers import Simulator, Schedule
from ams.nodes.vehicle import EventLoop as VehicleEventLoop
from ams.nodes.vehicle import CONST as VEHICLE
from ams.nodes.traffic_signal import Message as TrafficSignalMessage
from ams.nodes.sim_car_dispatcher import Message as DispatcherMessage
from ams.nodes.sim_car import CONST, Structure, Message, Helper, Publisher, StateMachine, Subscriber


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
        self.velocity = self.CONST.DEFAULT_VELOCITY

    def __set_sim_car_subscriber(self):
        topic = self.Subscriber.get_vehicle_location_topic()
        self.subscribers[topic] = {
            "topic": topic,
            "callback": self.Subscriber.on_vehicle_location_message,
            "structure": self.Message.Location,
            "user_data": self.user_data
        }

        topic = self.Subscriber.get_traffic_signal_status_topic()
        self.subscribers[topic] = {
            "topic": topic,
            "callback": self.Subscriber.on_traffic_signal_status_message,
            "structure": TrafficSignalMessage.Status,
            "user_data": self.user_data
        }

    def set_velocity(self, velocity):
        self.velocity = velocity

    def update_pose(self, vehicle_status):
        if vehicle_status.state == self.CONST.STATE.MOVE:
            vehicle_schedules_key, vehicle_schedules = \
                self.Helper.get_vehicle_schedules_key_and_value(self.user_data["kvs_client"], self.target)

            traffic_signal_status_keys = self.Helper.get_all_traffic_signal_status_keys(
                self.user_data["kvs_client"], self.target)
            traffic_signal_statuses = \
                self.Helper.get_traffic_signal_statuses(self.user_data["kvs_client"],
                                                        traffic_signal_status_keys)
            other_vehicle_location_keys = self.Helper.get_other_vehicle_location_keys(
                self.user_data["kvs_client"], self.target)
            other_vehicle_locations = self.Helper.get_vehicle_locations(
                self.user_data["kvs_client"], other_vehicle_location_keys)

            route = Schedule.get_schedule_by_id(vehicle_schedules, vehicle_status.schedule_id).route
            Simulator.update_pose(
                self.user_data["maps_client"], vehicle_status, route, traffic_signal_statuses,
                other_vehicle_locations)
            # Simulator.update_velocity(self.user_data["maps_client"], vehicle_status)

            self.Helper.set_vehicle_status(self.user_data["kvs_client"], self.target, vehicle_status)

    def start(self):
        self.__set_vehicle_subscriber()
        self.__set_sim_car_subscriber()

        self.__connect_and_subscribe()

        if self.initials["config"] is not None:
            self.Helper.set_vehicle_config(self.user_data["kvs_client"], self.target, self.initials["config"])
        if self.initials["status"] is not None:
            self.Helper.set_vehicle_status(self.user_data["kvs_client"], self.target, self.initials["status"])

        try:
            while True:
                _ = self.StateMachine.update_vehicle_state(
                    self.target, self.user_data["kvs_client"], self.user_data["mqtt_client"],
                    self.user_data["maps_client"])

                vehicle_config_key, vehicle_config = \
                    self.Helper.get_vehicle_config_key_and_value(self.user_data["kvs_client"], self.target)
                vehicle_status_key, vehicle_status = \
                    self.Helper.get_vehicle_status_key_and_value(self.user_data["kvs_client"], self.target)

                self.update_pose(vehicle_status)

                self.Publisher.publish_vehicle_location_message(
                    self.user_data["mqtt_client"], self.target, vehicle_status.location)
                self.Publisher.publish_vehicle_status(
                    self.user_data["mqtt_client"], self.target, vehicle_status, vehicle_config.target_dispatcher)

                if vehicle_status.state == VEHICLE.STATE.END_PROCESSING:
                    break

                sleep(self.dt)

        except KeyboardInterrupt:
            _, vehicle_status = \
                self.Helper.get_vehicle_status_key_and_value(self.user_data["kvs_client"], self.target)
            StateMachine.Transition.to_end_processing(vehicle_status)
            self.Helper.set_vehicle_status(self.user_data["kvs_client"], self.target, vehicle_status)

        self.user_data["mqtt_client"].disconnect()
