#!/usr/bin/env python
# coding: utf-8

from ams import Topic, Target
from ams.nodes import User
from ams.messages import VehicleStatus
from ams.structures import SIM_BUS, VEHICLE, BUS_USER, USER


class BusUser(User):

    CONST = BUS_USER

    def __init__(self, name, dt=1.0):
        super().__init__(name, dt)
        self.target_start_bus_stop = None
        self.target_goal_bus_stop = None
        self.bus_id = None
        self.bus_statuses = {}

        self.topicBusState = Topic()
        self.topicBusState.set_root(VEHICLE.TOPIC.PUBLISH)

        self.add_on_message_function(self.update_bus_status)
        self.set_subscriber(self.topicBusState.root+"/#")

    def update_bus_status(self, _client, _userdata, topic, payload):
        if self.topicBusState.root in topic and "/schedules" not in topic:
            bus_id = self.topicBusState.get_id(topic)
            self.bus_statuses[bus_id] = VehicleStatus.new_data(**self.topicBusState.unserialize(payload))

    def is_bus_arrived(self, target_bus_stop):
        for bus_id in self.bus_statuses:
            targets_bus_statuses = list(filter(
                lambda x: Target.is_same_id(x, target_bus_stop),
                self.bus_statuses[bus_id].schedule.targets))
            print("BusUser:", targets_bus_statuses, self.bus_statuses[bus_id].schedule.event)
            if 0 < len(targets_bus_statuses):
                if self.bus_statuses[bus_id].schedule.event in [
                        SIM_BUS.SCHEDULE.STOP_TO_TAKE_UP, SIM_BUS.SCHEDULE.STOP_TO_DISCHARGE_AND_TAKE_UP]:
                    self.bus_id = bus_id
                    # todo: unsbscribe # and subscribe target bus
                    return True
        return False

    def is_bus_approached_target_bus_stop(self, target_bus_stop):
        return 0 < len(list(filter(
            lambda x: Target.is_same_id(target_bus_stop, x),
            self.bus_statuses[self.bus_id].schedule.targets
        )))

    def update_status(self):
        if self.target_start_bus_stop is None:
            if self.trip_schedules is not None:
                self.target_start_bus_stop = \
                    list(filter(
                        lambda x: x.group == BUS_USER.TARGET_GROUP.START_BUS_STOP, self.trip_schedules[0].targets))[0]
                self.target_goal_bus_stop = \
                    list(filter(
                        lambda x: x.group == BUS_USER.TARGET_GROUP.GOAL_BUS_STOP, self.trip_schedules[0].targets))[0]

        print(self.name, self.state, self.schedules[0].event)
        if self.state == USER.STATE.LOG_IN:
            self.state = BUS_USER.STATE.WAITING
            self.publish_status()
        elif self.state == BUS_USER.STATE.ARRIVED_AT_BUS_STOP:
            self.state = BUS_USER.STATE.WAITING
            self.schedules[0].event = None
            self.publish_status()
        elif self.state == BUS_USER.STATE.WAITING:
            if self.is_bus_arrived(self.target_start_bus_stop):
                self.state = BUS_USER.STATE.GETTING_ON
                self.schedules[0].event = None
                self.publish_status()
        elif self.state == BUS_USER.STATE.GETTING_ON:
            self.state = BUS_USER.STATE.GOT_ON
            self.schedules[0].targets.append(Target.new_target(self.bus_id, "SimBus"))
            self.publish_status()
        elif self.state == BUS_USER.STATE.GOT_ON:
            self.state = BUS_USER.STATE.MOVING
            self.schedules[0].targets.append(Target.new_target(self.bus_id, "SimBus"))
            self.publish_status()
        elif self.state == BUS_USER.STATE.MOVING:
            self.schedules[0].targets.append(Target.new_target(self.bus_id, "SimBus"))
            if self.is_bus_approached_target_bus_stop(self.target_goal_bus_stop):
                self.state = BUS_USER.STATE.READY_TO_GET_OUT
                self.schedules[0].event = BUS_USER.ACTION.REQUEST_STOP
                self.publish_status()
        elif self.state == BUS_USER.STATE.READY_TO_GET_OUT:
            self.schedules[0].targets.append(Target.new_target(self.bus_id, "SimBus"))
            if self.is_bus_arrived(self.target_goal_bus_stop):
                self.state = BUS_USER.STATE.GETTING_OUT
                self.publish_status()
        elif self.state == BUS_USER.STATE.GETTING_OUT:
            self.state = BUS_USER.STATE.GOT_OUT
            self.publish_status()
        elif self.state == BUS_USER.STATE.GOT_OUT:
            self.state = USER.STATE.LOG_OUT
            self.publish_status()
