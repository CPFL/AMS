#!/usr/bin/env python
# coding: utf-8

from time import time
from copy import deepcopy

from transitions import Machine

from ams import logger, Topic, Schedule, Target, Relation
from ams.nodes import FleetManager, SimBus, Vehicle
from ams.messages import VehicleStatus
from ams.structures import SIM_BUS_FLEET, SIM_BUS


class SimBusFleet(FleetManager):

    CONST = SIM_BUS_FLEET

    def __init__(self, _id, name, waypoint, arrow, route, spot):
        super().__init__(_id, name, waypoint, arrow, route)

        self.waypoint = waypoint
        self.arrow = arrow
        self.route = route
        self.spot = spot
        self.relation = Relation()
        self.__bus_routes = {}
        self.__bus_schedules = {}

        self.bus_parkable_spots = self.spot.get_spots_of_target_group(Target.new_node_target(SimBus))

        self.vehicle_statuses = self.manager.dict()
        self.vehicle_statuses_lock = self.manager.Lock()

        self.vehicle_schedules = {}
        self.vehicle_last_indices = {}
        self.bus_schedules = {}
        self.bus_stop_spots = {}
        self.state_machines = {}

        self.__topicPubVehicleSchedules = Topic()
        self.__topicPubVehicleSchedules.set_categories(FleetManager.CONST.TOPIC.CATEGORIES.SCHEDULES)

        self.__topicSubVehicleStatus = Topic()
        self.__topicSubVehicleStatus.set_targets(Target.new_target(None, SIM_BUS.NODE_NAME), None)
        self.__topicSubVehicleStatus.set_categories(Vehicle.CONST.TOPIC.CATEGORIES.STATUS)
        self.__topicSubVehicleStatus.set_message(VehicleStatus)
        self.set_subscriber(self.__topicSubVehicleStatus, self.update_vehicle_status)

    def __publish_vehicle_schedules(self, vehicle_id, payload):
        self.__topicPubVehicleSchedules.set_targets(self.target, Target.new_target(vehicle_id, SIM_BUS.NODE_NAME))
        self.publish(self.__topicPubVehicleSchedules, payload)

    def set_bus_schedules(self, bus_schedules):
        self.__bus_schedules = bus_schedules
        for bus_schedules_id in self.__bus_schedules:
            self.relation.add_relation(
                Target.new_target(bus_schedules_id, SIM_BUS_FLEET.TARGET_GROUP.BUS_SCHEDULES),
                Target.new_target(None, SIM_BUS.NODE_NAME)
            )

    def update_vehicle_status(self, _client, _userdata, topic, payload):
        self.vehicle_statuses_lock.acquire()
        vehicle_id = self.__topicSubVehicleStatus.get_from_id(topic)
        vehicle_status = self.__topicSubVehicleStatus.unserialize(payload)
        self.vehicle_statuses[vehicle_id] = vehicle_status
        self.vehicle_statuses_lock.release()

    def update_vehicle_schedules(self, vehicle_statuses):
        for vehicle_id, vehicle_status in vehicle_statuses.items():
            if vehicle_id not in self.vehicle_schedules:
                self.vehicle_schedules[vehicle_id] = [vehicle_status.schedule]
            else:
                while self.vehicle_schedules[vehicle_id][0].id != vehicle_status.schedule.id:
                    self.vehicle_schedules[vehicle_id].pop(0)

    def update_relation(self, target_bus_schedule, vehicle_id):
        self.relation.remove_relation(target_bus_schedule, Target.new_target(None, SIM_BUS.NODE_NAME))
        self.relation.add_relation(target_bus_schedule, Target.new_target(vehicle_id, SIM_BUS.NODE_NAME))

    def get_unassigned_bus_schedule_target(self):
        targets = Target.new_targets(self.relation.get_related(
            Target.new_target(None, SIM_BUS.NODE_NAME)
        ))
        if len(targets) == 0:
            return None
        return targets[0]

    def get_assigned_bus_schedule_target(self, target_vehicle):
        return self.relation.get_related(target_vehicle)[0]

    def get_to_circular_route(self, start_waypoint_id, start_arrow_code, bus_schedules_id):
        start_point = {
            "arrow_code": start_arrow_code,
            "waypoint_id": start_waypoint_id
        }
        goal_points = []
        for branch_index, branch in enumerate(self.__bus_schedules[bus_schedules_id]):
            for part_index, schedule in enumerate(branch.common):
                if schedule.event == SIM_BUS.EVENT.MOVE_TO_SELECT_POINT:
                    goal_point = {
                        "goal_id": ",".join(map(str, [branch_index,  "common", part_index])),
                        "waypoint_id": schedule.route.start_waypoint_id,
                        "arrow_code": schedule.route.arrow_codes[0],
                        # "waypoint_id": schedule.route.goal_waypoint_id,
                        # "arrow_code": schedule.route.arrow_codes[-1],
                    }
                    if goal_point not in goal_points:
                        goal_points.append(goal_point)
        shortest_routes = self.route.get_shortest_routes(start_point, goal_points, reverse=False)
        to_circular_route = min(shortest_routes.items(), key=lambda x: x[1]["cost"])[1]
        to_circular_route.pop("cost")
        branch_index, part_type, part_index = to_circular_route.pop("goal_id").split(",")
        return to_circular_route, int(branch_index), part_type, int(part_index)

    def get_move_to_circular_route_schedule(self, target_vehicle, target_bus_schedule, start_time):
        vehicle_status = self.vehicle_statuses[target_vehicle.id]
        to_circular_route, branch_index, part_type, part_index = \
            self.get_to_circular_route(
                vehicle_status.location.waypoint_id, vehicle_status.location.arrow_code, target_bus_schedule.id)

        return Schedule.new_schedule(
                [target_vehicle], SIM_BUS.EVENT.MOVE_TO_CIRCULAR_ROUTE, start_time, start_time + 1000,
                to_circular_route),\
            branch_index, part_type, part_index

    def get_move_to_branch_point_schedules(
            self, target_vehicle, target_bus_schedule, start_time,  branch_index, part_type, part_index):
        bus_schedule = self.__bus_schedules[target_bus_schedule.id]
        schedules = []
        if part_type in ["main", "sub"]:
            for i in range(part_index, len(bus_schedule[branch_index][part_type])):
                targets = [target_vehicle]
                if bus_schedule[branch_index][part_type][i].targets is not None:
                    targets.extend(bus_schedule[branch_index][part_type][i].targets)
                schedules.append(Schedule.new_schedule(
                    targets,
                    bus_schedule[branch_index][part_type][i].event, start_time, start_time + 1000,
                    bus_schedule[branch_index][part_type][i].route
                ))
                start_time += 1000
            branch_index = (branch_index + 1) * (branch_index + 1 < len(bus_schedule))
            part_type = "common"
            part_index = 0

        if part_type == "common":
            for i in range(part_index, len(bus_schedule[branch_index][part_type])):
                targets = [target_vehicle]
                if bus_schedule[branch_index][part_type][i].targets is not None:
                    targets.extend(bus_schedule[branch_index][part_type][i].targets)
                schedules.append(Schedule.new_schedule(
                    targets,
                    bus_schedule[branch_index][part_type][i].event, start_time, start_time + 1000,
                    bus_schedule[branch_index][part_type][i].route
                ))
                start_time += 1000
                if bus_schedule[branch_index][part_type][i].event == SIM_BUS.EVENT.MOVE_TO_BRANCH_POINT:
                    part_index = i
                    break
        return schedules, branch_index, part_type, part_index

    def get_move_to_branch_point_via_bus_stop_schedules(
            self, target_vehicle, target_bus_schedule, start_time,  branch_index):
        bus_schedule = self.__bus_schedules[target_bus_schedule.id]
        schedules = []
        for i in range(0, len(bus_schedule[branch_index].sub)):
            targets = [target_vehicle]
            if bus_schedule[branch_index].sub[i].targets is not None:
                targets.extend(bus_schedule[branch_index].sub[i].targets)
            schedules.append(Schedule.new_schedule(
                targets,
                bus_schedule[branch_index].sub[i].event, start_time, start_time + 5,
                bus_schedule[branch_index].sub[i].route
            ))
            start_time += 1000
        branch_index = (branch_index + 1) * (branch_index + 1 < len(bus_schedule))

        part_index = 0
        for i in range(len(bus_schedule[branch_index].common)):
            targets = [target_vehicle]
            if bus_schedule[branch_index].common[i].targets is not None:
                targets.extend(bus_schedule[branch_index].common[i].targets)
            schedules.append(Schedule.new_schedule(
                targets,
                bus_schedule[branch_index].common[i].event, start_time, start_time + 1000,
                bus_schedule[branch_index].common[i].route
            ))
            start_time += 1000
            if bus_schedule[branch_index].common[i].event == SIM_BUS.EVENT.MOVE_TO_BRANCH_POINT:
                part_index = i
                break
        return schedules, branch_index, "common", part_index

    def get_event_renamed_schedules(self, schedules):
        event_renamed_schedules = deepcopy(schedules)
        for i, schedule in enumerate(event_renamed_schedules):
            if schedule.event in [
                SIM_BUS.EVENT.MOVE_TO_CIRCULAR_ROUTE,
                SIM_BUS.EVENT.MOVE_TO_SELECT_POINT,
                SIM_BUS.EVENT.MOVE_TO_BRANCH_POINT,
                SIM_BUS.EVENT.MOVE_TO_BUS_STOP,
                SIM_BUS.EVENT.MOVE_TO_JUNCTION,
                SIM_BUS.EVENT.MOVE_TO_PARKING,
            ]:
                event_renamed_schedules[i].event = SIM_BUS.TRIGGER.MOVE
            elif schedule.event in [
                SIM_BUS.EVENT.STOP_TO_TAKE_UP,
                SIM_BUS.EVENT.STOP_TO_DISCHARGE,
                SIM_BUS.EVENT.STOP_TO_DISCHARGE_AND_TAKE_UP,
                SIM_BUS.EVENT.STOP_TO_PARKING
            ]:
                event_renamed_schedules[i].event = SIM_BUS.TRIGGER.STOP
            elif schedule.event in [
            ]:
                event_renamed_schedules[i].event = SIM_BUS.TRIGGER.REQUEST_SCHEDULES
        return event_renamed_schedules

    def get_state_machine(self, initial_state):
        machine = Machine(
            states=list(SIM_BUS_FLEET.STATE),
            initial=initial_state,
        )
        machine.add_transitions([
            {
                "trigger": SIM_BUS_FLEET.TRIGGER.ASSIGN_BUS_SCHEDULES,
                "source": SIM_BUS_FLEET.STATE.WAITING_FOR_BUS_STAND_BY,
                "dest": SIM_BUS_FLEET.STATE.WAITING_FOR_SCHEDULES_REQUEST,
                "conditions": [self.condition_schedules_length_and_publish_new_bus_schedules]
            },
            {
                "trigger": SIM_BUS_FLEET.TRIGGER.SEND_THROUGH_SCHEDULES,
                "source": SIM_BUS_FLEET.STATE.WAITING_FOR_SCHEDULES_REQUEST,
                "dest": SIM_BUS_FLEET.STATE.WAITING_FOR_SCHEDULES_REQUEST,
                "conditions": [self.condition_vehicle_state_and_publish_through_schedules]
            },
            {
                "trigger": SIM_BUS_FLEET.TRIGGER.SEND_VIA_SCHEDULES,
                "source": SIM_BUS_FLEET.STATE.WAITING_FOR_SCHEDULES_REQUEST,
                "dest": SIM_BUS_FLEET.STATE.WAITING_FOR_SCHEDULES_REQUEST,
                "conditions": [self.condition_vehicle_state_and_publish_via_schedules]
            },
        ])
        return machine

    def condition_schedules_length(self, vehicle_id, expected_length):
        return len(self.vehicle_schedules[vehicle_id]) == expected_length

    def condition_vehicle_state(self, vehicle_status, expected_state):
        return vehicle_status.state == expected_state

    def after_change_state_update_last_indices(self, vehicle_id, branch_index, part_type, part_index):
        self.vehicle_last_indices[vehicle_id] = {
            "branch_index": branch_index,
            "part_type": part_type,
            "part_index": part_index
        }
        return True

    def after_change_state_publish_schedules(self, vehicle_id):
        # logger.pp(self.vehicle_schedules[vehicle_id])
        event_renamed_schedules = self.get_event_renamed_schedules(self.vehicle_schedules[vehicle_id])
        payload = self.__topicPubVehicleSchedules.serialize(event_renamed_schedules)
        self.__publish_vehicle_schedules(vehicle_id, payload)
        return True

    def after_change_state_publish_new_bus_schedules(self, current_time, vehicle_id):
        target_vehicle = Target.new_target(vehicle_id, SIM_BUS.NODE_NAME)
        target_bus_schedule = self.get_unassigned_bus_schedule_target()

        vehicle_schedules = []
        start_time = current_time

        move_to_circular_route_schedule, branch_index, part_type, part_index = \
            self.get_move_to_circular_route_schedule(
                target_vehicle, target_bus_schedule, start_time)
        vehicle_schedules = \
            Schedule.get_merged_schedules(vehicle_schedules, [move_to_circular_route_schedule])
        start_time = vehicle_schedules[-1].period.end

        move_to_branch_point_schedules, branch_index, part_type, part_index = \
            self.get_move_to_branch_point_schedules(
                target_vehicle, target_bus_schedule, start_time, branch_index, part_type, part_index)
        vehicle_schedules = \
            Schedule.get_merged_schedules(vehicle_schedules, move_to_branch_point_schedules)

        self.vehicle_schedules[vehicle_id][0].period.end = current_time
        self.vehicle_schedules[vehicle_id] = \
            Schedule.get_merged_schedules(self.vehicle_schedules[vehicle_id], vehicle_schedules)

        self.update_relation(target_bus_schedule, vehicle_id)

        self.after_change_state_update_last_indices(vehicle_id, branch_index, part_type, part_index)
        self.after_change_state_publish_schedules(vehicle_id)
        return True

    def after_change_state_publish_through_schedules(self, vehicle_id):
        target_vehicle = Target.new_target(vehicle_id, SIM_BUS.NODE_NAME)
        target_bus_schedule = self.get_assigned_bus_schedule_target(target_vehicle)
        move_to_branch_point_schedules, branch_index, part_type, part_index = \
            self.get_move_to_branch_point_schedules(
                target_vehicle, target_bus_schedule, self.vehicle_schedules[vehicle_id][-1].period.end,
                self.vehicle_last_indices[vehicle_id]["branch_index"], "main", 0)

        self.vehicle_schedules[vehicle_id] = Schedule.get_merged_schedules(
            # [self.vehicle_schedules[vehicle_id][-1]], move_to_branch_point_schedules)
            self.vehicle_schedules[vehicle_id], move_to_branch_point_schedules)

        self.after_change_state_update_last_indices(vehicle_id, branch_index, part_type, part_index)
        self.after_change_state_publish_schedules(vehicle_id)
        return True

    def after_change_state_publish_via_schedules(self, vehicle_id):
        target_vehicle = Target.new_target(vehicle_id, SIM_BUS.NODE_NAME)
        target_bus_schedule = self.get_assigned_bus_schedule_target(target_vehicle)
        move_to_branch_point_via_bus_stop_schedules, branch_index, part_type, part_index = \
            self.get_move_to_branch_point_via_bus_stop_schedules(
                target_vehicle, target_bus_schedule, self.vehicle_schedules[vehicle_id][-1].period.end,
                self.vehicle_last_indices[vehicle_id]["branch_index"])

        self.vehicle_schedules[vehicle_id] = Schedule.get_merged_schedules(
            # [self.vehicle_schedules[vehicle_id][-1]], move_to_branch_point_via_bus_stop_schedules)
            self.vehicle_schedules[vehicle_id], move_to_branch_point_via_bus_stop_schedules)

        self.after_change_state_update_last_indices(vehicle_id, branch_index, part_type, part_index)
        self.after_change_state_publish_schedules(vehicle_id)
        return True

    def condition_schedules_length_and_publish_new_bus_schedules(self, current_time, vehicle_id):
        if self.condition_schedules_length(vehicle_id, 1):
            self.after_change_state_publish_new_bus_schedules(current_time, vehicle_id)
            return True
        return False

    def condition_vehicle_state_and_publish_through_schedules(self, vehicle_id, vehicle_status, expected_state):
        if self.condition_vehicle_state(vehicle_status, expected_state):
            if self.condition_schedules_length(vehicle_id, 1):
                self.after_change_state_publish_through_schedules(vehicle_id)
                return True
        return False

    def condition_vehicle_state_and_publish_via_schedules(self, vehicle_id, vehicle_status, expected_state):
        if self.condition_vehicle_state(vehicle_status, expected_state):
            if self.condition_schedules_length(vehicle_id, 1):
                self.after_change_state_publish_via_schedules(vehicle_id)
                return True
        return False

    def get_vehicle_statuses_and_lock(self):
        self.vehicle_statuses_lock.acquire()
        return deepcopy(self.vehicle_statuses)

    def set_vehicle_statuses_and_unlock(self, vehicle_statuses):
        self.vehicle_statuses.clear()
        self.vehicle_statuses.update(vehicle_statuses)
        self.vehicle_statuses_lock.release()

    def update_status(self):
        vehicle_statuses = self.get_vehicle_statuses_and_lock()

        self.update_vehicle_schedules(vehicle_statuses)

        self.update_state_machines(vehicle_statuses)

        self.set_vehicle_statuses_and_unlock(vehicle_statuses)

    def update_state_machines(self, vehicle_statuses):
        current_time = time()

        for vehicle_id, vehicle_status in vehicle_statuses.items():
            if vehicle_id not in self.state_machines:
                self.state_machines[vehicle_id] = self.get_state_machine(SIM_BUS_FLEET.STATE.WAITING_FOR_BUS_STAND_BY)

            vehicle_state = self.state_machines[vehicle_id].state

            if vehicle_state == SIM_BUS_FLEET.STATE.WAITING_FOR_BUS_STAND_BY:
                self.state_machines[vehicle_id].send_circular_route_schedules(current_time, vehicle_id)
            elif vehicle_state == SIM_BUS_FLEET.STATE.WAITING_FOR_SCHEDULES_REQUEST:
                if not self.state_machines[vehicle_id].send_through_schedules(
                        vehicle_id, vehicle_status, SIM_BUS.STATE.REQUEST_THROUGH_SCHEDULES):
                    self.state_machines[vehicle_id].send_via_schedules(
                        vehicle_id, vehicle_status, SIM_BUS.STATE.REQUEST_VIA_SCHEDULES)

        # logger.pp({"fleet": {
        #     "fleet": dict(map(lambda x: (x[0], x[1].state), self.state_machines.items())),
        #     "vehicle": dict(map(lambda x: (x[0], x[1].state), vehicle_statuses.items())),
        # }})
