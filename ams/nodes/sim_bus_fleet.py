#!/usr/bin/env python
# coding: utf-8

from time import time

from ams import Topic, Route, Schedule, Target, Relation
from ams.nodes import FleetManager, SimBus, Vehicle
from ams.messages import VehicleStatus
from ams.structures import SIM_BUS_FLEET, SIM_BUS
from pprint import PrettyPrinter
pp = PrettyPrinter(indent=2).pprint


class SimBusFleet(FleetManager):

    CONST = SIM_BUS_FLEET

    def __init__(self, name, waypoint, arrow, route, spot):
        super().__init__(name, waypoint, arrow, route)

        self.waypoint = waypoint
        self.arrow = arrow
        self.route = route
        self.spot = spot
        self.relation = Relation()
        self.__bus_routes = {}
        self.__bus_schedules = {}

        self.bus_parkable_spots = self.spot.get_spots_of_target_group(Target.new_node_target(SimBus))

        self.vehicle_statuses = {}
        self.vehicle_schedules = {}
        self.vehicle_last_indices = {}
        self.bus_schedules = {}
        self.bus_stop_spots = {}

        self.__topicPubVehicleSchedules = Topic()
        self.__topicPubVehicleSchedules.set_categories(FleetManager.CONST.TOPIC.CATEGORIES.SCHEDULES)

        self.__topicSubVehicleStatus = Topic()
        self.__topicSubVehicleStatus.set_targets(Target.new_target(None, SimBus.__name__), None)
        self.__topicSubVehicleStatus.set_categories(Vehicle.CONST.TOPIC.CATEGORIES.STATUS)
        self.__topicSubVehicleStatus.set_message(VehicleStatus)
        self.set_subscriber(self.__topicSubVehicleStatus, self.update_vehicle_status)

    def __publish_vehicle_schedules(self, vehicle_id, payload):
        self.__topicPubVehicleSchedules.set_targets(self.target, Target.new_target(vehicle_id, SimBus.__name__))
        self.publish(self.__topicPubVehicleSchedules, payload)

    def set_bus_schedules(self, bus_schedules):
        self.__bus_schedules = bus_schedules
        for bus_schedules_id in self.__bus_schedules:
            self.relation.add_relation(
                Target.new_target(bus_schedules_id, SIM_BUS_FLEET.TARGET_GROUP.BUS_SCHEDULES),
                {
                    SIM_BUS_FLEET.UNASSIGNED_BUS_SCHEDULES_KEY: SIM_BUS_FLEET.TARGET_GROUP.BUS_SCHEDULES
                }
            )

    def update_vehicle_status(self, _client, _userdata, topic, payload):
        vehicle_id = self.__topicSubVehicleStatus.get_from_id(topic)
        vehicle_status = self.__topicSubVehicleStatus.unserialize(payload)
        if vehicle_id not in self.vehicle_schedules:
            self.vehicle_schedules[vehicle_id] = [vehicle_status.schedule]
        else:
            if self.vehicle_statuses[vehicle_id].state != vehicle_status.state:
                self.update_current_vehicle_schedule(vehicle_id, vehicle_status)
        self.vehicle_statuses[vehicle_id] = vehicle_status

    def update_current_vehicle_schedule(self, vehicle_id, vehicle_status):
        self.vehicle_schedules[vehicle_id].pop(0)
        self.vehicle_schedules[vehicle_id][0] = vehicle_status.schedule

    def get_unassigned_bus_schedule_target(self):
        targets = Target.new_targets(self.relation.get_related({
            SIM_BUS_FLEET.UNASSIGNED_BUS_SCHEDULES_KEY: SIM_BUS_FLEET.TARGET_GROUP.BUS_SCHEDULES}))
        if len(targets) == 0:
            return None
        return targets[0]

    def get_to_circular_route(self, start_waypoint_id, start_arrow_code, bus_schedules_id):
        start_point = {
            "arrow_code": start_arrow_code,
            "waypoint_id": start_waypoint_id
        }
        goal_points = []
        for branch_index, branch in enumerate(self.__bus_schedules[bus_schedules_id]):
            for part_index, schedule in enumerate(branch.common):
                if schedule.event == SIM_BUS.SCHEDULE.MOVE_TO_SELECT_POINT:
                    goal_point = {
                        "goal_id": ",".join(map(str, [branch_index,  "common", part_index])),
                        "waypoint_id": schedule.route.goal_waypoint_id,
                        "arrow_code": schedule.route.arrow_codes[-1],
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
                [target_vehicle], SIM_BUS.SCHEDULE.MOVE_TO_CIRCULAR_ROUTE, start_time, start_time + 1000,
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
                if bus_schedule[branch_index][part_type][i].event == SIM_BUS.SCHEDULE.MOVE_TO_BRANCH_POINT:
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
            if bus_schedule[branch_index].common[i].event == SIM_BUS.SCHEDULE.MOVE_TO_BRANCH_POINT:
                part_index = i
                break
        return schedules, branch_index, "common", part_index

    def update_status(self):
        for vehicle_id, vehicle_status in self.vehicle_statuses.items():
            if vehicle_status.schedule.event == SIM_BUS.SCHEDULE.STAND_BY:
                if self.vehicle_schedules[vehicle_id][0].event != SIM_BUS.SCHEDULE.MOVE_TO_CIRCULAR_ROUTE:
                    start_time = time()
                    target_vehicle = Target.new_target(vehicle_id, SimBus.__name__)
                    target_bus_schedule = self.get_unassigned_bus_schedule_target()

                    target_vehicle_schedules = [Schedule.new_schedule(
                            [target_vehicle], SIM_BUS.SCHEDULE.STAND_BY, start_time, start_time + 1,
                            Route.new_point_route(vehicle_status.location.waypoint_id,
                                                  vehicle_status.location.arrow_code))]
                    start_time = target_vehicle_schedules[-1].period.end

                    move_to_circular_route_schedule, branch_index, part_type, part_index = \
                        self.get_move_to_circular_route_schedule(
                            target_vehicle, target_bus_schedule, start_time)
                    target_vehicle_schedules = \
                        Schedule.get_merged_schedules(target_vehicle_schedules, [move_to_circular_route_schedule])
                    start_time = target_vehicle_schedules[-1].period.end

                    move_to_branch_point_schedules, branch_index, part_type, part_index = \
                        self.get_move_to_branch_point_schedules(
                            target_vehicle, target_bus_schedule, start_time, branch_index, part_type, part_index)
                    target_vehicle_schedules = \
                        Schedule.get_merged_schedules(target_vehicle_schedules, move_to_branch_point_schedules)

                    self.vehicle_last_indices[vehicle_id] = {
                        "branch_index": branch_index,
                        "part_type": part_type,
                        "part_index": part_index
                    }
                    self.vehicle_schedules[vehicle_id] = target_vehicle_schedules
                    payload = self.__topicPubVehicleSchedules.serialize(self.vehicle_schedules[vehicle_id])
                    self.__publish_vehicle_schedules(vehicle_id, payload)

            elif vehicle_status.schedule.event == SIM_BUS.SCHEDULE.MOVE_TO_CIRCULAR_ROUTE:
                pass
            elif vehicle_status.schedule.event == SIM_BUS.SCHEDULE.MOVE_TO_BRANCH_POINT:
                target_vehicle = Target.new_target(vehicle_id, SimBus.__name__)
                target_bus_schedule = self.get_unassigned_bus_schedule_target()
                if vehicle_status.state == SIM_BUS.STATE.THROUGH:
                    move_to_branch_point_schedules, branch_index, part_type, part_index = \
                        self.get_move_to_branch_point_schedules(
                            target_vehicle, target_bus_schedule, self.vehicle_schedules[vehicle_id][-1].period.end,
                            self.vehicle_last_indices[vehicle_id]["branch_index"], "main", 0)
                    self.vehicle_last_indices[vehicle_id] = {
                        "branch_index": branch_index,
                        "part_type": part_type,
                        "part_index": part_index
                    }
                    # pp(move_to_branch_point_schedules)
                    self.vehicle_schedules[vehicle_id] = Schedule.get_merged_schedules(
                        [self.vehicle_schedules[vehicle_id][-1]], move_to_branch_point_schedules)
                    payload = self.__topicPubVehicleSchedules.serialize(self.vehicle_schedules[vehicle_id])
                    self.__publish_vehicle_schedules(vehicle_id, payload)
                elif vehicle_status.state == SIM_BUS.STATE.VIA:
                    move_to_branch_point_via_bus_stop_schedules, branch_index, part_type, part_index = \
                        self.get_move_to_branch_point_via_bus_stop_schedules(
                            target_vehicle, target_bus_schedule, self.vehicle_schedules[vehicle_id][-1].period.end,
                            self.vehicle_last_indices[vehicle_id]["branch_index"])
                    self.vehicle_last_indices[vehicle_id] = {
                        "branch_index": branch_index,
                        "part_type": part_type,
                        "part_index": part_index
                    }
                    # pp(move_to_branch_point_via_bus_stop_schedules)
                    self.vehicle_schedules[vehicle_id] = Schedule.get_merged_schedules(
                        [self.vehicle_schedules[vehicle_id][-1]], move_to_branch_point_via_bus_stop_schedules)
                    payload = self.__topicPubVehicleSchedules.serialize(self.vehicle_schedules[vehicle_id])
                    self.__publish_vehicle_schedules(vehicle_id, payload)
                    pass
                # print("BusFleet:", vehicle_status.state, vehicle_status.schedule.event)
        return
