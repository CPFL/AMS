#!/usr/bin/env python
# coding: utf-8

from time import time
from copy import deepcopy
from ams import Topic
from ams.nodes import EventLoop, User, Vehicle, TrafficSignal, SimTaxi
from ams.messages import user_message, vehicle_message, traffic_signal_message, fleet_manager_message


class FleetManager(EventLoop):
    class ACTION(object):
        PUBLISH_RELATIONS = "pub_relations"

    class TOPIC(object):
        PUBLISH = "pub_fleet_manager"
        SUBSCRIBE = "sub_fleet_manager"

    def __init__(self, waypoint, arrow, route):
        super().__init__()

        self.topicUserPublish = Topic()
        self.topicUserPublish.set_root(User.TOPIC.PUBLISH)
        self.topicUserPublish.set_message(user_message)

        self.topicUserSubscribe = Topic()
        self.topicUserSubscribe.set_root(User.TOPIC.SUBSCRIBE)
        self.topicUserSubscribe.set_message(user_message)

        self.topicVehiclePublish = Topic()
        self.topicVehiclePublish.set_root(Vehicle.TOPIC.PUBLISH)
        self.topicVehiclePublish.set_message(vehicle_message)

        self.topicVehicleSubscribe = Topic()
        self.topicVehicleSubscribe.set_root(Vehicle.TOPIC.SUBSCRIBE)
        self.topicVehicleSubscribe.set_message(vehicle_message)

        self.topicTrafficSignalPublish = Topic()
        self.topicTrafficSignalPublish.set_root(TrafficSignal.TOPIC.PUBLISH)
        self.topicTrafficSignalPublish.set_message(traffic_signal_message)

        self.topicFleetManagerPublish = Topic()
        self.topicFleetManagerPublish.set_id(self.event_loop_id)
        self.topicFleetManagerPublish.set_root(FleetManager.TOPIC.PUBLISH)
        self.topicFleetManagerPublish.set_message(fleet_manager_message)

        self.topicFleetManagerSubscribe = Topic()
        self.topicFleetManagerSubscribe.set_id(self.event_loop_id)
        self.topicFleetManagerSubscribe.set_root(FleetManager.TOPIC.SUBSCRIBE)
        self.topicFleetManagerSubscribe.set_message(fleet_manager_message)

        self.waypoint = waypoint
        self.arrow = arrow
        self.route = route

        self.users = {}
        self.vehicles = {}  # vehicle_id: {lat: [float], lng: [float], schedule: }
        self.traffic_signals = {}
        self.relations = {}  # vehicle_id -> user_id, user_id -> vehicle_id

        self.add_on_message_function(self.update_user_status)
        self.add_on_message_function(self.update_vehicle_status)
        self.add_on_message_function(self.update_traffic_signal_status)
        self.add_on_message_function(self.response_request)

        self.set_subscriber(self.topicUserPublish.all)
        self.set_subscriber(self.topicVehiclePublish.all)
        self.set_subscriber(self.topicTrafficSignalPublish.all)
        self.set_subscriber(self.topicFleetManagerSubscribe.all)

    def update_user_status(self, _client, _userdata, topic, payload):
        # print("update_user_status", topic)
        if self.topicUserPublish.root in topic:
            user_id = self.topicUserPublish.get_id(topic)
            message = self.topicUserPublish.unserialize(payload)

            if message["state"] == User.STATE.LOGIN:
                # print("user", User.STATE.LOGIN)
                # todo: move to dispatcher class
                self.dispatch(user_id, message)
            elif message["state"] == User.STATE.WAITING:
                pass
                # print("user", User.STATE.WAITING)
            elif message["state"] == User.STATE.GETTING_ON:
                pass
                # print("user", User.STATE.GETTING_ON)
            elif message["state"] == User.STATE.GOT_ON:
                # print("user", User.STATE.GOT_ON)
                vehicle_id = self.relations[user_id]
                self.vehicles[vehicle_id]["schedules"][0]["action"] = Vehicle.ACTION.MOVE
                payload = self.topicVehicleSubscribe.serialize({"schedules": self.vehicles[vehicle_id]["schedules"]})
                self.publish(self.topicVehicleSubscribe.root + "/" + vehicle_id + "/schedules", payload)
            elif message["state"] == User.STATE.MOVING:
                pass
                # print("user", User.STATE.MOVING)
            elif message["state"] == User.STATE.GETTING_OUT:
                pass
                # print("user", User.STATE.GETTING_OUT)
            elif message["state"] == User.STATE.GOT_OUT:
                pass
                # print("user", User.STATE.GOT_OUT)
            else:
                print("user", message["state"])

            self.users[user_id] = message

    def update_vehicle_status(self, _client, _userdata, topic, payload):
        if self.topicVehiclePublish.root in topic:
            vehicle_id = self.topicVehiclePublish.get_id(topic)
            message = self.topicVehiclePublish.unserialize(payload)
            # print("update_vehicle_status", topic, message["state"])

            if vehicle_id in self.relations:
                prev_state = self.vehicles[vehicle_id]["state"]
                if message["state"] == SimTaxi.STATE.MOVE_TO_USER:
                    # print("vehicle", SimTaxi.STATE.MOVE_TO_USER)
                    if prev_state == SimTaxi.STATE.STANDBY:
                        user_id = self.relations[vehicle_id]
                        self.users[user_id]["schedules"][0]["action"] = User.ACTION.WAIT
                        self.publish(
                            self.topicUserSubscribe.root+"/"+user_id+"/schedules",
                            self.topicUserSubscribe.serialize({"schedules": self.users[user_id]["schedules"]}))
                elif message["state"] == SimTaxi.STATE.STOP_FOR_PICKING_UP:
                    # print("vehicle", SimTaxi.STATE.STOP_FOR_PICKING_UP)
                    if prev_state == SimTaxi.STATE.MOVE_TO_USER:
                        # print("vehicle", SimTaxi.STATE.STOP_FOR_PICKING_UP, SimTaxi.STATE.MOVE_TO_USER)
                        user_id = self.relations[vehicle_id]
                        self.users[user_id]["schedules"][0]["action"] = User.ACTION.GET_ON
                        self.publish(
                            self.topicUserSubscribe.root+"/"+user_id+"/schedules",
                            self.topicUserSubscribe.serialize({"schedules": self.users[user_id]["schedules"]}))
                elif message["state"] == SimTaxi.STATE.MOVE_TO_USER_DESTINATION:
                    # print("vehicle", SimTaxi.STATE.MOVE_TO_USER_DESTINATION)
                    if prev_state == SimTaxi.STATE.STOP_FOR_PICKING_UP:
                        # print("vehicle", SimTaxi.STATE.MOVE_TO_USER_DESTINATION, SimTaxi.STATE.STOP_FOR_PICKING_UP)
                        user_id = self.relations[vehicle_id]
                        self.users[user_id]["schedules"][0]["event"] = User.EVENT.MOVE_VEHICLE
                        self.publish(
                            self.topicUserSubscribe.root+"/"+user_id+"/event",
                            self.topicUserSubscribe.serialize({"event": User.EVENT.MOVE_VEHICLE}))
                elif message["state"] == SimTaxi.STATE.STOP_FOR_DISCHARGING:
                    # print("vehicle", SimTaxi.STATE.STOP_FOR_DISCHARGING)
                    if prev_state == SimTaxi.STATE.MOVE_TO_USER_DESTINATION:
                        user_id = self.relations[vehicle_id]
                        self.users[user_id]["schedules"][0]["action"] = User.ACTION.GET_OUT
                        self.publish(
                            self.topicUserSubscribe.root+"/"+user_id+"/schedules",
                            self.topicUserSubscribe.serialize({"schedules": self.users[user_id]["schedules"]}))
                elif message["state"] == SimTaxi.STATE.MOVE_TO_STANDBY:
                    pass
                    # print("vehicle", SimTaxi.STATE.MOVE_TO_STANDBY)
                elif message["state"] == SimTaxi.STATE.STANDBY:
                    pass
                    # print("vehicle", SimTaxi.STATE.STANDBY)
                else:
                    print("vehicle", message["state"])

            # vehicleSchedule = vehicle.pop("schedule")
            if vehicle_id not in self.vehicles:
                # print("set vehicle", vehicle_id, message["name"])
                self.vehicles[vehicle_id] = message
            else:
                # print("update vehicle", vehicle_id, message["name"])
                self.vehicles[vehicle_id].update(message)

    def update_traffic_signal_status(self, _client, _userdata, topic, payload):
        if self.topicTrafficSignalPublish.root in topic:
            message = self.topicTrafficSignalPublish.unserialize(payload)
            for route in message["routes"]:
                self.traffic_signals[route["route_code"]] = route

    def response_request(self, _client, _userdata, topic, payload):
        if self.topicFleetManagerSubscribe.root in topic:
            message = self.topicFleetManagerSubscribe.unserialize(payload)
            if message["action"] == FleetManager.ACTION.PUBLISH_RELATIONS:
                self.publish_relations()


    def get_dispatchable_vehicles(self):
        return dict(filter(
            lambda x: x[1]["state"] in [SimTaxi.STATE.STANDBY],
            self.vehicles.items()
        ))

    def dispatch(self, user_id, user_status):
        start_point = {
            "arrow_code": self.arrow.get_arrow_codes_from_waypoint_id(
                user_status["schedules"][0]["start"]["waypoint_id"])[0],
            "waypoint_id": user_status["schedules"][0]["start"]["waypoint_id"],
        }
        vehicles = self.get_dispatchable_vehicles()
        if len(vehicles) == 0:
            print("no dispatchable vehicles")
            return

        goal_points = []
        for vehicle_id, goal_waypoint_id in map(
                lambda x: (x[0], x[1]["pose"]["position"]["waypoint_id"]), vehicles.items()):
            goal_points.append({
                "goal_id": vehicle_id,
                "arrow_code": self.arrow.get_arrow_codes_from_waypoint_id(goal_waypoint_id)[0],
                "waypoint_id": goal_waypoint_id,
            })
        routes = self.route.get_shortest_routes(start_point, goal_points, reverse=True)
        if len(routes) == 0:
            print("no pick_up_route")
            return
        pick_up_route = min(routes.items(), key=lambda x: x[1]["cost"])[1]

        vehicle_id = pick_up_route["goal_id"]

        start_point = {
            "arrow_code": self.arrow.get_arrow_codes_from_waypoint_id(
                user_status["schedules"][0]["start"]["waypoint_id"])[0],
            "waypoint_id": user_status["schedules"][0]["start"]["waypoint_id"],
        }
        goal_points = [{
            "goal_id": user_id,
            "arrow_code": self.arrow.get_arrow_codes_from_waypoint_id(
                user_status["schedules"][0]["goal"]["waypoint_id"])[0],
            "waypoint_id": user_status["schedules"][0]["goal"]["waypoint_id"],
        }]
        routes = self.route.get_shortest_routes(start_point, goal_points, reverse=False)
        if len(routes) == 0:
            print("cant carry_route")
            return
        carry_route = min(routes.items(), key=lambda x: x[1]["cost"])[1]

        current_time = time()
        vehicle_schedule = deepcopy(self.topicVehicleSubscribe.get_template()["schedules"][0])
        vehicle_schedule.update({
            "name": "pickup",
            "start_time": current_time,
            "duration": 1000,
            "action": Vehicle.ACTION.MOVE,
            "route": {
                "start": {
                    "waypoint_id": pick_up_route["goal_waypoint_id"],
                },
                "goal": {
                    "waypoint_id": pick_up_route["start_waypoint_id"],
                },
                "arrow_codes": pick_up_route["arrow_codes"],
            }
        })
        self.vehicles[vehicle_id]["schedules"].append(vehicle_schedule)

        vehicle_schedule = deepcopy(self.topicVehicleSubscribe.get_template()["schedules"][0])
        vehicle_schedule.update({
            "name": "takeOn",
            "start_time": current_time+1000,
            "duration": 10,
            "action": Vehicle.ACTION.STOP,
        })
        self.vehicles[vehicle_id]["schedules"].append(vehicle_schedule)

        vehicle_schedule = deepcopy(self.topicVehicleSubscribe.get_template()["schedules"][0])
        vehicle_schedule.update({
            "name": "carry",
            "start_time": current_time+1010,
            "duration": 1000,
            "action": Vehicle.ACTION.MOVE,
            "route": {
                "start": {
                    "waypoint_id": carry_route["start_waypoint_id"],
                },
                "goal": {
                    "waypoint_id": carry_route["goal_waypoint_id"],
                },
                "arrow_codes": carry_route["arrow_codes"],
            }
        })
        self.vehicles[vehicle_id]["schedules"].append(vehicle_schedule)

        vehicle_schedule = deepcopy(self.topicVehicleSubscribe.get_template()["schedules"][0])
        vehicle_schedule.update({
            "name": "discharge",
            "start_time": current_time+2010,
            "duration": 10,
            "action": Vehicle.ACTION.STOP,
        })
        self.vehicles[vehicle_id]["schedules"].append(vehicle_schedule)

        vehicle_schedule = deepcopy(self.topicVehicleSubscribe.get_template()["schedules"][0])
        vehicle_schedule.update({
            "name": "standBy",
            "start_time": current_time+2020,
            "duration": 86400,
            "action": SimTaxi.ACTION.STANDBY,
        })
        self.vehicles[vehicle_id]["schedules"].append(vehicle_schedule)

        payload = self.topicVehicleSubscribe.serialize({"schedules": self.vehicles[vehicle_id]["schedules"]})
        self.publish(self.topicVehicleSubscribe.root+"/"+vehicle_id+"/schedules", payload)

        self.relations[user_id] = vehicle_id
        self.relations[vehicle_id] = user_id

        self.publish_relations()

    def publish_relations(self):
        message = self.topicFleetManagerPublish.get_template()
        message["time"] = time();
        message["relations"] = self.relations
        payload = self.topicFleetManagerPublish.serialize(message)
        self.publish(self.topicFleetManagerPublish.private, payload)
