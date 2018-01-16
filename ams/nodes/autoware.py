#!/usr/bin/env python
# coding: utf-8

from sys import float_info
from transforms3d.quaternions import axangle2quat

from ams import Topic
from ams.nodes import Vehicle, TrafficSignal
from ams.messages import autoware_message, traffic_signal_message


class Autoware(Vehicle):

    FLOAT_MAX = float_info.max

    class TRAFFIC_LIGHT(object):
        RED = 0
        GREEN = 1
        UNKNOWN = 2

    class ROSTOPIC(object):
        WAYPOINTS = "/based/lane_waypoints_array"
        TRAFFIC_LIGHT = "/light_color_managed"
        CLOSEST_WAYPOINT = "/closest_waypoint"

    class TOPIC(object):
        PUBLISH = "pub_autoware"
        SUBSCRIBE = "sub_autoware"

    def __init__(self, name, waypoint, arrow, route, waypoint_id, velocity, schedules=None, dt=1.0):
        super().__init__(name, waypoint, arrow, route, waypoint_id, velocity, schedules, dt)

        self.name = name

        self.autowarePublishTopic = Topic()
        self.autowarePublishTopic.set_id(self.name)
        self.autowarePublishTopic.set_root(Autoware.TOPIC.PUBLISH)
        self.autowarePublishTopic.set_message(autoware_message)

        self.autowareSubscribeTopic = Topic()
        self.autowareSubscribeTopic.set_id(self.name)
        self.autowareSubscribeTopic.set_root(Autoware.TOPIC.SUBSCRIBE)
        self.autowareSubscribeTopic.set_message(autoware_message)

        self.topicTrafficSignalPublish = Topic()
        self.topicTrafficSignalPublish.set_root(TrafficSignal.TOPIC.PUBLISH)
        self.topicTrafficSignalPublish.set_message(traffic_signal_message)

        self.pose_index = 0
        self.current_poses = []
        self.traffic_signals = {}

        self.add_on_message_function(self.set_autoware_pose)
        self.add_on_message_function(self.set_traffic_signals)
        self.set_subscriber(self.autowareSubscribeTopic.private+"/closest_waypoint")
        self.set_subscriber(self.topicTrafficSignalPublish.all)

    def set_autoware_pose(self, _client, _userdata, topic, payload):
        if topic == self.autowareSubscribeTopic.private+"/closest_waypoint":
            message = self.autowareSubscribeTopic.unserialize(payload)
            if 0 <= message["index"] < len(self.current_poses):
                self.pose_index = message["index"]
                print(self.current_poses[self.pose_index])
                self.arrow_code = self.current_poses[self.pose_index]["arrow_code"]
                self.waypoint_id = self.current_poses[self.pose_index]["waypoint_id"]
                self.position = self.waypoint.get_position(self.waypoint_id)
                self.yaw = self.arrow.get_heading(self.arrow_code, self.waypoint_id)

                self.set_autoware_traffic_light()
            else:
                print("Lost Autoware.")

    def set_autoware_waypoints(self):
        waypoints = []
        schedule = self.schedules[0]

        arrow_waypoint_array = self.route.get_arrow_waypoint_array({
            "start_waypoint_id": schedule["route"]["start"]["waypoint_id"],
            "goal_waypoint_id": schedule["route"]["goal"]["waypoint_id"],
            "arrow_codes": schedule["route"]["arrow_codes"]
        })
        for arrowWaypoint in arrow_waypoint_array:
            waypoint_id = arrowWaypoint["waypoint_id"]
            waypoints.append({
                "position": dict(zip(["x", "y", "z"], self.waypoint.get_position(waypoint_id))),
                "orientation": dict(zip(
                    ["w", "x", "y", "z"], axangle2quat([0, 0, 1], self.waypoint.get_yaw(waypoint_id)))),
                "velocity": 2.0
            })
        if 0 < len(waypoints):
            num = min(10, len(waypoints))
            for i in range(num-1, 0, -1):
                waypoints[-i]["velocity"] = (i/num)*waypoints[-i-1]["velocity"]
            self.current_poses = arrow_waypoint_array
            payload = self.autowarePublishTopic.serialize(waypoints)
            self.publish(self.autowarePublishTopic.private+"/waypoints", payload)

    def set_traffic_signals(self, _client, _user_data, topic, payload):
        if self.topicTrafficSignalPublish.root in topic:
            message = self.topicTrafficSignalPublish.unserialize(payload)
            self.traffic_signals.update(dict(map(lambda x: (x["route_code"], x), message["routes"])))

    def __get_inter_traffic_signal_distance(self, monitored_route):
        monitored_arrow_codes = monitored_route["arrow_codes"]
        inter_traffic_signal_distance = Autoware.FLOAT_MAX

        not_green_traffic_signal_route_codes = list(map(
            lambda x: x["route_code"], filter(
                lambda x: x["state"] in [TrafficSignal.STATE.YELLOW, TrafficSignal.STATE.RED],
                self.traffic_signals.values())))

        new_monitored_route = {
            "start_waypoint_id": monitored_route["start_waypoint_id"],
            "arrow_codes": None,
            "goal_waypoint_id": None
        }
        for i, monitored_arrow_code in enumerate(monitored_arrow_codes):
            for not_green_traffic_signal_route_code in not_green_traffic_signal_route_codes:
                if monitored_arrow_code in not_green_traffic_signal_route_code:
                    start_waypoint_id, arrow_codes, _ = self.route.split_route_code(not_green_traffic_signal_route_code)
                    if monitored_arrow_code == arrow_codes[0]:
                        waypoint_ids = self.arrow.get_waypoint_ids(monitored_arrow_code)
                        if self.waypoint_id not in waypoint_ids or \
                                waypoint_ids.index(self.waypoint_id) <= waypoint_ids.index(start_waypoint_id):
                            new_monitored_route["goal_waypoint_id"] = start_waypoint_id
                            new_monitored_route["arrow_codes"] = monitored_arrow_codes[:i]
                            break
            if new_monitored_route["arrow_codes"] is not None:
                break

        if new_monitored_route["arrow_codes"] is not None:
            inter_traffic_signal_distance = self.route.get_route_length(new_monitored_route)

        # print("inter_traffic_signal_distance {}[m]".format(inter_traffic_signal_distance))
        return inter_traffic_signal_distance

    def get_monitored_route(self, distance=100.0):
        if distance <= 0:
            return None

        arrow_codes = self.schedules[0]["route"]["arrow_codes"]
        if arrow_codes is None:
            return None

        i_s = 0
        if self.arrow_code in arrow_codes:
            i_s = arrow_codes.index(self.arrow_code)
        arrow_codes = arrow_codes[i_s:]
        route = {
            "start_waypoint_id": self.waypoint_id,
            "goal_waypoint_id": self.arrow.get_waypoint_ids(self.schedules[0]["route"]["arrow_codes"][-1])[-1],
            "arrow_codes": arrow_codes
        }
        return self.route.get_sliced_route(route, distance)

    def set_autoware_traffic_light(self):
        if self.schedules[0]["action"] == Vehicle.ACTION.MOVE:
            monitored_route = self.get_monitored_route()
            if monitored_route is None:
                payload = self.autowarePublishTopic.serialize({"traffic_light": Autoware.TRAFFIC_LIGHT.RED})
            else:
                inter_traffic_signal_distance = self.__get_inter_traffic_signal_distance(monitored_route)
                print("inter_traffic_signal_distance", inter_traffic_signal_distance)
                if inter_traffic_signal_distance <= 20.0:
                    payload = self.autowarePublishTopic.serialize({"traffic_light": Autoware.TRAFFIC_LIGHT.RED})
                else:
                    payload = self.autowarePublishTopic.serialize({"traffic_light": Autoware.TRAFFIC_LIGHT.GREEN})
            self.publish(self.autowarePublishTopic.private+"/traffic_light", payload)
