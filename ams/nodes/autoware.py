#!/usr/bin/env python
# coding: utf-8

from transforms3d.quaternions import axangle2quat

from ams import Topic
from ams.nodes import Vehicle
from ams.messages import autoware_message


class Autoware(Vehicle):

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

        self.pose_index = 0
        self.current_poses = []

        self.add_on_message_function(self.set_autoware_pose)
        self.set_subscriber(self.autowareSubscribeTopic.private+"/closest_waypoint")

    def set_autoware_pose(self, _client, _userdata, topic, payload):
        if topic == self.autowareSubscribeTopic.private+"/closest_waypoint":
            message = self.autowareSubscribeTopic.unserialize(payload)
            if 0 <= message["index"] < len(self.current_poses):
                self.pose_index = message["index"]
                print(self.current_poses[self.pose_index])
                self.arrow_id = self.current_poses[self.pose_index]["arrow_id"]
                self.waypoint_id = self.current_poses[self.pose_index]["waypoint_id"]
                self.lat, self.lng = self.waypoint.get_latlng(self.waypoint_id)
                self.yaw = self.arrow.get_heading(self.arrow_id, self.waypoint_id)
            else:
                print("Lost Autoware.")

    def set_autoware_waypoints(self):
        print("sendWaypointsToAutoware")
        waypoints = []
        schedule = self.schedules[0]

        arrow_waypoint_array = self.route.getArrowWaypointArray({
            "startWaypointID": schedule["route"]["start"]["waypoint_id"],
            "goalWaypointID": schedule["route"]["goal"]["waypoint_id"],
            "arrowIDs": schedule["route"]["arrow_ids"]
        })
        for arrowWaypoint in arrow_waypoint_array:
            waypoint_id = arrowWaypoint["waypoint_id"]
            waypoint = self.waypoint.getWaypoint(waypoint_id)
            w, x, y, z = axangle2quat([0, 0, 1], waypoint["yaw"])
            waypoints.append({
                "position": {
                    "x": waypoint["x"],
                    "y": waypoint["y"],
                    "z": waypoint["z"],
                },
                "orientation": {
                    "w": w,
                    "x": x,
                    "y": y,
                    "z": z,
                },
                "velocity": 2.0
            })
        if 0 < len(waypoints):
            num = min(10, len(waypoints))
            for i in range(num-1, 0, -1):
                waypoints[-i]["velocity"] = (i/num)*waypoints[-i-1]["velocity"]
            self.current_poses = arrow_waypoint_array
            payload = self.autowarePublishTopic.serialize(waypoints)
            self.publish(self.autowarePublishTopic.private+"/waypoints", payload)
