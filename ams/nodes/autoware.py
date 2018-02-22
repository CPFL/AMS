#!/usr/bin/env python
# coding: utf-8

from time import time

from ams import Topic, Route, Schedule, MapMatch
from ams.nodes import Vehicle, TrafficSignal
from ams.messages import CurrentPose, ClosestWaypoint, LaneArray, LightColor, TrafficSignalStatus
from ams.structures import AUTOWARE

from pprint import PrettyPrinter
pp = PrettyPrinter(indent=2).pprint


class Autoware(Vehicle):

    CONST = AUTOWARE

    def __init__(self, name, waypoint, arrow, route, dt=1.0):
        super().__init__(name, waypoint, arrow, route, dt=dt)

        self.name = name

        self.topicAutowarePub = Topic()
        self.topicAutowarePub.set_id(self.name)
        self.topicAutowarePub.set_root(AUTOWARE.TOPIC.PUBLISH)

        self.topicAutowareSub = Topic()
        self.topicAutowareSub.set_id(self.name)
        self.topicAutowareSub.set_root(AUTOWARE.TOPIC.SUBSCRIBE)

        self.topicTrafficSignalStatus = Topic()
        self.topicTrafficSignalStatus.set_root(TrafficSignal.CONST.TOPIC.PUBLISH)

        self.__map_match = MapMatch()
        self.__map_match.set_waypoint(self.waypoint)

        self.current_pose = None
        self.closest_waypoint = None
        self.current_arrow_waypoint_array = []
        self.traffic_signals = {}

        self.add_on_message_function(self.update_current_pose)
        self.add_on_message_function(self.update_closest_waypoint)
        self.add_on_message_function(self.update_traffic_signals)
        self.set_subscriber(self.topicAutowareSub.private+AUTOWARE.TOPIC.CURRENT_POSE)
        self.set_subscriber(self.topicAutowareSub.private+AUTOWARE.TOPIC.CLOSEST_WAYPOINT)
        self.set_subscriber(self.topicTrafficSignalStatus.all)

    def update_pose_from_closest_arrow_waypoint(self, closest_arrow_waypoint):
        self.arrow_code = closest_arrow_waypoint["arrow_code"]
        self.waypoint_id = closest_arrow_waypoint["waypoint_id"]
        self.np_position = self.waypoint.get_np_position(self.waypoint_id)
        self.yaw = self.arrow.get_yaw(self.arrow_code, self.waypoint_id)

    def update_pose_from_current_pose(self):
        location = self.__map_match.get_matched_location_on_arrows(self.current_pose, self.arrow.get_arrow_codes)
        self.arrow_code = location.arrow_code
        self.waypoint_id = location.waypoint_id
        self.np_position = self.waypoint.get_np_position(self.waypoint_id)
        self.yaw = self.arrow.get_yaw(self.arrow_code, self.waypoint_id)

    def update_current_pose(self, _client, _userdata, topic, payload):
        if topic == self.topicAutowareSub.private+AUTOWARE.TOPIC.CURRENT_POSE:
            self.current_pose = CurrentPose.new_data(**self.topicAutowareSub.unserialize(payload))

    def update_closest_waypoint(self, _client, _userdata, topic, payload):
        if topic == self.topicAutowareSub.private+AUTOWARE.TOPIC.CLOSEST_WAYPOINT:
            self.closest_waypoint = ClosestWaypoint.new_data(**self.topicAutowareSub.unserialize(payload))
            if 0 <= self.closest_waypoint.index < len(self.current_arrow_waypoint_array):
                self.update_pose_from_closest_arrow_waypoint(
                    self.current_arrow_waypoint_array[self.closest_waypoint.index])
                self.set_autoware_traffic_light()
            else:
                if self.current_pose is not None:
                    self.update_pose_from_current_pose()
                else:
                    print("Lost Autoware.")

    def get_lane_array_from_arrow_waypoint_array(self, arrow_waypoint_array):
        waypoints = []
        for arrow_waypoint in arrow_waypoint_array:
            waypoints.append({
                "pose": self.waypoint.get_pose(arrow_waypoint["waypoint_id"]),
                "velocity": 2.0
            })
        lanes = [{"waypoints": waypoints}]
        return LaneArray.new_data(
            name=self.name,
            time=time(),
            lanes=lanes
        )

    def update_autoware_waypoints(self):
        schedule = self.schedules[0]

        arrow_waypoint_array = self.route.get_arrow_waypoint_array(schedule.route)
        lane_array = self.get_lane_array_from_arrow_waypoint_array(arrow_waypoint_array)

        if 0 < len(lane_array.lanes.waypoints):
            num = min(10, len(lane_array.lanes.waypoints))
            for i in range(num-1, 0, -1):
                lane_array.lanes.waypoints[-i].velocity = (i/num)*lane_array.lanes.waypoints[-i-1].velocity
            self.current_arrow_waypoint_array = arrow_waypoint_array
            payload = self.topicAutowarePub.serialize(lane_array)
            self.publish(self.topicAutowarePub.private+AUTOWARE.TOPIC.WAYPOINTS, payload)

    def update_traffic_signals(self, _client, _user_data, topic, payload):
        if self.topicTrafficSignalStatus.root in topic:
            traffic_signal_status = TrafficSignalStatus.new_data(**self.topicTrafficSignalStatus.unserialize(payload))
            self.traffic_signals[traffic_signal_status.route_code] = traffic_signal_status

    def __get_inter_traffic_signal_distance(self, monitored_route):
        monitored_arrow_codes = monitored_route.arrow_codes
        inter_traffic_signal_distance = AUTOWARE.FLOAT_MAX

        not_green_traffic_signal_route_codes = list(map(
            lambda x: x.route_code, filter(
                lambda x: x.state in [TrafficSignal.CONST.STATE.YELLOW, TrafficSignal.CONST.STATE.RED],
                self.traffic_signals.values())))

        new_monitored_route = None
        for i, monitored_arrow_code in enumerate(monitored_arrow_codes):
            for not_green_traffic_signal_route_code in not_green_traffic_signal_route_codes:
                if monitored_arrow_code in not_green_traffic_signal_route_code:
                    not_green_traffic_signal_route = Route.decode_route_code(not_green_traffic_signal_route_code)
                    if monitored_arrow_code == not_green_traffic_signal_route.arrow_codes[0]:
                        waypoint_ids = self.arrow.get_waypoint_ids(monitored_arrow_code)
                        if self.waypoint_id not in waypoint_ids or \
                                waypoint_ids.index(self.waypoint_id) <= waypoint_ids.index(
                                    not_green_traffic_signal_route.start_waypoint_id):
                            new_monitored_route = Route.new_route(
                                monitored_route.start_waypoint_id,
                                not_green_traffic_signal_route.start_waypoint_id,
                                monitored_arrow_codes[:i+1])
                            break
            if new_monitored_route is not None:
                break

        if new_monitored_route is not None:
            inter_traffic_signal_distance = self.route.get_route_length(new_monitored_route)

        # print("inter_traffic_signal_distance {}[m]".format(inter_traffic_signal_distance))
        return inter_traffic_signal_distance

    def get_monitored_route(self, distance=100.0):
        if distance <= 0:
            return None

        arrow_codes = self.schedules[0].route.arrow_codes
        if arrow_codes is None:
            return None

        i_s = 0
        if self.arrow_code in arrow_codes:
            i_s = arrow_codes.index(self.arrow_code)
        arrow_codes = arrow_codes[i_s:]
        route = Route.new_route(
            self.waypoint_id,
            self.arrow.get_waypoint_ids(self.schedules[0].route.arrow_codes[-1])[-1],
            arrow_codes)
        return self.route.get_sliced_route(route, distance)

    def set_autoware_traffic_light(self):
        if self.schedules[0].event == Vehicle.CONST.ACTION.MOVE:
            monitored_route = self.get_monitored_route()
            if monitored_route is None:
                traffic_light = AUTOWARE.TRAFFIC_LIGHT.RED
            else:
                inter_traffic_signal_distance = self.__get_inter_traffic_signal_distance(monitored_route)
                # print("inter_traffic_signal_distance", inter_traffic_signal_distance)
                if inter_traffic_signal_distance <= 20.0:
                    traffic_light = AUTOWARE.TRAFFIC_LIGHT.RED
                else:
                    traffic_light = AUTOWARE.TRAFFIC_LIGHT.GREEN
            payload = self.topicAutowarePub.serialize(LightColor.new_data(
                name=self.name,
                time=time(),
                traffic_light=traffic_light
            ))
            # print("set_autoware_traffic_light", payload)
            self.publish(self.topicAutowarePub.private+AUTOWARE.TOPIC.TRAFFIC_LIGHT, payload)

    def update_status(self):
        if None not in [self.waypoint_id, self.arrow_code]:
            current_time = time()
            if self.state == Vehicle.CONST.STATE.STOP:
                if self.schedules[0].period.end < current_time:
                    self.schedules.pop(0)

                    self.update_autoware_waypoints()

                    # update next schedule
                    dif_time = current_time - self.schedules[0].period.start
                    self.schedules = Schedule.get_shifted_schedules(self.schedules, dif_time)

                    self.state = Vehicle.CONST.STATE.MOVE
