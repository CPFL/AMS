#!/usr/bin/env python
# coding: utf-8

import unittest
import json

from ams.helpers import Route, Lane, Waypoint
from ams.structures import RoutePoint, RouteSection


class Test(unittest.TestCase):

    def __init__(self, method_name):

        super(Test, self).__init__(method_name)

    def test_encode(self):
        expected = "1:0>2>4:3"
        value = Route.encode(Route.new_route(
            waypoint_ids=["1", "3"],
            lane_codes=["0_2", "2_4"]
        ))
        self.assertEqual(expected, value)

        expected = "1:0>2:2:2<0:1:0>2>4:3"
        value = Route.encode(Route.new_route(
            waypoint_ids=["1", "2", "1", "3"],
            lane_codes=["0_2", "0_2", "0_2", "2_4"],
            delimiters=[":", ">", ":", ":", "<", ":", ":", ">", ">", ":"]
        ))
        self.assertEqual(expected, value)

    def test_decode(self):
        expected = Route.new_route(
            waypoint_ids=["1", "2", "1", "3"],
            lane_codes=["0_2", "0_2", "0_2", "2_4"],
            delimiters=[":", ">", ":", ":", "<", ":", ":", ">", ">", ":"]
        )
        value = Route.decode("1:0>2:2:2<0:1:0>2>4:3")
        self.assertEqual(expected, value)

        expected = Route.new_route(
            waypoint_ids=["1942", "2078"],
            lane_codes=["2078_1942"],
            delimiters=[":", "<", ":"]
        )
        value = Route.decode("1942:1942<2078:2078")
        self.assertEqual(expected, value)

    def test_get_routes_divided_by_action(self):
        expected = [
            Route.new_route(
                waypoint_ids=["1", "2"],
                lane_codes=["0_2"],
                delimiters=[":", ">", ":"]
            ),
            Route.new_route(
                waypoint_ids=["2", "1"],
                lane_codes=["0_2"],
                delimiters=[":", "<", ":"]
            ),
            Route.new_route(
                waypoint_ids=["1", "3"],
                lane_codes=["0_2", "2_4"],
                delimiters=[":", ">", ">", ":"]
            )
        ]
        value = Route.get_routes_divided_by_action(
            Route.new_route(
                waypoint_ids=["1", "2", "1", "3"],
                lane_codes=["0_2", "0_2", "0_2", "2_4"],
                delimiters=[":", ">", ":", ":", "<", ":", ":", ">", ">", ":"]
            )
        )
        self.assertEqual(expected, value)

    def test_get_waypoint_ids(self):
        lanes, _, _ = Lane.load("./res/maps/lane.json")
        with open("./tests/res/route/test_get_waypoint_ids.json", "r") as f:
            resources = json.load(f)
        for resource in resources:
            value = Route.get_waypoint_ids(resource["route_code"], lanes)
            self.assertEqual(resource["expected"], value)

    def test_get_route_point_pose_and_location(self):
        lanes, _, _ = Lane.load("./res/maps/lane.json")
        waypoints = Waypoint.load("./res/maps/waypoint.json")
        route_point = RoutePoint.new_data(**{
            "route_code": "10502:10471>9686:9686:9686<9673:9673:9673>9988:9676",
            "index": 0
        })
        expected = (
            {
                'position': {'x': 3753.102, 'y': -99405.981, 'z': 85.725},
                'orientation': {
                    'quaternion': {'w': -0.868629124833237, 'x': 0.0, 'y': 0.0, 'z': 0.4954628578323958},
                    'rpy': {'roll': None, 'pitch': None, 'yaw': 5.246450085504264}
                }
            },
            {
                'waypoint_id': '10502', 'lane_code': '10471_9686', 'geohash': None
            }
        )
        value = Route.get_route_point_pose_and_location(route_point, lanes, waypoints)
        self.assertEqual(expected, value)

        expected = (
            {
                'position': {'x': 3754.346, 'y': -99399.875, 'z': 85.696},
                'orientation': {
                    'quaternion': {'w': -0.6971157998367005, 'x': 0.0, 'y': 0.0, 'z': 0.7169585494420423},
                    'rpy': {'roll': None, 'pitch': None, 'yaw': 4.684326173951593}}},
            {
                'waypoint_id': '9676', 'lane_code': '9673_9988', 'geohash': None
            }
        )
        route_point.index = 20
        value = Route.get_route_point_pose_and_location(route_point, lanes, waypoints)
        self.assertEqual(expected, value)

        expected = (
            {
                'position': {'x': 3754.353, 'y': -99408.875, 'z': 85.681},
                'orientation': {
                    'quaternion': {'w': -0.707283535768144, 'x': 0.0, 'y': 0.0, 'z': 0.7069299824107849},
                    'rpy': {'roll': None, 'pitch': None, 'yaw': 4.712888980342898}
                }
            },
            {
                'waypoint_id': '9685', 'lane_code': '9673_9686', 'geohash': None
            }
        )
        route_point.route_code = "10472:10471>9686:9686:9686<9673:9673:9673>9988:10333"
        route_point.index = 35
        value = Route.get_route_point_pose_and_location(route_point, lanes, waypoints)
        self.assertEqual(expected, value)

        route_point.route_code = "9883:9875>9887>9563:9563"
        route_point.index = 100
        expected = (None, None)
        value = Route.get_route_point_pose_and_location(route_point, lanes, waypoints)
        self.assertEqual(expected, value)

    def test_get_pose_and_velocity_set(self):
        with open("./tests/res/route/get_pose_and_velocity_set_expected1.json", "r") as f:
            expected = json.load(f)
        lanes, _, _ = Lane.load("./res/maps/lane.json")
        waypoints = Waypoint.load("./res/maps/waypoint.json")
        value = Route.get_pose_and_velocity_set(
            "10502:10471>9686:9686:9686<9673:9673:9673>9988:9676", lanes, waypoints)
        self.assertEqual(expected, value)

    def test_generate_lane_array(self):
        with open("./tests/res/route/lane_array_expected1.json", "r") as f:
            expected = json.load(f)["lanes"]

        lanes, _, _ = Lane.load("./res/maps/lane.json")
        waypoints = Waypoint.load("./res/maps/waypoint.json")
        value1 = Route.generate_lane_array(
            "10471:10471>9686:9686:9686<9673:9673:9673>9988:9988", lanes, waypoints, 0).lanes
        self.assertEqual(expected, value1)
        value2 = Route.generate_lane_array(
            "10471:10471>9686:9686:9686<9673:9673:9673>9988:9988", lanes, waypoints, 0).lanes
        self.assertEqual(expected, value2)

    def test_get_shortest_routes(self):
        lanes, to_lanes, from_lanes = Lane.load("./res/maps/lane.json")
        waypoints = Waypoint.load("./res/maps/waypoint.json")
        cost_function = Route.get_length
        start = {
            "waypoint_id": "9910",
            "lane_code": "9908_9930"
        }
        goals = [
            {
                "goal_id": "test1",
                "waypoint_id": "10125",
                "lane_code": "10121_8973"
            }
        ]
        value = Route.get_shortest_routes(
            start, goals, lanes, to_lanes, from_lanes, waypoints, cost_function)
        expected = {
            'test1': {
                'waypoint_ids': ['9910', '10125'],
                'lane_codes': [
                    '9908_9930', '9930_9935', '9935_9054', '9054_9059', '9059_10067', '10067_10106', '10106_10121',
                    '10121_8973'
                ],
                'goal_id': 'test1',
                'cost': 139.6795663094191
            }
        }
        self.assertEqual(expected, value)

    def test_generate_route_code_from_polyline(self):
        lanes, to_lanes, from_lanes = Lane.load("./res/maps/lane.json")
        waypoints = Waypoint.load("./res/maps/waypoint.json")
        with open("./tests/res/route/test_generate_route_code_from_polyline.json", "r") as f:
            resources = json.load(f)
        for resource in resources:
            value = Route.generate_route_code_from_polyline(
                resource["polyline_code"], lanes, waypoints, to_lanes, from_lanes,
                start_waypoint_id=None if "start_waypoint_id" not in resource else resource["start_waypoint_id"],
                goal_waypoint_id=None if "goal_waypoint_id" not in resource else resource["goal_waypoint_id"]
            )
            self.assertEqual(resource["expected"], value)

    def test_generate_route_section_with_route_codes(self):
        lanes, _, _ = Lane.load("./res/entre/lane.json")
        with open("./tests/res/test_generate_route_section_with_route_codes.json", "r") as f:
            resources = json.load(f)
        for resource in resources:
            value = Route.generate_route_section_with_route_codes(
                resource["inner_route_code"], resource["outer_route_code"], lanes)
            self.assertEqual(resource["expected"], value)

    def test_calculate_route_section_length(self):
        lanes, _, _ = Lane.load("./res/maps/lane.json")
        waypoints = Waypoint.load("./res/maps/waypoint.json")
        value = Route.calculate_route_section_length(
            RouteSection.new_data(**{
                "route_code": "10471:10471>9686:9686:9686<9673:9673:9673>9988:9988",
                "start_index": 7,
                "end_index": 19
            }),
            lanes, waypoints
        )
        self.assertEqual(12.00029999395704, value)

    def test_calculate_distance_from_route_point_to_inner_route(self):
        lanes, _, _ = Lane.load("./res/maps/lane.json")
        waypoints = Waypoint.load("./res/maps/waypoint.json")
        value = Route.calculate_distance_from_route_point_to_inner_route(
            RoutePoint.new_data(**{
                "route_code": "10471:10471>9686:9686:9686<9673:9673:9673>9988:9988",
                "index": 7
            }),
            "10490:10471>9686:9686:9686<9673:9673:9673>9988:9988",
            lanes, waypoints
        )
        self.assertEqual(12.00029999395704, value)
