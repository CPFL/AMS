#!/usr/bin/env python
# coding: utf-8

import sys
import unittest
import json

from ams.helpers import Route, Arrow, Waypoint


class TestRoute(unittest.TestCase):

    def __init__(self, method_name):

        super(TestRoute, self).__init__(method_name)

    def test_encode(self):
        expected = "1:0>2>4:3"
        value = Route.encode(Route.new_route(
            waypoint_ids=["1", "3"],
            arrow_codes=["0_2", "2_4"]
        ))
        self.assertEqual(expected, value)

        expected = "1:0>2:2:2<0:1:0>2>4:3"
        value = Route.encode(Route.new_route(
            waypoint_ids=["1", "2", "1", "3"],
            arrow_codes=["0_2", "0_2", "0_2", "2_4"],
            delimiters=[":", ">", ":", ":", "<", ":", ":", ">", ">", ":"]
        ))
        self.assertEqual(expected, value)

    def test_decode(self):
        expected = Route.new_route(
            waypoint_ids=["1", "2", "1", "3"],
            arrow_codes=["0_2", "0_2", "0_2", "2_4"],
            delimiters=[":", ">", ":", ":", "<", ":", ":", ">", ">", ":"]
        )
        value = Route.decode("1:0>2:2:2<0:1:0>2>4:3")
        self.assertEqual(expected, value)

        expected = Route.new_route(
            waypoint_ids=["1942", "2078"],
            arrow_codes=["2078_1942"],
            delimiters=[":", "<", ":"]
        )
        value = Route.decode("1942:1942<2078:2078")
        self.assertEqual(expected, value)

    def test_get_routes_divided_by_action(self):
        expected = [
            Route.new_route(
                waypoint_ids=["1", "2"],
                arrow_codes=["0_2"],
                delimiters=[":", ">", ":"]
            ),
            Route.new_route(
                waypoint_ids=["2", "1"],
                arrow_codes=["0_2"],
                delimiters=[":", "<", ":"]
            ),
            Route.new_route(
                waypoint_ids=["1", "3"],
                arrow_codes=["0_2", "2_4"],
                delimiters=[":", ">", ">", ":"]
            )
        ]
        value = Route.get_routes_divided_by_action(
            Route.new_route(
                waypoint_ids=["1", "2", "1", "3"],
                arrow_codes=["0_2", "0_2", "0_2", "2_4"],
                delimiters=[":", ">", ":", ":", "<", ":", ":", ">", ">", ":"]
            )
        )
        self.assertEqual(expected, value)

    def test_get_shortest_routes(self):
        arrows, to_arrows, from_arrows = Arrow.load("./res/arrow.json")
        waypoints = Waypoint.load("./res/waypoint.json")
        cost_function = Route.get_length
        start = {
            "waypoint_id": "9910",
            "arrow_code": "9908_9930"
        }
        goals = [
            {
                "goal_id": "test1",
                "waypoint_id": "10125",
                "arrow_code": "10121_8973"
            }
        ]
        value = Route.get_shortest_routes(
            start, goals, arrows, to_arrows, from_arrows, waypoints, cost_function)
        expected = {
            'test1': {
                'waypoint_ids': ['9910', '10125'],
                'arrow_codes': [
                    '9908_9930', '9930_9935', '9935_9054', '9054_9059', '9059_10067', '10067_10106', '10106_10121',
                    '10121_8973'
                ],
                'goal_id': 'test1',
                'cost': 139.6795663094191
            }
        }
        self.assertEqual(expected, value)

    def test_get_nth_pose_and_location(self):
        arrows, _, _ = Arrow.load("./res/arrow.json")
        waypoints = Waypoint.load("./res/waypoint.json")
        route_code = "10502:10471>9686:9686:9686<9673:9673:9673>9988:9676"
        expected = (
            {
                'position': {'x': 3753.102, 'y': -99405.981, 'z': 85.725},
                'orientation': {
                    'quaternion': {'w': -0.868629124833237, 'x': 0.0, 'y': 0.0, 'z': 0.4954628578323958},
                    'rpy': {'roll': None, 'pitch': None, 'yaw': 5.246450085504264}
                }
            },
            {
                'waypoint_id': '10502', 'arrow_code': '10471_9686', 'geohash': None
            }
        )
        value = Route.get_nth_pose_and_location(0, route_code, arrows, waypoints)
        self.assertEqual(expected, value)

        expected = (
            {
                'position': {'x': 3754.346, 'y': -99399.875, 'z': 85.696},
                'orientation': {
                    'quaternion': {'w': -0.6971157998367005, 'x': 0.0, 'y': 0.0, 'z': 0.7169585494420423},
                    'rpy': {'roll': None, 'pitch': None, 'yaw': 4.684326173951593}}},
            {
                'waypoint_id': '9676', 'arrow_code': '9673_9988', 'geohash': None
            }
        )
        value = Route.get_nth_pose_and_location(22, route_code, arrows, waypoints)
        self.assertEqual(expected, value)

        route_code = "10472:10471>9686:9686:9686<9673:9673:9673>9988:10333"
        expected = (
            {
                'position': {'x': 3754.354, 'y': -99409.875, 'z': 85.677},
                'orientation': {
                    'quaternion': {'w': -0.7074602460561826, 'x': 0.0, 'y': 0.0, 'z': 0.7067531395403387},
                    'rpy': {'roll': None, 'pitch': None, 'yaw': 4.713388980051105}
                }
            },
            {
                'waypoint_id': '9686', 'arrow_code': '9673_9686', 'geohash': None
            }
        )
        value = Route.get_nth_pose_and_location(35, route_code, arrows, waypoints)
        self.assertEqual(expected, value)

        route_code = "9883:9875>9887>9563:9563"
        expected = (None, None)
        value = Route.get_nth_pose_and_location(100, route_code, arrows, waypoints)
        self.assertEqual(expected, value)


    def test_get_pose_and_velocity_set(self):
        with open("./res/get_pose_and_velocity_set_expected1.json", "r") as f:
            expected = json.load(f)
        arrows, _, _ = Arrow.load("./res/arrow.json")
        waypoints = Waypoint.load("./res/waypoint.json")
        value = Route.get_pose_and_velocity_set(
            "10502:10471>9686:9686:9686<9673:9673:9673>9988:9676", arrows, waypoints)
        self.assertEqual(expected, value)

    def test_get_lane_array(self):
        with open("./tests/res/lane_array_expected1.json", "r") as f:
            expected = json.load(f)

        arrows, _, _ = Arrow.load("./res/arrow.json")
        waypoints = Waypoint.load("./res/waypoint.json")
        value1 = Route.get_lane_array(
            "10471:10471>9686:9686:9686<9673:9673:9673>9988:9988", arrows, waypoints, 0)
        self.assertEqual(expected, value1)
        value2 = Route.get_lane_array(
            "10471:10471>9686:9686:9686<9673:9673:9673>9988:9988", arrows, waypoints, 0)
        self.assertEqual(expected, value2)


def test_route(test_loader, suite):
    test_names = test_loader.getTestCaseNames(TestRoute)

    for test_name in test_names:
        suite.addTest(TestRoute(test_name))

    return suite


if __name__ == "__main__":

    test_loader = unittest.TestLoader()
    suite = unittest.TestSuite()

    test_route(test_loader, suite)

    result = unittest.TextTestRunner().run(suite)
    sys.exit(not result.wasSuccessful())
