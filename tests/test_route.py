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

    def test_get_lane_array(self):
        with open("./res/lane_array_expected1.json", "r") as f:
            expected = json.load(f)

        arrows, to_arrows, from_arrows = Arrow.load("./res/arrow.json")
        waypoints = Waypoint.load("./res/waypoint.json")
        value = Route.get_lane_array(
            "10471:10471>9686:9686:9686<9673:9673:9673>9988:9988", arrows, waypoints, 0)
        self.assertEqual(expected, value)


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
