#!/usr/bin/env python
# coding: utf-8

import sys
import unittest

from ams.helpers import Route


class TestRouteHelper(unittest.TestCase):

    def __init__(self, method_name):

        super(TestRouteHelper, self).__init__(method_name)

    def test_join_arrow_codes(self):
        arrow_codes = ["0_1", "0_1", "1_2"]
        joined_arrow_codes = Route.join_arrow_codes(arrow_codes)
        expected = "0>1<0>1>2"
        self.assertEqual(expected, joined_arrow_codes)

    def test_encode(self):
        expected = "1:0>2>4:3"
        route = Route.new_route(
            start_waypoint_id="1",
            goal_waypoint_id="3",
            arrow_codes=["0_2", "2_4"]
        )
        route_code = Route.encode(route)
        self.assertEqual(expected, route_code)

        expected = "1:0>2<0>2>4:3"
        route = Route.new_route(
            start_waypoint_id="1",
            goal_waypoint_id="3",
            arrow_codes=["0_2", "0_2", "2_4"]
        )
        route_code = Route.encode(route)
        self.assertEqual(expected, route_code)

    def test_decode(self):
        route_code = "1:0>2<0>2>4:3"
        expected = Route.new_route(
            start_waypoint_id="1",
            goal_waypoint_id="3",
            arrow_codes=["0_2", "0_2", "2_4"]
        )
        route = Route.decode(route_code)
        self.assertEqual(expected, route)


def test_route_helper(test_loader, suite):
    test_names = test_loader.getTestCaseNames(TestRouteHelper)

    for test_name in test_names:
        suite.addTest(TestRouteHelper(test_name))

    return suite


if __name__ == "__main__":

    test_loader = unittest.TestLoader()
    suite = unittest.TestSuite()

    test_route_helper(test_loader, suite)

    result = unittest.TextTestRunner().run(suite)
    sys.exit(not result.wasSuccessful())
