#!/usr/bin/env python
# coding: utf-8

import unittest
import json
import sys
import re
from argparse import ArgumentParser
from ams import Waypoint, Arrow, Intersection, Converter, MapMatch, Route, Location
from numpy import allclose, ndarray


parser = ArgumentParser()
parser.add_argument("-AP", "--arrow_path", type=str, default="./test/res/arrow.json", help="arrow_path")
parser.add_argument("-WP", "--waypoint_path", type=str, default="./test/res/waypoint.json", help="waypoint_path")
args = parser.parse_args()


class TestUtilityMapMatch(unittest.TestCase):

    """test class of converter.py

    test functions:
        def get_similarity_between_poses(pose1, pose2):
        def get_matched_location_on_route(self, pose, route):
        def get_matched_location_on_arrows(self, pose, arrow_codes=None):
    """

    def __init__(self, methodName, arrow_path, waypoint_path):
        """Create an instance of the class that will use the named test
           method when executed. Raises a ValueError if the instance does
           not have a method with the specified name.
        """
        super(TestUtilityMapMatch, self).__init__(methodName)
        self.arrow_path = arrow_path
        self.waypoint_path = waypoint_path
        self.mapmatch = MapMatch()
        self.mapmatch.set_arrow(Arrow())
        self.__waypoint = Waypoint()
        self.__waypoint.load(waypoint_path)
        self.__arrow = Arrow(self.__waypoint)
        self.__arrow.load(self.arrow_path)
        self.mapmatch.set_arrow(self.__arrow)
        self.mapmatch.set_waypoint(self.__waypoint)
        self.__route = Route()
        with open(self.arrow_path, "r") as f:
            self.data = json.load(f)


    def test_get_matched_location_on_route(self):
        expected_get_matched_location_on_route = {'geohash': '123456789012345', 'waypoint_id': '8926', 'arrow_code': '8926_8936'}
        arrow_code = ["8855_8871"]
        arrow_codes = ["8926_8936", "9044_9054", "9151_9155"]
        route = Route().new_route("8926", "9155", arrow_codes)

        waypoint_id = "8823"
        pose = self.__waypoint.get_pose(waypoint_id)
        actual_get_matched_location_on_route = self.mapmatch.get_matched_location_on_route(pose, route)
        self.assertEqual(expected_get_matched_location_on_route, actual_get_matched_location_on_route)

    def test_get_matched_location_on_arrows(self):
        expected_get_matched_location_on_arrows = {'geohash': None, 'waypoint_id': '8805', 'arrow_code': '8805_8855'}
        arrow_code = ["8855_8871"]
        arrow_codes = ["8926_8936", "9044_9054", "9151_9155"]

        waypoint_id = "8823"
        pose = self.__waypoint.get_pose(waypoint_id)
        actual_get_matched_location_on_arrows = self.mapmatch.get_matched_location_on_arrows(pose)
        self.assertEqual(expected_get_matched_location_on_arrows, actual_get_matched_location_on_arrows)



if __name__ == "__main__":

    test_loader = unittest.TestLoader()
    test_names = test_loader.getTestCaseNames(TestUtilityMapMatch)

    suite = unittest.TestSuite()
    for test_name in test_names:
        suite.addTest(TestUtilityMapMatch(test_name, args.arrow_path, args.waypoint_path))

    result = unittest.TextTestRunner().run(suite)
    sys.exit(not result.wasSuccessful())
