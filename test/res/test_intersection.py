#!/usr/bin/env python
# coding: utf-8

import unittest
import json
import sys
from argparse import ArgumentParser
from ams import Waypoint, Arrow, Intersection
from numpy import allclose, ndarray


parser = ArgumentParser()
parser.add_argument("-IP", "--intersection_path", type=str, default="./test/res/intersection.json", help="intersection_path")
args = parser.parse_args()


class TestUtilityIntersection(unittest.TestCase):

    """test class of intersection.py

    test functions:
        def __init__(self):
        def load(self, path):
        def connect_to_redis(self, _host, _port, _dbname):
        def set_intersections(self, intersections):
        def get_intersections(self):
        def get_intersection(self, intersection_id):
        def get_intersection_ids(self):
        def get_traffic_signals(self, intersection_id):
        def get_entry_exit_route_ids(self, intersection_id):
        def get_border_point(self, intersection_id, arrow_code):
        def get_to_in_border_waypoint_ids(self, intersection_id):
        def get_to_in_arrow_codes(self, intersection_id):
        def get_entry_exit_route_arrow_codes_set(self, intersection_id):
        def get_entry_exit_route_ids_in_arrow_codes(self, intersection_id, arrow_codes):
        def get_arrow_codes_of_entry_exit_route(self, intersection_id, entry_exit_route_id):
    """

    def __init__(self, methodName, intersection_path):
        """Create an instance of the class that will use the named test
           method when executed. Raises a ValueError if the instance does
           not have a method with the specified name.
        """
        super(TestUtilityIntersection, self).__init__(methodName)


        # Set Common variable
        self.intersection_path = intersection_path
        self.intersection = Intersection()
        self.intersection.load(intersection_path)

#    def load(self, path):
    def test_load(self):
        """test method for load
        arg:
         string

        return:
         bool
        """

        expected = True
        actual = self.intersection.load(self.intersection_path)
        self.assertEqual(expected, actual)

    """
    def test_get_border_point(self):
        expected_border_point = 0
        arrow_code = ["8855_8871"]
        actual_border_point = self.intersection.get_border_point("550", arrow_code)
        self.assertEqual(expected_border_point, actual_border_point)

    def test_get_entry_exit_route_ids_in_arrow_codes(self):
        expected_entry_exit_route_ids_in_arrow_codes = 0
        entry_exit_route_ids = []
        to_in_arrow_codes = self.intersection.get_to_in_arrow_codes("550")
        for i, arrow_code in enumerate(arrow_codes):
            if arrow_code in to_in_arrow_codes:
                for entry_exit_route_id, entryExitRoute in \
                        self.__intersections[intersection_id]["entryExitRoutes"].items():
                    entry_exit_route_arrow_codes = entryExitRoute["arrow_codes"]
                    length = len(entry_exit_route_arrow_codes)
                    if arrow_codes[i:i+length] == entry_exit_route_arrow_codes:
                        entry_exit_route_ids.append(entry_exit_route_id)
        actual_entry_exit_route_ids_in_arrow_codes = \
            self.intersection.get_entry_exit_route_ids_in_arrow_codes(["8805_8855", "8897_8910", "9044_9054"])
        self.assertEqual(expected_entry_exit_route_ids_in_arrow_codes, actual_entry_exit_route_ids_in_arrow_codes)
    """

if __name__ == "__main__":

    test_loader = unittest.TestLoader()
    test_names = test_loader.getTestCaseNames(TestUtilityIntersection)

    suite = unittest.TestSuite()
    for test_name in test_names:
        suite.addTest(TestUtilityIntersection(test_name, args.intersection_path))

    result = unittest.TextTestRunner().run(suite)
    sys.exit(not result.wasSuccessful())
