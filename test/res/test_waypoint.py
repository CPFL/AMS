#!/usr/bin/env python
# coding: utf-8

import unittest
import json
import sys
from argparse import ArgumentParser
from ams import Waypoint, Arrow
from numpy import allclose, ndarray


parser = ArgumentParser()
parser.add_argument("-WP", "--waypoint_path", type=str, default="./test/res/waypoint.json", help="waypoint_path")
args = parser.parse_args()


class TestUtilityWaypoint(unittest.TestCase):

    """test class of waypoint.py

    test functions:
        def __init__(self):
        def load(self, path):
        def connect_to_redis(self, _host, _port, _dbname):
        def set_waypoints(self, waypoints):
        def get_waypoint_ids(self):
        def get_latlng(self, waypoint_id):
        def get_geohash(self, waypoint_id):
        def get_np_position(self, waypoint_id):
        def get_position(self, waypoint_id):
        def get_xyz(self, waypoint_id):
        def get_yaw(self, waypoint_id):
        def get_orientation(self, waypoint_id):
        def get_pose(self, waypoint_id):
        def get_speed_limit(self, waypoint_id):
    """

    def __init__(self, methodName, waypoint_path):
        """Create an instance of the class that will use the named test
           method when executed. Raises a ValueError if the instance does
           not have a method with the specified name.
        """
        super(TestUtilityWaypoint, self).__init__(methodName)


        # Set Common variable
        self.waypoint_path = waypoint_path

        self.waypoint = Waypoint()
        self.waypoint.load(waypoint_path)


#    def load(self, path):
    def test_load(self):
        """test method for load
        arg:
         string

        return:
         bool
        """

        expected = True
        actual = self.waypoint.load(self.waypoint_path)
        self.assertEqual(expected, actual)

    def test_get_latlng(self):
        expected_latlng = (35.103619889196125, 137.2084355554398)
        actulal_latlng = self.waypoint.get_latlng("8808")
        self.assertEqual(expected_latlng, actulal_latlng)


if __name__ == "__main__":

    test_loader = unittest.TestLoader()
    test_names = test_loader.getTestCaseNames(TestUtilityWaypoint)

    suite = unittest.TestSuite()
    for test_name in test_names:
        suite.addTest(TestUtilityWaypoint(test_name, args.waypoint_path))

    result = unittest.TextTestRunner().run(suite)
    sys.exit(not result.wasSuccessful())
