#!/usr/bin/env python
# coding: utf-8

import unittest
import json
import sys
import re
from argparse import ArgumentParser
# from ams import Waypoint, Arrow, Intersection, Converter, MapMatch, Route, Location
from numpy import allclose, ndarray

import math
from ams.helpers import Vector, Rpy
# from ams.structures import Rpy #as Structure

parser = ArgumentParser()
# parser.add_argument("-AP", "--arrow_path", type=str, default="./test/res/arrow.json", help="arrow_path")
# parser.add_argument("-WP", "--waypoint_path", type=str, default="./test/res/waypoint.json", help="waypoint_path")
args = parser.parse_args()

class TestUtilityRpy(unittest.TestCase):

    # Structure = Structure
    Rpy = Rpy

    """test class of rpy.py

    test functions:
        def new_rpy(roll, pitch, yaw):
        def to_quaternion(vector, theta, is_normalized=False):
    """

    def __init__(self, methodName):
        """Create an instance of the class that will use the named test
           method when executed. Raises a ValueError if the instance does
           not have a method with the specified name.
        """
        super(TestUtilityRpy, self).__init__(methodName)
        # self.Rpy = Rpy ?
        # self.new_rpy = new_rpy ?
        # self.to_quaternion = to_quaternion ?

    def test_new_rpy(self):
        expected_new_rpy = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        roll = 0.0
        pitch = 0.0
        yaw = 0.0
        Rpy.new_rpy(roll,pitch,yaw)
        actual_new_rpy = self.Rpy.new_rpy(roll, pitch, yaw)
        self.assertEqual(expected_new_rpy, actual_new_rpy)
    # validate_rpy = Rpy.validate_data #Structure.validate_data
    # get_errors = Rpy.get_errors #Structure.get_errors

    def test_to_quaternion(self):
        expected_to_quaternion = 0

        # 初期値がわからなかったためエラーのままにしてある　薮田
        vector = [1,2]
        theta = 0.7
        Rpy.to_quaternion(vector,theta,is_normalized=False)

        actual_to_quaternion = self.rpy.quaternion(vector,theta)
        self.assertEqual(expected_to_quaternion, actual_to_quaternion)




if __name__ == "__main__":

    test_loader = unittest.TestLoader()
    test_names = test_loader.getTestCaseNames(TestUtilityRpy)

    suite = unittest.TestSuite()
    for test_name in test_names:
        suite.addTest(TestUtilityRpy(test_name))

    result = unittest.TextTestRunner().run(suite)
    sys.exit(not result.wasSuccessful())
