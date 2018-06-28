#!/usr/bin/env python
# coding: utf-8

import unittest
import json
import sys
import re
from argparse import ArgumentParser
from ams import Waypoint, Arrow, Intersection, Converter
from numpy import allclose, ndarray


parser = ArgumentParser()
args = parser.parse_args()


class TestUtilityConverter(unittest.TestCase):

    """test class of converter.py

    test functions:
        def camel_case_to_snake_case(camel_case):
    """

    def __init__(self, methodName):
        """Create an instance of the class that will use the named test
           method when executed. Raises a ValueError if the instance does
           not have a method with the specified name.
        """
        super(TestUtilityConverter, self).__init__(methodName)

        self.converter = Converter()
    

    def test_camel_case_to_snake_case(self):
        expected_camel_case_to_snake_case = "paison_muzui"
        camel_case = "PaisonMuzui"
        s1 = re.sub("(.)([A-Z][a-z]+)", r"\1_\2", camel_case)
        actual_camel_case_to_snake_case = self.converter.camel_case_to_snake_case(camel_case)
        self.assertEqual(expected_camel_case_to_snake_case, actual_camel_case_to_snake_case)

    def test_camel_case_to_snake_case(self):
        expected_camel_case_to_snake_case = "hongoh_campus"
        camel_case = "hongohCampus"
        s1 = re.sub("(.)([A-Z][a-z]+)", r"\1_\2", camel_case)
        actual_camel_case_to_snake_case = self.converter.camel_case_to_snake_case(camel_case)
        self.assertEqual(expected_camel_case_to_snake_case, actual_camel_case_to_snake_case)

    def test_camel_case_to_snake_case(self):
        expected_camel_case_to_snake_case = "yellow_balls_red_balls"
        camel_case = "YellowBallsRedBalls"
        s1 = re.sub("(.)([A-Z][a-z]+)", r"\1_\2", camel_case)
        actual_camel_case_to_snake_case = self.converter.camel_case_to_snake_case(camel_case)
        self.assertEqual(expected_camel_case_to_snake_case, actual_camel_case_to_snake_case)


if __name__ == "__main__":

    test_loader = unittest.TestLoader()
    test_names = test_loader.getTestCaseNames(TestUtilityConverter)

    suite = unittest.TestSuite()
    for test_name in test_names:
        suite.addTest(TestUtilityConverter(test_name))

    result = unittest.TextTestRunner().run(suite)
    sys.exit(not result.wasSuccessful())
