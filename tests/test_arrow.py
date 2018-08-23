#!/usr/bin/env python
# coding: utf-8

import sys
import unittest
from ams.helpers import Arrow, Position


class TestArrow(unittest.TestCase):

    def __init__(self, method_name):

        super(TestArrow, self).__init__(method_name)

    def test_get_distance(self):
        position1 = Position.new_position(1.0, 2.0, 3.0)
        position2 = Position.new_position(4.0, 2.0, -1.0)
        value = Arrow.get_distance(position1, position2)
        expected = 5.0
        self.assertEqual(expected, value)

    def test_split_arrow_code(self):
        arrow_code = "0_1"
        value = Arrow.split_arrow_code(arrow_code)
        expected = ["0", "1"]
        self.assertEqual(expected, value)


def test_arrow(test_loader, suite):
    test_names = test_loader.getTestCaseNames(TestArrow)

    for test_name in test_names:
        suite.addTest(TestArrow(test_name))


if __name__ == "__main__":

    test_loader = unittest.TestLoader()
    suite = unittest.TestSuite()

    test_arrow(test_loader, suite)

    result = unittest.TextTestRunner().run(suite)
    sys.exit(not result.wasSuccessful())
