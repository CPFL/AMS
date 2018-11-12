#!/usr/bin/env python
# coding: utf-8

import sys
import unittest
from ams.helpers import Lane, Position


class Test(unittest.TestCase):

    def __init__(self, method_name):

        super(Test, self).__init__(method_name)

    def test_get_distance(self):
        position1 = Position.new_position(1.0, 2.0, 3.0)
        position2 = Position.new_position(4.0, 2.0, -1.0)
        value = Lane.get_distance(position1, position2)
        expected = 5.0
        self.assertEqual(expected, value)

    def test_split_lane_code(self):
        lane_code = "0_1"
        value = Lane.split_lane_code(lane_code)
        expected = ["0", "1"]
        self.assertEqual(expected, value)
