#!/usr/bin/env python
# coding: utf-8

import unittest
import json
from ams import Waypoint, Arrow
from ams.structures import Arrow as Arrow_structure
from numpy import allclose, ndarray, unicode


class TestUtilityArrow(unittest.TestCase):

    """test class of arrow.py

    test functions:
        def load(self, path):
        def connect_to_redis(self, _host, _port, _dbname):
        def set_arrows(self, arrows, to_arrows, from_arrows):
        def get_arrow(self, arrow_code):
        def get_arrows(self):
        def get_to_arrows(self):
        def get_from_arrows(self):
        def get_waypoint_ids(self, arrow_code):
        def get_length(self, arrow_code):
        def get_distance(position1, position2):
        def get_yaw(self, arrow_code, waypoint_id):
        def get_heading(self, arrow_code, waypoint_id):
        def get_point_to_edge(self, point_position, edge_position1, edge_position2):
        def get_point_to_waypoints(self, point_position, waypoint_ids):
        def get_point_to_arrow(self, point_position, arrow_code):
        def get_point_to_arrows(self, point_position, arrow_codes=None):
        def get_arrow_codes_to_arrows(self, arrow_codes):
        def get_arrow_codes_from_waypoint_id(self, waypoint_id):
        def get_arrow_codes_set_from_waypoint_ids(self, waypoint_ids):
        def split_arrow_code(arrow_code):
    """

    arrow_path = "./res/arrow.json"
    waypoint = Waypoint()
    waypoint.load("./res/waypoint.json")

    def __init__(self, methodName='runTest'):
        """Create an instance of the class that will use the named test
           method when executed. Raises a ValueError if the instance does
           not have a method with the specified name.
        """
        super().__init__(methodName)
        self._testMethodName = methodName
        self._resultForDoCleanups = None
        try:
            test_method = getattr(self, methodName)
        except AttributeError:
            raise ValueError("no such test method in %s: %s" % (self.__class__, methodName))
        self._testMethodDoc = test_method.__doc__
        self._cleanups = []

        # Map types to custom assertEqual functions that will compare
        # instances of said type in more detail to generate a more useful
        # error message.
        self._type_equality_funcs = {}
        self.addTypeEqualityFunc(dict, 'assertDictEqual')
        self.addTypeEqualityFunc(list, 'assertListEqual')
        self.addTypeEqualityFunc(tuple, 'assertTupleEqual')
        self.addTypeEqualityFunc(set, 'assertSetEqual')
        self.addTypeEqualityFunc(frozenset, 'assertSetEqual')
        try:
            self.addTypeEqualityFunc(unicode, 'assertMultiLineEqual')
        except NameError:
            # No unicode support in this build
            pass

        # override to add original equality function
        self.addTypeEqualityFunc(type(Arrow_structure.get_template()), self.assert_arrow)
        self.addTypeEqualityFunc(ndarray, self.assert_np_array)

        # Set Common variable

        self.arrow = Arrow(self.waypoint)
        self.arrow.load(self.arrow_path)

        with open(self.arrow_path, "r") as f:
            self.data = json.load(f)

    def assert_np_array(self, first, second, msg=None):
        if allclose(first, second) is False:
            standard_msg = "np array do not match. %s  :  %s" % (str(first), str(second))
            self.fail(self._formatMessage(msg, standard_msg))

    def assert_arrow(self, first, second, msg=None):
        if Arrow_structure.validate_data(first) is False:
            standard_msg = "fail to validate Arrow. %s" % (str(first))
            self.fail(self._formatMessage(msg, standard_msg))
        if Arrow_structure.validate_data(second) is False:
            standard_msg = "fail to validate Arrow. %s" % (str(second))
            self.fail(self._formatMessage(msg, standard_msg))

        if first != second:
            standard_msg = "Arrow data do not match. %s  :  %s" % (str(first), str(second))
            self.fail(self._formatMessage(msg, standard_msg))


#    def load(self, path):
    def test_load(self):
        """test method for load
        arg:
         string

        return:
         bool
        """

        expected = True
        actual = self.arrow.load(self.arrow_path)
        self.assertEqual(expected, actual)

#    def get_distance(position1, position2):
    def test_get_distance(self):
        """test method for get_length
        arg:
       　numpy.array,numpy.array
        return:
         float
        """

        pos1 = self.waypoint.get_np_position("8813")
        pos2 = self.waypoint.get_np_position("8832")

        expected = 19.000144209979041
        actual = self.arrow.get_distance(pos1, pos2)
        self.assertEqual(expected, actual)

#    def get_heading(self, arrow_code, waypoint_id):
    def test_get_heading(self):
        """test method for get_length
        arg:
         string,string
        return:
         float
        """

        expected = 206.97696610768335
        actual = self.arrow.get_heading("10234_8883", "10239")
        self.assertEqual(expected, actual)


#    def get_point_to_arrows(self, point_position, arrow_codes=None):
    def test_get_point_to_arrows(self):
        """test method for get_length
        arg:
         ndarray, string
        return:
         string, string, ndarray, float
        """

        arrow_codes = ["8926_8936", "9044_9054", "9151_9155"]
        pos = self.waypoint.get_np_position("9045")

        expected_arrow = "9044_9054"
        expected_id, expected_pos, expected_distance = \
            self.arrow.get_point_to_arrow(pos, expected_arrow)

        actual_arrow, actual_id, actual_pos, actual_distance = \
            self.arrow.get_point_to_arrows(pos, arrow_codes)

        self.assertEqual(expected_arrow, actual_arrow)
        self.assertEqual(expected_id, actual_id)
        self.assertEqual(expected_pos, actual_pos)
        self.assertEqual(expected_distance, actual_distance)

#    def get_arrow_codes_to_arrows(self, arrow_codes):
    def test_get_arrow_codes_to_arrows(self):
        """test method for get_length
        arg:
         string
        return:
         dict, dict, dict
        """

        expected_arrows = {"9313_9315": self.data["arrows"]["9313_9315"], "9354_9496": self.data["arrows"]["9354_9496"],
                           "9610_9673": self.data["arrows"]["9610_9673"]}

        expected_to_arrows = {"9313_9315": ["9354_9496"], "9354_9496": ["9610_9673"]}

        expected_from_arrows = {"9354_9496": ["9313_9315"], "9610_9673": ["9354_9496"]}

        arrow_codes = ["9313_9315", "9354_9496", "9610_9673"]

        actual_arrows, actual_to_arrows, actual_from_arrows = \
            self.arrow.get_arrow_codes_to_arrows(arrow_codes)

        self.assertEqual(expected_arrows, actual_arrows)
        self.assertEqual(expected_to_arrows, actual_to_arrows)
        self.assertEqual(expected_from_arrows, actual_from_arrows)


#    def get_arrow_codes_from_waypoint_id(self, waypoint_id):
    def test_get_arrow_codes_from_waypoint_id(self):
        """test method for get_length
        arg:
         string
        return:
         list
        """

        waypoint_id = "10033"

        expected_arrow = ["10027_9335"]

        actual_arrows = self.arrow.get_arrow_codes_from_waypoint_id(waypoint_id)

        self.assertEqual(expected_arrow, actual_arrows)


#    def get_arrow_codes_set_from_waypoint_ids(self, waypoint_ids):
    def test_get_arrow_codes_from_waypoint_ids(self):
        """test method for get_length
        arg:
         list
        return:
         list
        """

        waypoint_ids = ["9686", "9431", "9143", "8883"]

        expected_arrow = [['9673_9686', '10471_9686', '9418_9431',
                           '9710_9431', '9134_9143', '9019_9143', '8873_8883', '10234_8883']]

        actual_arrows = self.arrow.get_arrow_codes_set_from_waypoint_ids(waypoint_ids)

        self.assertEqual(expected_arrow, actual_arrows)


#    def split_arrow_code(arrow_code):
    def test_split_arrow_code(self):
        """test method for get_length
        arg:
         string
        return:
         list
        """

        arrow_code = "10027_9335"

        expected_arrow = ["10027", "9335"]

        actual_split_arrows = self.arrow.split_arrow_code(arrow_code)

        self.assertEqual(expected_arrow, actual_split_arrows)


if __name__ == "__main__":
    unittest.main()
