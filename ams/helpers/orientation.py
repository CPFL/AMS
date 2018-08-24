#!/usr/bin/env python
# coding: utf-8

import math
from copy import deepcopy

from ams.structures import Orientation as Structure
from ams.structures import Quaternion, Rpy
from ams.helpers import Rpy as RpyHelper


class Orientation(object):

    @staticmethod
    def new_orientation(quaternion, rpy):
        return Structure.new_data(quaternion=Quaternion.new_data(**quaternion), rpy=Rpy.new_data(**rpy))

    validate_orientation = Structure.validate_data
    get_errors = Structure.get_errors

    @staticmethod
    def get_inverted(orientation):
        inverted_rpy = RpyHelper.get_inverted(orientation.rpy)

        inverted_orientation = Orientation.new_orientation(
            dict(zip(
                ["w", "x", "y", "z"],
                RpyHelper.to_quaternion([0, 0, 1], inverted_rpy.yaw))),
            inverted_rpy
        )
        return inverted_orientation
