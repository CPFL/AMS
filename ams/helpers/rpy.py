#!/usr/bin/env python
# coding: utf-8

import math
from ams.helpers import Vector
from ams.structures import Rpy as Structure


class Rpy(object):

    Structure = Structure

    @staticmethod
    def new_rpy(roll, pitch, yaw):
        return Structure.new_data(roll=roll, pitch=pitch, yaw=yaw)

    validate_rpy = Structure.validate_data
    get_errors = Structure.get_errors

    @staticmethod
    def to_quaternion(vector, theta, is_normalized=False):
        if not is_normalized:
            vector = Vector.get_div_vector(vector, [math.sqrt(Vector.get_dot(vector, vector))]*len(vector))
        t2 = theta / 2.0
        st2 = math.sin(t2)
        return [math.cos(t2)] + Vector.get_mul_vector(vector, [st2]*len(vector))
