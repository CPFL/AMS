#!/usr/bin/env python
# coding: utf-8

import math
from ams.helpers import Vector
from ams.structures import RPY
from ams.structures import Rpy as Structure


class Rpy(object):

    Structure = Structure

    @staticmethod
    def new_rpy(roll, pitch, yaw):
        return Structure.new_data(roll=roll, pitch=pitch, yaw=yaw)

    validate_rpy = Structure.validate_data
    get_errors = Structure.get_errors

    @staticmethod
    def get_inverted_radian(radian):
        inverted_radian = radian + math.pi
        if inverted_radian < RPY.MIN:
            inverted_radian += 2.0*math.pi*float(1+math.floor((RPY.MIN - inverted_radian)/(2.0*math.pi)))
        if RPY.MAX < inverted_radian:
            inverted_radian -= 2.0*math.pi*float(1+math.floor((inverted_radian - RPY.MAX)/(2.0*math.pi)))
        return inverted_radian

    @staticmethod
    def get_inverted(rpy):
        return Rpy.new_rpy(
            rpy.roll,
            Rpy.get_inverted_radian(rpy.pitch) if rpy.pitch is not None else None,
            Rpy.get_inverted_radian(rpy.yaw) if rpy.yaw is not None else None,
        )

    @staticmethod
    def to_quaternion(vector, theta, is_normalized=False):
        if not is_normalized:
            vector = Vector.get_div_vector(vector, [math.sqrt(Vector.get_dot(vector, vector))]*len(vector))
        t2 = theta / 2.0
        st2 = math.sin(t2)
        return [math.cos(t2)] + Vector.get_mul_vector(vector, [st2]*len(vector))
