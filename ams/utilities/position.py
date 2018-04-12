#!/usr/bin/env python
# coding: utf-8

from numpy import array as np_array
from ams.structures import Position as Structure


class Position(object):

    @staticmethod
    def new_position(x, y, z):
        return Structure.new_data(x=x, y=y, z=z)

    @staticmethod
    def new_position_from_np_position(np_position):
        return Position.new_position(*np_position.data[0:3])

    validate_position = Structure.validate_data
    get_errors = Structure.get_errors

    @staticmethod
    def new_np_position(x, y, z):
        return np_array([x, y, z])
