#!/usr/bin/env python
# coding: utf-8

from ams.structures import Position as Structure


class Position(object):

    @staticmethod
    def new_position(x, y, z):
        return Structure.new_data(x=x, y=y, z=z)

    validate_position = Structure.validate_data

    @staticmethod
    def get_vector(position):
        return list(map(lambda key: position[key], ("x", "y", "z")))
