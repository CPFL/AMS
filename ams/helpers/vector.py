#!/usr/bin/env python
# coding: utf-8

import math


class Vector(object):
    @staticmethod
    def get_norm(vector):
        return math.sqrt(sum(map(lambda x: x**2, vector)))

    @staticmethod
    def get_add_vector(vector1, vector2):
        return list(map(lambda e: e[0]+e[1], zip(vector1, vector2)))

    @staticmethod
    def get_sub_vector(vector1, vector2):
        return list(map(lambda e: e[0]-e[1], zip(vector1, vector2)))

    @staticmethod
    def get_mul_vector(vector1, vector2):
        return list(map(lambda e: e[0]*e[1], zip(vector1, vector2)))

    @staticmethod
    def get_div_vector(vector1, vector2):
        return list(map(lambda e: e[0]/e[1], zip(vector1, vector2)))

    @staticmethod
    def get_dot(vector1, vector2):
        return sum(Vector.get_mul_vector(vector1, vector2))
