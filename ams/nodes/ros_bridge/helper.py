#!/usr/bin/env python
# coding: utf-8

from ams.nodes.ros_bridge import CONST


class Helper(object):

    AUTOWARE = CONST

    @classmethod
    def get_lane_array(cls, clients, route_code):
        return clients["maps"].route.get_lane_array(route_code)
