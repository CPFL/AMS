#!/usr/bin/env python
# coding: utf-8

from autoware_msgs.msg import LaneArray

from ams.nodes.ros_bridge import CONST, Helper


class Publisher(object):

    CONST = CONST
    Helper = Helper

    @classmethod
    def publish_lane_array(cls, clients, lane_array):
        clients["ros"].publish(cls.CONST.TOPIC.LANE_ARRAY, lane_array, LaneArray)
        return True
