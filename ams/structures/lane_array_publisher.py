#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_namedtuple_from_dict


LANE_ARRAY_PUBLISHER = get_namedtuple_from_dict("CONST", {
    "NODE_NAME": "LaneArrayPublisher",
    "ROSNODE": "ams_lane_array_publisher",
    "ROSTOPIC": "/based/lane_waypoints_array"
})
