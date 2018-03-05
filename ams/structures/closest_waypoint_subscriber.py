#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_namedtuple_from_dict


CLOSEST_WAYPOINT_SUBSCRIBER = get_namedtuple_from_dict("CONST", {
    "NODE_NAME": "ClosestWaypointSubscriber",
    "ROSNODE": "ams_closest_waypoint_subscriber",
    "ROSTOPIC": "/closest_waypoint",
    "TOPIC_CATEGORIES": ["/closest_waypoint"],
})
