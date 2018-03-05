#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_namedtuple_from_dict


CURRENT_POSE_SUBSCRIBER = get_namedtuple_from_dict("CONST", {
    "NODE_NAME": "CurrentPoseSubscriber",
    "ROSNODE": "ams_current_pose_subscriber",
    "ROSTOPIC": "/current_pose",
    "TOPIC_CATEGORIES": ["current_pose"],
})
