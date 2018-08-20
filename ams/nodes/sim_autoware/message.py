#!/usr/bin/env python
# coding: utf-8

from ams.nodes.sim_autoware import Structure


class Message(object):
    CurrentPose = Structure.Status.CurrentPose
    ClosestWaypoint = Structure.Status.ClosestWaypoint
    DecisionMakerState = Structure.Status.DecisionMakerState
