#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_namedtuple_from_dict


TAXI_FLEET = get_namedtuple_from_dict("CONST", {
    "DISPATCHABLE_GEOHASH_DIGIT": 6,
    "TIMEOUT": 30.0
})
