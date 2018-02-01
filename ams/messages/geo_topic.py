#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_base_class

geo_topic = {
    "node": "EventLoop",
    "id": "id"
}

geo_topic_schema = {
    "node": {
        "type": "string",
        "required": True,
        "nullable": False,
    },
    "id": {
        "type": "string",
        "required": True,
        "nullable": False,
    }
}


class GeoTopic(get_base_class(geo_topic, geo_topic_schema)):
    pass
