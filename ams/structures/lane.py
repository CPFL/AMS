#!/usr/bin/env python
# coding: utf-8

from ams import get_structure_superclass, get_namedtuple_from_dict


LANE = get_namedtuple_from_dict("CONST", {
    "DELIMITER": "_"
})

template = {
    "waypoint_ids": ["0", "1"],
    "length": 0
}

'''
lane_code = "[start_waypoint_id]_[goal_waypoint_id]"
lanes = {
    lane_code: lane,
}

lane_codes = [lane_code]
from_lane_code = lane_code
to_lane_codes = lane_codes
to_lanes = {
    from_lane_code: to_lane_codes
}

to_lane_code = lane_code
from_lane_codes = lane_codes
from_lanes = {
    to_lane_code: from_lane_codes
}
'''

schema = {
    "waypoint_ids": {
        "type": "list",
        "required": True,
        "nullable": False,
        "minlength": 2,
    },
    "length": {
        "type": "number",
        "required": True,
        "nullable": False,
    }
}


class Lane(get_structure_superclass(template, schema)):
    pass
