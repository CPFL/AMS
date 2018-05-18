#!/usr/bin/env python
# coding: utf-8

from ams import get_structure_superclass, get_namedtuple_from_dict


ARROW = get_namedtuple_from_dict("CONST", {
    "DELIMITER": "_"
})

template = {
    "waypoint_ids": ["0", "1"],
    "length": 0
}

'''
arrow_code = "[start_waypoint_id]_[goal_waypoint_id]"
arrows = {
    arrow_code: arrow,
}

arrow_codes = [arrow_code]
from_arrow_code = arrow_code
to_arrow_codes = arrow_codes
to_arrows = {
    from_arrow_code: to_arrow_codes
}

to_arrow_code = arrow_code
from_arrow_codes = arrow_codes
from_arrows = {
    to_arrow_code: from_arrow_codes
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


class Arrow(get_structure_superclass(template, schema)):
    pass
