#!/usr/bin/env python
# coding: utf-8

import csv
import json
import sys
import math
from copy import deepcopy
from argparse import ArgumentParser

from ams.structures import ARROW


def reconnect_arrows(input_arrow_json_path, output_arrow_json_path):
    with open(input_arrow_json_path, "r") as f:
        arrows = json.load(f)["arrows"]

    to_arrows = {}
    from_arrows = {}
    for arrow_code in arrows:
        waypoint_id1, waypoint_id2 = arrow_code.split(ARROW.DELIMITER)
        arrow_codes = list(arrows.keys())
        to_arrows[arrow_code] = list(filter(
            lambda x: waypoint_id2 == x.split(ARROW.DELIMITER)[0] and waypoint_id1 != x.split(ARROW.DELIMITER)[1],
            arrow_codes))
        from_arrows[arrow_code] = list(filter(
            lambda x: waypoint_id1 == x.split(ARROW.DELIMITER)[1] and waypoint_id2 != x.split(ARROW.DELIMITER)[0],
            arrow_codes))

    output_arrows = {
        "arrows": arrows,
        "toArrows": to_arrows,
        "fromArrows": from_arrows
    }

    with open(output_arrow_json_path, "w") as f:
        json.dump(output_arrows, f, indent=" ")


if __name__ == '__main__':

    parser = ArgumentParser()
    parser.add_argument(
        "-IAJP", "--input_arrow_json_path", type=str, required=True, help="input arrow.json file path")
    parser.add_argument(
        "-OAJP", "--output_arrow_json_path", type=str, required=True, help="output arrow.json file path")
    args = parser.parse_args()

    output_arrow_json_path = args.output_arrow_json_path
    if args.output_arrow_json_path is None:
        output_arrow_json_path = args.to_arrow_json_path

    reconnect_arrows(args.input_arrow_json_path, output_arrow_json_path)
