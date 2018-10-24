#!/usr/bin/env python
# coding: utf-8

import json
from argparse import ArgumentParser


def merge_arrows(input_arrow_json_path, output_arrow_json_path, arrow_code_csv):
    with open(input_arrow_json_path, "r") as f:
        input_arrows = json.load(f)

    arrow_codes = arrow_code_csv.split(",")

    merged_arrow_code = "_".join([arrow_codes[0].split("_")[0], arrow_codes[-1].split("_")[1]])
    merged_waypoint_ids = []
    for waypoint_ids in map(lambda x: input_arrows["arrows"][x]["waypointIDs"], arrow_codes):
        if 0 < len(merged_waypoint_ids):
            merged_waypoint_ids.pop()
        merged_waypoint_ids.extend(waypoint_ids)
    input_arrows["arrows"][merged_arrow_code] = {
        "arrowCode": merged_arrow_code,
        "waypointIDs": merged_waypoint_ids
    }
    with open(output_arrow_json_path, "w") as f:
        json.dump(input_arrows, f, indent=" ")


if __name__ == '__main__':

    parser = ArgumentParser()
    parser.add_argument("-IAJP", "--input_arrow_json_path", type=str, required=True, help="input arrow.json file path")
    parser.add_argument("-OAJP", "--output_arrow_json_path", type=str, default=None, help="output arrow.json file path")
    parser.add_argument("-ACC", "--arrow_code_csv", type=str, default=None, help="arrow_codes as csv")
    args = parser.parse_args()

    output_arrow_json_path = args.output_arrow_json_path
    if args.output_arrow_json_path is None:
        output_arrow_json_path = args.input_arrow_json_path

    merge_arrows(
        args.input_arrow_json_path, output_arrow_json_path, args.arrow_code_csv
    )
