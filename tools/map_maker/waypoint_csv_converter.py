#!/usr/bin/env python
# coding: utf-8

import csv
import json
import sys
import math
from copy import deepcopy
from argparse import ArgumentParser
from pprint import PrettyPrinter

from pyproj import Proj, transform

from ams import logger

pp = PrettyPrinter(indent=2).pprint
ARROW_DELIMITER = "_"


def translate(x, y):
    # in_proj = Proj(init="EPSG:2449")  # http://blog.godo-tys.jp/2012/11/21/999/
    in_proj = Proj(init="EPSG:2451")  # http://blog.godo-tys.jp/2012/11/21/999/
    out_proj = Proj(init='EPSG:4612')
    return transform(in_proj, out_proj, x, y)


def get_degree_from_dmmss(dmmss):
    degree_string, mmss = dmmss.split(".")
    degree = float(degree_string) + float(mmss[:2]) / 60.0
    if 2 < len(mmss):
        degree += float(".".join([mmss[2:4], mmss[4:]])) / 3600.0
    return degree


def get_waypoints_from_pyproj(waypoints_csv, offset):
    waypoints = []
    for waypoint_index, waypoint_csv in enumerate(waypoints_csv):
        waypoint_id = str(offset+waypoint_index)

        yaw = float(waypoint_csv["yaw"])
        while yaw < 0.0:
            yaw += 2.0*math.pi
        while 2.0*math.pi < yaw:
            yaw -= 2.0*math.pi

        lng, lat = translate(float(waypoint_csv["x"]), float(waypoint_csv["y"]))

        waypoints.append({
            "waypointID": waypoint_id,
            "lat": lat,
            "lng": lng,
            "x": float(waypoint_csv["x"]),
            "y": float(waypoint_csv["y"]),
            "z": float(waypoint_csv["z"]),
            "yaw": yaw,
            "speedLimit": float(waypoint_csv["velocity"])/3.6
            # "speedLimit": 1.1
        })
    return waypoints


def get_waypoints_set(waypoints):
    waypoints_set = []
    prev_i = 0
    for i in range(1, len(waypoints)-1):
        if any([
            waypoints[i]["speedLimit"] < 0.0 <= waypoints[i - 1]["speedLimit"],
            waypoints[i]["speedLimit"] < 0.0 <= waypoints[i + 1]["speedLimit"],
            i == len(waypoints)-2
        ]):
            waypoints_set.append(waypoints[prev_i:i+1])
            if (waypoints[i]["speedLimit"] < 0.0 and 0.0 <= waypoints[i + 1]["speedLimit"]) or \
                    (waypoints[i]["speedLimit"] < 0.0 and i == len(waypoints)-2):
                print(waypoints[i]["speedLimit"], waypoints[i + 1]["speedLimit"])
                waypoints_set[-1].reverse()
            prev_i = i
    return waypoints_set


def get_node_ids(offset, waypoints_set, node_id_csv=None):
    node_ids_int = [offset]
    for waypoints in waypoints_set:
        node_ids_int.append(node_ids_int[-1]+len(waypoints)-1)

    if node_id_csv is not None:
        node_ids_int += list(map(int, node_id_csv.split(",")))

    node_ids_int = list(set(node_ids_int))
    node_ids_int.sort()
    return list(map(str, node_ids_int))


def get_arrows(waypoints_set, node_ids):
    arrows = {}
    for waypoints in waypoints_set:
        prev_i = None
        for i, waypoint in enumerate(waypoints):
            if waypoint["waypointID"] in node_ids:
                if prev_i is not None:
                    print(waypoint["waypointID"], node_ids)
                    arrow_code = waypoints[prev_i]["waypointID"] + "_" + waypoint["waypointID"]
                    waypoint_ids = list(map(lambda x: waypoints[x]["waypointID"], range(prev_i, i+1)))
                    arrows[arrow_code] = {
                        "arrowCode": arrow_code,
                        "waypointIDs": waypoint_ids,
                        "length": 12.3
                    }
                prev_i = i
    return arrows


def abs_waypoint_speed_limit(waypoints):
    for i, waypoint in enumerate(waypoints):
        waypoints[i]["speedLimit"] = abs(waypoint["speedLimit"])


if __name__ == '__main__':

    parser = ArgumentParser()
    parser.add_argument("-WCP", "--waypoints_csv_path", type=str, required=True, help="waypoints.csv file path")
    parser.add_argument("-WJP", "--waypoint_json_path", type=str, required=True, help="waypoint.json file path")
    parser.add_argument("-AJP", "--arrow_json_path", type=str, required=True, help="arrow.json file path")
    parser.add_argument("-WCO", "--waypoints_csv_offset", type=int, default=0, help="waypoints.csv offset")
    parser.add_argument("-NIC", "--node_id_csv", type=str, default=None, help="node id csv")
    parser.add_argument("-LCF", "--loop_close_flag", type=bool, default=False, help="loop close flag.")
    args = parser.parse_args()

    if args.waypoints_csv_path == args.waypoint_json_path:
        print("same i/o paths.")
        exit()

    with open(args.waypoints_csv_path, "r") as f:
        reader = csv.reader(f)
        header = next(reader)
        waypoints_csv = []
        for row in reader:
            waypoints_csv.append(dict(zip(header, deepcopy(row))))

    waypoints = get_waypoints_from_pyproj(waypoints_csv, args.waypoints_csv_offset)

    waypoints_set = get_waypoints_set(waypoints)
    if args.loop_close_flag:
        waypoints_set[-1].append(waypoints_set[0][0])
    logger.info(logger.pformat((list(map(lambda x: (x[0]["waypointID"], x[-1]["waypointID"]), waypoints_set)))))

    node_ids = get_node_ids(args.waypoints_csv_offset, waypoints_set, args.node_id_csv)

    arrows = get_arrows(waypoints_set, node_ids)
    print(list(map(lambda x: (x["arrowCode"], x["waypointIDs"][0], x["waypointIDs"][-1]), arrows.values())))

    to_arrows = {}
    from_arrows = {}
    for arrow_code in arrows:
        waypoint_id1, waypoint_id2 = arrow_code.split(ARROW_DELIMITER)
        arrow_codes = list(arrows.keys())
        to_arrows[arrow_code] = list(filter(
            lambda x: waypoint_id2 == x.split(ARROW_DELIMITER)[0] and waypoint_id1 != x.split(ARROW_DELIMITER)[1],
            arrow_codes))
        from_arrows[arrow_code] = list(filter(
            lambda x: waypoint_id1 == x.split(ARROW_DELIMITER)[1] and waypoint_id2 != x.split(ARROW_DELIMITER)[0],
            arrow_codes))

    abs_waypoint_speed_limit(waypoints)

    waypoint_json = {"waypoints": dict(zip(map(lambda x: x["waypointID"], waypoints), waypoints))}

    with open(args.waypoint_json_path, "w") as f:
        json.dump(waypoint_json, f, indent="  ")

    with open(args.arrow_json_path, "w") as f:
        json.dump({"arrows": arrows, "toArrows": to_arrows, "fromArrows": from_arrows}, f, indent="  ")

    print("last waypoint id:", node_ids[-1])
