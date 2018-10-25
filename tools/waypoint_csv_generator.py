#!/usr/bin/env python
# coding: utf-8

from argparse import ArgumentParser
from ams.clients import MapsClient


def generate_waypoint_csv(waypoint_json_path, arrow_json_path, waypoint_csv_path, route_code):
    mc = MapsClient()
    mc.load_waypoint_json_file(waypoint_json_path)
    mc.load_arrow_json_file(arrow_json_path)
    arrow_code_waypoint_id_relations = mc.route.generate_arrow_code_waypoint_id_relations(route_code)

    waypoints_for_csv = [["x", "y", "z", "yaw", "velocity", "change_flag"]]
    for arrow_code_waypoint_id_relation in arrow_code_waypoint_id_relations:
        for waypoint_id in arrow_code_waypoint_id_relation["waypoint_ids"]:
            pose = mc.arrow.get_pose(arrow_code_waypoint_id_relation["arrow_code"], waypoint_id)
            velocity = mc.waypoint.get_speed_limit(waypoint_id) * 3.6
            if arrow_code_waypoint_id_relation["direction"] == mc.route.CONST.DELIMITERS.BACKWARD:
                velocity = -velocity
            yaw = mc.arrow.get_yaw(arrow_code_waypoint_id_relation["arrow_code"], waypoint_id)
            waypoints_for_csv.append(list(map(
                str, [pose.position.x, pose.position.y, pose.position.z, yaw, velocity, 0])))

    with open(waypoint_csv_path, "w") as f:
        for waypoint in waypoints_for_csv:
            f.write(",".join(waypoint)+"\n")


if __name__ == '__main__':

    parser = ArgumentParser()
    parser.add_argument("-WJP", "--waypoint_json_path", type=str, required=True, help="waypoint.json file path")
    parser.add_argument("-AJP", "--arrow_json_path", type=str, required=True, help="arrow.json file path")
    parser.add_argument("-WCP", "--waypoint_csv_path", type=str, required=True, help="waypoint.csv file path")
    parser.add_argument("-RC", "--route_code", type=str, required=True, help="route_code")
    args = parser.parse_args()

    generate_waypoint_csv(args.waypoint_json_path, args.arrow_json_path, args.waypoint_csv_path, args.route_code)

