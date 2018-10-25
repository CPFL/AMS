#!/usr/bin/env python
# coding: utf-8

from argparse import ArgumentParser
from subprocess import check_output
import json


def make_maps(commands):
    for command in commands:
        check_output(command.split(" "))


if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument("-CJP", "--map_maker_commands_json_path", type=str, required=True, help="map_maker_commands.json file path")
    args = parser.parse_args()

    print("make maps")
    with open(args.map_maker_commands_json_path, "r") as f:
        commands = json.load(f)
    make_maps(commands)
