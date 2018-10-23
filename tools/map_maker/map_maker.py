#!/usr/bin/env python
# coding: utf-8

from subprocess import check_output


def make_maps():
    for command in [
        "python waypoint_csv_converter.py -WCP ../../res/entre/6F_track.csv -WJP ../../res/entre/waypoint_track.json -AJP ../../res/entre/arrow_track.json -LCF true -NIC 21,32,41",
        "python waypoint_csv_converter.py -WCP ../../res/entre/6F_round.csv -WJP ../../res/entre/waypoint_round.json -AJP ../../res/entre/arrow_round.json -WCO 84 -NIC 90,97",
        "python map_blender.py -FWJP ../../res/entre/waypoint_round.json -FAJP ../../res/entre/arrow_round.json -TWJP ../../res/entre/waypoint_track.json -TAJP ../../res/entre/arrow_track.json -OWJP ../../res/entre/waypoint.json -OAJP ../../res/entre/arrow.json",
        "python waypoint_csv_converter.py -WCP ../../res/entre/6F_rotary.csv -WJP ../../res/entre/waypoint_rotary.json -AJP ../../res/entre/arrow_rotary.json -WCO 127 -NIC 137",
        "python map_blender.py -FWJP ../../res/entre/waypoint_rotary.json -FAJP ../../res/entre/arrow_rotary.json -TWJP ../../res/entre/waypoint.json -TAJP ../../res/entre/arrow.json",
        "python waypoint_csv_converter.py -WCP ../../res/entre/6F_kturn.csv -WJP ../../res/entre/waypoint_kturn.json -AJP ../../res/entre/arrow_kturn.json -WCO 151",
        "python map_blender.py -FWJP ../../res/entre/waypoint_kturn.json -FAJP ../../res/entre/arrow_kturn.json -TWJP ../../res/entre/waypoint.json -TAJP ../../res/entre/arrow.json",

        "python waypoint_id_replacer.py -IWJP ../../res/entre/waypoint.json -IAJP ../../res/entre/arrow.json -OWJP ../../res/entre/waypoint.json -OAJP ../../res/entre/arrow.json -FWID 127 -TWID 21",
        "python waypoint_id_replacer.py -IWJP ../../res/entre/waypoint.json -IAJP ../../res/entre/arrow.json -OWJP ../../res/entre/waypoint.json -OAJP ../../res/entre/arrow.json -FWID 148 -TWID 41",
        "python waypoint_id_replacer.py -IWJP ../../res/entre/waypoint.json -IAJP ../../res/entre/arrow.json -OWJP ../../res/entre/waypoint.json -OAJP ../../res/entre/arrow.json -FWID 84 -TWID 21",
        "python waypoint_id_replacer.py -IWJP ../../res/entre/waypoint.json -IAJP ../../res/entre/arrow.json -OWJP ../../res/entre/waypoint.json -OAJP ../../res/entre/arrow.json -FWID 105 -TWID 0",
        "python waypoint_id_replacer.py -IWJP ../../res/entre/waypoint.json -IAJP ../../res/entre/arrow.json -OWJP ../../res/entre/waypoint.json -OAJP ../../res/entre/arrow.json -FWID 151 -TWID 90",
        "python waypoint_id_replacer.py -IWJP ../../res/entre/waypoint.json -IAJP ../../res/entre/arrow.json -OWJP ../../res/entre/waypoint.json -OAJP ../../res/entre/arrow.json -FWID 202 -TWID 97",

        "python arrow_appender.py -IAJP ../../res/entre/arrow.json -OAJP ../../res/entre/arrow.json -WIDC 137,32",

        "python arrows_connector.py -IAJP ../../res/entre/arrow.json -OAJP ../../res/entre/arrow.json",
    ]:
        check_output(command.split(" "))


if __name__ == '__main__':
    print("make maps")
    make_maps()
