#!/usr/bin/env python
# coding: utf-8

import csv
from ams.structures import Cycle


class TrafficSignal(object):

    @classmethod
    def load(cls, path_to_lane_signal_cycle_relation_csv, path_to_signal_cycle_csv, path_to_signal_color_pattern_csv):
        with open(path_to_lane_signal_cycle_relation_csv, "r") as f:
            lane_signal_cycle_relations = dict(map(lambda x: (x["lane_id"], x["signal_cycle_id"]), csv.DictReader(f)))
        with open(path_to_signal_cycle_csv, "r") as f:
            signal_cycles = dict(map(lambda x: (x["signal_cycle_id"], x), csv.DictReader(f)))
        with open(path_to_signal_color_pattern_csv, "r") as f:
            signal_color_patterns = dict(map(lambda x: (x["signal_color_pattern_id"], x), csv.DictReader(f)))

        traffic_signals = {}
        for lane_id, signal_cycle_id in lane_signal_cycle_relations.items():
            signal_color_pattern_id = signal_cycles[signal_cycle_id]["signal_color_pattern_id"]
            signal_colors = list(signal_color_patterns[signal_color_pattern_id]["signal_color_pattern"].split(":"))
            durations = list(map(
                float, signal_cycles[signal_cycle_id]["durations"].split(":")
            ))
            traffic_signals[lane_id] = Cycle.new_data(**{
                "time": float(signal_cycles[signal_cycle_id]["base_time"]),
                "period": sum(durations),
                "phases": list(map(lambda x: {"state": x[0], "duration": x[1]}, zip(signal_colors, durations)))
            })
        return traffic_signals
