#!/usr/bin/env bash
python test/test_arrow.py --arrow_path test/res/arrow.json
python test/res/test_waypoint.py --waypoint_path test/res/waypoint.json
python test/res/test_intersection.py --intersection_path test/res/intersection.json
python test/res/test_converter.py
python test/res/test_map_match.py
