#!/usr/bin/env python
# coding: utf-8

import json
from argparse import ArgumentParser


def remove_arrow(input_arrow_json_path, output_arrow_json_path, arrow_code):
    with open(input_arrow_json_path, "r") as f:
        input_arrows = json.load(f)

    input_arrows["arrows"].pop(arrow_code)
    with open(output_arrow_json_path, "w") as f:
        json.dump(input_arrows, f, indent=" ")


if __name__ == '__main__':

    parser = ArgumentParser()
    parser.add_argument("-IAJP", "--input_arrow_json_path", type=str, required=True, help="input arrow.json file path")
    parser.add_argument("-OAJP", "--output_arrow_json_path", type=str, default=None, help="output arrow.json file path")
    parser.add_argument("-AC", "--arrow_code", type=str, default=None, help="arrow_code")
    args = parser.parse_args()

    output_arrow_json_path = args.output_arrow_json_path
    if args.output_arrow_json_path is None:
        output_arrow_json_path = args.input_arrow_json_path

    remove_arrow(
        args.input_arrow_json_path, output_arrow_json_path, args.arrow_code
    )
