#!/usr/bin/env python
# coding: utf-8

from ams.structures import SelectiveRoute as Structure

from pprint import PrettyPrinter
pp = PrettyPrinter(indent=2).pprint


class SelectiveRoute(object):

    def __init__(self):
        self.__route

    def set_route(self, route):
        self._route = route

    @staticmethod
    def new_selective_route(main_route, sub_routes):
        return Structure.new_data(
            main_route=main_route,
            sub_routes=sub_routes,
        )

    validate_selective_route = Structure.validate_data
    get_errors = Structure.get_errors

    @staticmethod
    def get_arrow_codes(selective_route):
        arrow_codes = selective_route.main_route.arrow_codes
        for sub_route in selective_route.sub_routes:
            arrow_codes.extend(sub_route.arrow_codes)
        return arrow_codes
