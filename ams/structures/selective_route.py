#!/usr/bin/env python
# coding: utf-8

from ams.structures import get_base_class, Route, Routes

template = {
    "main_route": Route.get_template(),
    "sub_routes": Routes.get_template(),
}

schema = {
    "main_route": {
        "schema": Route.get_schema(),
    },
    "sub_routes": Routes.get_schema()
}


class SelectiveRoute(get_base_class(template, schema)):
    pass


selective_routes = [SelectiveRoute.get_template()]

selective_routes_schema = {
    "type": "list",
    "valueschema": {
        "schema": SelectiveRoute.get_schema(),
        "required": True,
        "nullable": False,
    },
    "minlength": 1
}


class SelectiveRoutes(get_base_class(selective_routes, selective_routes_schema)):
    pass
