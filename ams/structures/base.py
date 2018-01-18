#!/usr/bin/env python
# coding: utf-8

from copy import deepcopy
from ams import Validator


def get_base_class(template, schema):
    class Base(Validator):
        def __init__(self):
            self.__template = template
            super().__init__(schema)

        @staticmethod
        def get_template():
            return deepcopy(template)

        @staticmethod
        def get_schema():
            return deepcopy(schema)
    return Base
