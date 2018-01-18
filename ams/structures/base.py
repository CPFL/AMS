#!/usr/bin/env python
# coding: utf-8

from copy import deepcopy
from ams import Validator


class AttrDict(dict):
    def __getattr__(self, name):
        if name in self:
            return self[name]
        else:
            raise AttributeError("No such attribute: " + name)

    def __setattr__(self, name, value):
        self[name] = value

    def __delattr__(self, name):
        if name in self:
            del self[name]
        else:
            raise AttributeError("No such attribute: " + name)


def get_base_class(template, schema):

    attr_template = AttrDict()
    for key, value in template.items():
        attr_template[key] = value

    class Base(Validator):
        def __init__(self):
            self.__template = attr_template
            super().__init__(schema)

        @staticmethod
        def get_template():
            return deepcopy(attr_template)

        @staticmethod
        def get_data(**kwargs):
            data = deepcopy(attr_template)
            for k, v in kwargs.items():
                data[k] = v
            return data

        @staticmethod
        def get_schema():
            return deepcopy(schema)
    return Base
