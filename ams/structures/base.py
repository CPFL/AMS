#!/usr/bin/env python
# coding: utf-8

from copy import deepcopy
from pprint import pformat
from ams import Validator, AttrDict, logger
from collections import namedtuple


def get_base_class(template, schema):

    _template = template
    if isinstance(template, list):
        _template = {"list": template}
    attr_template = AttrDict()
    for key, value in _template.items():
        attr_template[key] = value

    _schema = schema
    if isinstance(template, list):
        _schema = {"list": schema}
    validator = Validator(_schema)

    class Base(Validator):
        def __init__(self):
            self.__template = attr_template
            super().__init__(_schema)

        @staticmethod
        def get_template():
            if isinstance(template, list):
                return deepcopy(attr_template.list)
            return deepcopy(attr_template)

        @staticmethod
        def new_data(*args, **kwargs):
            if isinstance(template, list):
                data = []
                for _element in args[0]:
                    element = AttrDict.set_recursively(_element, deepcopy(attr_template.list[0]))
                    data.append(element)
                if not validator.validate({"list": data}):
                    logger.error(pformat({"errors": validator.validate_errors(), "data": data}))
                    raise ValueError
            else:
                data = AttrDict.set_recursively(kwargs, deepcopy(attr_template))
                if not validator.validate(data):
                    logger.error(pformat({"errors": validator.validate_errors(), "data": data}))
                    raise ValueError
            return data

        @staticmethod
        def get_schema():
            return deepcopy(schema)

        @staticmethod
        def validate_data(data):
            _data = data
            if isinstance(template, list):
                _data = {"list": data}
            return validator.validate(_data)

        @staticmethod
        def get_errors():
            return validator.validate_errors()

    return Base


def get_namedtuple_from_dict(typename, _dict):
    values = []
    for key, value in _dict.items():
        if isinstance(value, dict):
            values.append(get_namedtuple_from_dict(key, value))
        else:
            values.append(value)
    return namedtuple(typename, list(_dict.keys()))(*values)
