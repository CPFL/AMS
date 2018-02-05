#!/usr/bin/env python
# coding: utf-8

from copy import deepcopy
from ams import Validator, AttrDict


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
        def get_data(*args, **kwargs):
            if isinstance(template, list):
                data = []
                for _element in args[0]:
                    element = AttrDict.set_recursively(_element, deepcopy(attr_template.list[0]))
                    data.append(element)
                if not validator.validate({"list": data}):
                    print(validator.validate_errors())
                    print(data)
                    raise ValueError
            else:
                data = AttrDict.set_recursively(kwargs, deepcopy(attr_template))
                if not validator.validate(data):
                    print(validator.validate_errors())
                    print(data)
                    raise ValueError
            return data

        @staticmethod
        def get_schema():
            return deepcopy(schema)

        @staticmethod
        def check_data(data):
            _data = data
            if isinstance(template, list):
                _data = {"list": data}
            return validator.validate(_data)

        @staticmethod
        def get_errors():
            return validator.validate_errors()

    return Base
