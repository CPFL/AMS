#!/usr/bin/env python
# coding: utf-8

from cerberus import Validator as cv


class Validator(object):
    def __init__(self, schema):
        self.__schema = schema
        self.validator = cv(schema)

    def schema(self):
        return self.__schema

    def validate(self, v):
        return self.validator(v)

    def validate_object(self, obj):
        return self.validator(obj.__dict__)

    def validate_errors(self):
        return self.validator.errors

    @staticmethod
    def get_list_element_checker(inner_validator):
        def check_list_element(field, values, error):
            errors = []
            for i, value in enumerate(values):
                if not inner_validator.validate(value):
                    msg = inner_validator.validate_errors()
                    errors.append({field+"["+str(i)+"]": msg})
            if 0 < len(errors):
                error(field, str(errors))
        return check_list_element
