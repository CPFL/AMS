#!/usr/bin/env python
# coding: utf-8

from copy import deepcopy
from multiprocessing import Manager

from cerberus import Validator as cv

from ams import logger


class Validator(object):

    __manager = Manager()

    def __init__(self, schema):
        self.__schema = schema
        self.validator = cv(schema)
        self.__lock = self.__manager.Lock()

    def schema(self):
        return self.__schema

    def validate(self, v):
        ret = False
        with self.__lock:
            ret = self.validator(v)
            if not ret:
                logger.error(logger.pformat({"errors": self.validator.errors, "data": v}))
        return ret

    def validate_object(self, obj):
        return self.validator(obj.__dict__)

    def validate_errors(self, v):
        errors = None
        with self.__lock:
            if not self.validator(v):
                errors = deepcopy(self.validator.errors)
        return errors

    @staticmethod
    def get_list_element_checker(inner_validator):
        def check_list_element(field, values, error):
            errors = []
            for i, value in enumerate(values):
                msg = inner_validator.validate_errors(value)
                if msg is not None:
                    errors.append({field+"["+str(i)+"]": msg})
            if 0 < len(errors):
                error(field, str(errors))
        return check_list_element
