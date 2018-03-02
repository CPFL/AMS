#!/usr/bin/env python
# coding: utf-8

import re


class Converter(object):

    @staticmethod
    def camel_case_to_snake_case(camel_case):
        s1 = re.sub("(.)([A-Z][a-z]+)", r"\1_\2", camel_case)
        return re.sub("([a-z0-9])([A-Z])", r"\1_\2", s1).lower()
