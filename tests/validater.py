#!/usr/bin/env python
# coding: utf-8

import unittest
from queue import Queue
import threading

from ams import get_structure_superclass
import logging

# initialize validator
test_schema = {
    "test": {
        "type": "boolean",
        "required": True,
        "nullable": False
    }
}

test_template = {
    "test": True
}


class Validator(get_structure_superclass(schema=test_schema, template=test_template)):
    pass


class Test(unittest.TestCase):

    def __init__(self, method_name):
        super(Test, self).__init__(method_name)

    def setUp(self):
        logging.disable(logging.FATAL)

    def tearDown(self):
        logging.disable(logging.NOTSET)

    def test_validate_data_thread_safe(self):
        test_validator = Validator()
        error_results = Queue()

        def thread_test_1(validator, q):
            count = 0
            num = 500
            data = {"test": True}
            while count < num:
                if not validator.validate_data(data):
                    q.put("test1 missing")
                    break
                count += 1

            if count == num:
                q.put(None)

        def thread_test_2(validator, q):
            count = 0
            num = 500
            data = {}
            while count < num:
                if validator.validate_data(data):
                    q.put("test2 missing")
                    break
                count += 1

            if count == num:
                q.put(None)

        thread_test_1 = threading.Thread(target=thread_test_1, args=(test_validator, error_results))
        thread_test_2 = threading.Thread(target=thread_test_2, args=(test_validator, error_results))

        thread_test_1.start()
        thread_test_2.start()

        thread_test_1.join()
        thread_test_2.join()

        results = []
        while not error_results.empty():
            results.append(error_results.get())

        self.assertEqual(len(results), 2)
        self.assertEqual(results, [None, None], "expected results is [None, None]. test result is %s" % results)
