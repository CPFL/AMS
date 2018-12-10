#!/usr/bin/env python
# coding: utf-8

import unittest
import json

from ams.helpers import StateMachine


class Test(unittest.TestCase):

    def __init__(self, method_name):

        super(Test, self).__init__(method_name)

    def test_update_state(self):
        with open("./tests/res/state_machine/test_update_state.json", "r") as f:
            resources = json.load(f)
        for resource in resources:
            data = StateMachine.create_data(resource["state_machine_resource"])
            value = StateMachine.update_state(data, resource["event"])
            self.assertEqual(resource["expected"], value)
