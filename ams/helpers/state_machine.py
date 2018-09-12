#!/usr/bin/env python
# coding: utf-8

import json
from ams.structures import StateMachineResource


class StateMachine(object):

    @classmethod
    def load(cls, path):
        with open(path, "r") as f:
            state_machine_resource = StateMachineResource.new_data(**json.load(f))
        return {
            "resource": state_machine_resource,
            "state": state_machine_resource.initial_state,
            "callbacks": {},
            "variables": state_machine_resource.variables
        }

    @classmethod
    def set_callbacks(cls, data, callbacks):
        for callback in callbacks:
            data["callbacks"][callback.__name__] = callback

    @classmethod
    def update(cls, data, event=None):
        transitions = list(filter(
            lambda x: x["event"] == event and x["from"] == data["state"],
            data["resource"]["transitions"]
        ))
        for transition in transitions:

            if transition["hooks"] is not None:
                for hook in transition["hooks"]:
                    data["callbacks"][hook["function"]](*list(map(lambda x: data["variables"][x], hook["args"])))

            if transition["conditions"] is None:
                conditions_result = True
            else:
                conditions_result = all(map(
                    lambda x: not x[1] if x[0] else x[1],
                    map(
                        lambda y: (
                            y["not"] if "not" in y else False,
                            data["callbacks"][y["function"]](*list(map(lambda z: data["variables"][z], y["args"])))),
                        transition["conditions"])
                ))

            if conditions_result:
                data["state"] = transition["to"]
                return True
        return False

    @classmethod
    def get_state(cls, data):
        return data["state"]
