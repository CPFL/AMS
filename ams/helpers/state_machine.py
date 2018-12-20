#!/usr/bin/env python
# coding: utf-8

import json
from ams.structures import StateMachineResource


class StateMachine(object):

    @classmethod
    def load_resource(cls, path):
        with open(path, "r") as f:
            state_machine_resource = StateMachineResource.new_data(**json.load(f))
        return state_machine_resource

    @classmethod
    def create_data(cls, resource):
        return {
            "resource": resource,
            "state": resource["initial_state"],
            "callbacks": {},
            "variables": resource["variables"]
        }

    @classmethod
    def attach(cls, data, callbacks, variables=None):
        for callback in callbacks:
            data["callbacks"][callback.__name__] = callback
        if variables is not None:
            data["variables"].update(variables)

    @classmethod
    def reset_state(cls, data, state):
        data["state"] = state

    @classmethod
    def update_state(cls, data, event=None):
        transitions = list(filter(
            lambda x: x["event"] == event and x["from"] == data["state"],
            data["resource"]["transitions"]
        ))
        for transition in transitions:

            if transition["hooks"] is not None:
                for hook in transition["hooks"]:
                    data["callbacks"][hook["function"]](*list(map(
                        lambda x: x if any(map(lambda t: isinstance(x, t), (int, float))) else data["variables"][x],
                        hook["args"])))

            if transition["conditions"] is None:
                conditions_result = True
            else:
                conditions_result = all(map(
                    lambda x: not x[1] if x[0] else x[1],
                    map(
                        lambda y: (
                            y["not"] if "not" in y else False,
                            data["callbacks"][y["function"]](*list(map(
                                lambda z:
                                    z if any(map(lambda t: isinstance(z, t), (int, float))) else data["variables"][z],
                                y["args"])))),
                        transition["conditions"])
                ))

            if conditions_result:
                data["state"] = transition["to"]
                return True
        return False

    @classmethod
    def get_state(cls, data):
        return data["state"]
