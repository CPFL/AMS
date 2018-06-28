#!/usr/bin/env python
# coding: utf-8

from ams import nodes


if __name__ == '__main__':
    for node in dir(nodes):
        if "type" == eval("type(nodes."+node+").__name__"):
            if "get_state_machine" in eval("dir(nodes."+node+")"):
                model = eval(
                    "nodes." + node +
                    ".get_state_machine(nodes." + node +
                    ").model"
                )
                model.title = node
                state_machine = eval(
                    "model.get_graph().draw('./state_machine_graph/" + node +
                    ".png', prog='dot')"
                )
