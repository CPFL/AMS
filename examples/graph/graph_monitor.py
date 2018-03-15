#!/usr/bin/env python
# coding: utf-8

import paho.mqtt.client as mqtt
import json
from graphviz import Digraph

from config.env import env


class GraphMonitor(object):
    def __init__(self, host=env["MQTT_BROKER_HOST"], port=int(env["MQTT_BROKER_PORT"])):
        self.__relations = {}

        self.__client = mqtt.Client(protocol=mqtt.MQTTv311)
        self.__client.on_message = self.__on_message
        self.__client.connect(host, port=port, keepalive=60)

    def __del__(self):
        self.__client.disconnect()

    def __on_message(self, _client, _userdata, message_data):
        from_node_class, from_id, to_node_class, to_id = message_data.topic.split("/")[2:6]

        mode = "add"
        if from_node_class == "event_loop":
            payload = json.loads(message_data.payload.decode("utf-8"))
            if payload["event"] == "will":
                mode = "remove"
            else:
                mode = None
        elif from_node_class == "graph_monitor":
            mode = None

        if mode is not None and self.update_relations(from_node_class, from_id, to_node_class, to_id, mode):
            graph_dot, class_graph_dot = self.get_dots()
            if graph_dot is not None:
                with open("./graph.dot", "w") as f:
                    f.write(graph_dot)
            if class_graph_dot is not None:
                with open("./class_graph.dot", "w") as f:
                    f.write(class_graph_dot)

    def update_relations(self, from_class, from_id, to_class, to_id, mode):
        if mode == "add":
            from_id_to_class_id_relations = self.__relations.get(from_class, {})
            to_class_id_relations = from_id_to_class_id_relations.get(from_id, {})
            to_ids = to_class_id_relations.get(to_class, [])
            if to_id not in to_ids:
                to_ids.append(to_id)
                to_class_id_relations[to_class] = to_ids
                from_id_to_class_id_relations[from_id] = to_class_id_relations
                self.__relations[from_class] = from_id_to_class_id_relations
                return True
        elif mode == "remove":
            for from_class, from_id_to_class_id_relations in self.__relations.items():
                if from_id in from_id_to_class_id_relations:
                    self.__relations[from_class].pop(from_id)
            return True
        return False

    def get_dots(self):
        f = Digraph('finite_state_machine')
        f.attr(rankdir='LR', size='8,5')
        f.attr('node', shape='circle')
        edges = {}
        for from_class, from_id_to_class_id_relations in self.__relations.items():
            for from_id, to_class_id_relations in from_id_to_class_id_relations.items():
                for to_class, to_ids in to_class_id_relations.items():
                    edges[(from_class, to_class, from_class+" -> "+to_class)] = None
        for edge in set(list(edges.keys())):
            f.edge(edge[0], edge[1], label=edge[2])
        class_graph_dot = f.source

        f = Digraph('finite_state_machine')
        f.attr(rankdir='LR', size='8,5')
        f.attr('node', shape='circle')
        edges = {}
        for from_class, from_id_to_class_id_relations in self.__relations.items():
            for from_id, to_class_id_relations in from_id_to_class_id_relations.items():
                for to_class, to_ids in to_class_id_relations.items():
                    if from_class == "traffic_signal":
                        edges[(from_class, to_class, from_class + " -> " + to_class)] = None
                    else:
                        for to_id in to_ids:
                            edges[(from_class, to_class, from_id + " -> " + to_id)] = None
        for edge in set(list(edges.keys())):
            f.edge(edge[0], edge[1], label=edge[2])
        graph_dot = f.source

        return graph_dot, class_graph_dot

    def subscribe(self, topic):
        self.__client.subscribe(topic)

    def start(self):
        self.__client.loop_forever()


if __name__ == '__main__':
    graph_monitor = GraphMonitor()
    graph_monitor.subscribe("/ams/#")
    graph_monitor.start()
