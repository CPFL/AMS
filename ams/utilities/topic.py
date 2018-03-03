#!/usr/bin/env python
# coding: utf-8

import json
from ams import AttrDict, Converter, Target


class Topic(object):

    def __init__(self):
        self.domain = "ams"
        self.from_target = None
        self.to_target = None
        self.categories = []
        self.fix_path = None

    def set_fix_path(self, fix_path):
        self.fix_path = fix_path

    def set_targets(self, from_target, to_target=None):
        self.from_target = from_target
        self.to_target = to_target

    def set_categories(self, categories):
        self.categories = categories

    @staticmethod
    def get_topics(topic_node_classes):
        return AttrDict(map(
            lambda node_class: (node_class, Topic()),
            topic_node_classes
        ))

    @staticmethod
    def get_target_path_part(target, use_wild_card=False):
        if target is not None:
            return "/" + "/".join([
                Converter.camel_case_to_snake_case(target.group),
                (target.id if target.id is not None else ("+" if use_wild_card else "*"))
            ])
        else:
            if use_wild_card:
                return "/+/+"
            else:
                return "/*/*"

    def get_path(self, use_wild_card=False):
        if self.fix_path is not None:
            return self.fix_path

        path = "/" + self.domain

        path += Topic.get_target_path_part(self.from_target, use_wild_card)
        path += Topic.get_target_path_part(self.to_target, use_wild_card)

        path += "/" + "/".join(
            self.categories if self.categories is not None else "#"
        )
        return path

    def get_response_path(self, request_path):
        path = "/" + self.domain

        path += Topic.get_target_path_part(self.from_target)
        path += Topic.get_target_path_part(Target.new_target(*list(reversed(request_path.split("/")[2:4]))))

        path += "/" + "/".join(
            self.categories if self.categories is not None else "#"
        )
        return path

    def set_message(self, message):
        self.message = message

    @staticmethod
    def serialize(message):
        return json.dumps(message)

    def unserialize(self, payload):
        json_message = json.loads(payload)
        if isinstance(json_message, dict):
            return self.message.new_data(**json.loads(payload))
        elif isinstance(json_message, list):
            return self.message.new_data(json.loads(payload))

    @staticmethod
    def get_from_id(path):
        return path.split("/")[3]

    @staticmethod
    def is_path_matched(subscriber_path, incoming_path):
        split_subscriber_path = subscriber_path.split("/")
        split_incoming_path = incoming_path.split("/")
        if len(split_subscriber_path) != len(split_incoming_path):
            return False
        return all(map(lambda x: x[0] in ["+", "#"] or x[1] in ["*"] or x[0] == x[1], zip(
            split_subscriber_path, split_incoming_path
        )))
