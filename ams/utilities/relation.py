#!/usr/bin/env python
# coding: utf-8


class Relation(object):

    def __init__(self):
        self.__relations = {}

    def get_keys(self):
        return list(map(dict, self.__relations.keys()))

    def add_relation(self, dict1, dict2, directed=False):
        key1 = tuple(dict1.items())
        key2 = tuple(dict2.items())
        if key1 == key2:
            return False
        value1 = self.__relations.get(key1, [])
        if key2 not in value1:
            value1.append(key2)
        self.__relations[key1] = value1

        if not directed:
            self.add_relation(dict2, dict1, True)
        return True

    def add_relations(self, dict1, dicts, directed=False):
        for dict2 in dicts:
            self.add_relation(dict1, dict2, directed)

    def remove_relation(self, dict1, dict2, directed=False):
        key1 = tuple(dict1.items())
        key2 = tuple(dict2.items())
        value1 = self.__relations.get(key1, [])
        if key2 in value1:
            value1.remove(key2)
        self.__relations[key1] = value1

        if not directed:
            self.remove_relation(dict2, dict1, True)
        return

    def remove_relations(self, dict1, dicts, directed=False):
        for dict2 in dicts:
            self.remove_relation(dict1, dict2, directed)

    def get_related(self, dict1):
        key1 = tuple(dict1.items())
        return list(map(dict, self.__relations[key1]))

    def is_related(self, dict1, dict2):
        key1 = tuple(dict1.items())
        if key1 not in self.__relations:
            return False
        key2 = tuple(dict2.items())
        value1 = self.__relations[key1]
        return key2 in value1
