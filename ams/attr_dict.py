#!/usr/bin/env python
# coding: utf-8

from ams import logger


class AttrDict(dict):
    def __getattr__(self, name):
        if name in self:
            return self[name]
        else:
            # logger.error("No such attribute: " + str(name))
            raise AttributeError("No such attribute: " + str(name))

    def __setattr__(self, name, value):
        self[name] = AttrDict.set_recursively(value)

    def __delattr__(self, name):
        if name in self:
            del self[name]
        else:
            logger.warning("No such attribute: " + str(name))
            raise AttributeError("No such attribute: " + str(name))

    @staticmethod
    def set_recursively(_obj, attr_dict=None):
        if attr_dict is None:
            attr_dict = AttrDict()
        if isinstance(_obj, dict):
            for k, v in _obj.items():
                if isinstance(v, (dict, list)):
                    attr_dict[k] = AttrDict.set_recursively(v)
                else:
                    attr_dict[k] = v
            return attr_dict
        elif isinstance(_obj, list):
            _list = []
            for v in _obj:
                _list.append(AttrDict.set_recursively(v))
            return _list
        else:
            return _obj

    @staticmethod
    def get_dict(attr_dict):
        if isinstance(attr_dict, dict):
            _dict = {}
            for k, v in attr_dict.items():
                if isinstance(v, (dict, list)):
                    _dict[k] = AttrDict.get_dict(v)
                else:
                    _dict[k] = v
            return _dict
        elif isinstance(attr_dict, list):
            _list = []
            for v in attr_dict:
                _list.append(AttrDict.get_dict(v))
            return _list
