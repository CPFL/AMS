#!/usr/bin/env python
# coding: utf-8


class AttrDict(dict):
    def __getattr__(self, name):
        if name in self:
            return self[name]
        else:
            raise AttributeError("No such attribute: " + name)

    def __setattr__(self, name, value):
        self[name] = value

    def __delattr__(self, name):
        if name in self:
            del self[name]
        else:
            raise AttributeError("No such attribute: " + name)

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
