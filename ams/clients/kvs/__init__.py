#!/usr/bin/env python
# coding: utf-8

from ams.structures import CLIENT


def get_client_class(base_kvs_client_module):

    if base_kvs_client_module.__name__ == CLIENT.KVS.BASE_CLIENTS.MULTIPROCESSING.MODULE_NAME:
        from ams.clients.kvs.manager import KVSClient

    elif base_kvs_client_module.__name__ == CLIENT.KVS.BASE_CLIENTS.REDIS.MODULE_NAME:
        from ams.clients.kvs.redys import KVSClient

    else:
        raise AttributeError("Unknown module {}".format(base_kvs_client_module.__name__))

    return KVSClient
