#!/usr/bin/env python
# coding: utf-8

import importlib

from ams import get_namedtuple_from_dict

kvs = {
    "KEY_PATTERN_DELIMITER": "/",
    "BASE_CLIENTS": {
        "MULTIPROCESSING": {
            "MODULE_NAME": "multiprocessing"
        },
        "REDIS": {
            "MODULE_NAME": "redis",
        }
    },
    "TIMESTAMP_STRING_COEFFICIENT": 1000.0
}

pubsub = {
    "BASE_CLIENTS": {
        "PAHO_MQTT": {
            "MODULE_NAME": "paho.mqtt.client",
            "DEFAULT_PROTOCOL": None
        },
        "AWS_IOT_SDK": {
            "MODULE_NAME": "AWSIoTPythonSDK.MQTTLib",
            "DEFAULT_PROTOCOL": None,
            "DEFAULT_DRAINING_FREQUENCY": 20.0
        },
        "AWS_IOT_BOTO3": {
            "MODULE_NAME": "boto3",
            "CLIENT_TYPE": "iot-data"
        },
        "ROS_NODE":{
            "MODULE_NAME": "rospy"
        }
    }
}

try:
    pubsub["BASE_CLIENTS"]["PAHO_MQTT"]["DEFAULT_PROTOCOL"] = \
        importlib.import_module("paho.mqtt.client").MQTTv311
except ImportError:
    pass

try:
    pubsub["BASE_CLIENTS"]["AWS_IOT_SDK"]["DEFAULT_PROTOCOL"] = \
        importlib.import_module("AWSIoTPythonSDK.MQTTLib").MQTTv3_1_1
except ImportError:
    pass


CLIENT = get_namedtuple_from_dict("CONST", {
    "KVS": kvs,
    "PUBSUB": pubsub
})
