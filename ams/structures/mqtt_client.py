#!/usr/bin/env python
# coding: utf-8

import importlib

from ams import get_namedtuple_from_dict

base_clients = {
    "PAHO_MQTT": {
        "MODULE_NAME": "paho.mqtt.client",
        "DEFAULT_PROTOCOL": None
    },
    "AWS_IOT_SDK": {
        "MODULE_NAME": "AWSIoTPythonSDK.MQTTLib",
        "DEFAULT_PROTOCOL": None
    },
    "AWS_IOT_BOTO3": {
        "MODULE_NAME": "boto3",
        "CLIENT_TYPE": "iot-data"
    }
}

try:
    base_clients["PAHO_MQTT"]["DEFAULT_PROTOCOL"] = \
        importlib.import_module("paho.mqtt.client").MQTTv311
except ImportError:
    ImportError("paho.mqtt.client")

try:
    base_clients["AWS_IOT_SDK"]["DEFAULT_PROTOCOL"] = \
        importlib.import_module("AWSIoTPythonSDK.MQTTLib").MQTTv3_1_1
except ImportError:
    ImportError("paho.mqtt.client")

MQTT_CLIENT = get_namedtuple_from_dict("CONST", {
    "BASE_CLIENTS": base_clients
})
