from time import sleep

import paho.mqtt.client

from ams import get_ams_mqtt_client_class


def sub_and_print(_client, _user_data, topic, message):
    print("on_message: topic={} message_as_dict={}".format(topic, message))


if __name__ == '__main__':
    print("make ams_mqtt_client_class from paho mqtt client.")
    MQTTClient = get_ams_mqtt_client_class(paho.mqtt.client)

    print("new ams_mqtt_client instance.")
    mqtt_client = MQTTClient()

    print("set args of Client constructor.")
    mqtt_client.set_args_of_Client()
    # mqtt_client.set_args_of.tls_set()
    # mqtt_client.set_args_of.tls_insecure_set()
    # mqtt_client.set_args_of.will_set()
    print("set args of connect function")
    mqtt_client.set_args_of_connect(host="localhost")

    # common methods

    print("set subscriber.")
    mqtt_client.subscribe("/test", sub_and_print, user_data="{}")

    print("call all setup functions and connect to mqtt broker.")
    mqtt_client.connect()

    sleep(2)

    print("publish1: topic={} message_as_dict={}".format("/test", {"test": "data"}))
    mqtt_client.publish("/test", {"test": "data"})

    sleep(2)

    print("remove subscriber.")
    mqtt_client.unsubscribe("/test")

    sleep(2)

    print("publish2: topic={} message_as_dict={}".format("/test", {"test": "data"}))
    mqtt_client.publish("/test", {"test": "data"})

    sleep(2)

    print("set subscriber again.")
    mqtt_client.subscribe("/test", sub_and_print, user_data="{}")

    sleep(2)

    print("publish3: topic={} message_as_dict={}".format("/test", {"test": "data"}))
    mqtt_client.publish("/test", {"test": "data"})

    sleep(2)

    print("disconnect.")
    mqtt_client.disconnect()

    print("exit after 2 seconds.")
    sleep(2)
