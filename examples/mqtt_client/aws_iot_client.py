from time import sleep
from uuid import uuid4 as uuid

import AWSIoTPythonSDK.MQTTLib

from ams import get_ams_mqtt_client_class


def sub_and_print(_client, _user_data, topic, message):
    print("on_message: topic={} message_as_dict={}".format(topic, message))


ENDPOINT = "put your aws iot endpoint"
PORT = 8883
CA_FILE_PATH = "path/to/your/CAFile"
KEY_PATH = "path/to/your/KeyFile"
CERTIFICATE_PATH = "path/to/your/CertificateFile"
KEEP_ALIVE_INTERVAL_SECOND = 600  # default 600 sec

if __name__ == '__main__':

    print("make ams_mqtt_client_class from aws iot client.")
    MQTTClient = get_ams_mqtt_client_class(AWSIoTPythonSDK.MQTTLib)

    print("new ams_mqtt_client instance.")
    mqtt_client = MQTTClient()

    print("set args of AWSIoTMQTTClient constructor.")
    mqtt_client.set_args_of_AWSIoTMQTTClient(str(uuid()))
    print("set args of configureEndpoint function.")
    mqtt_client.set_args_of_configureEndpoint(ENDPOINT, PORT)
    print("set args of configureCredentials function.")
    mqtt_client.set_args_of_configureCredentials(
        CAFilePath=CA_FILE_PATH, KeyPath=KEY_PATH, CertificatePath=CERTIFICATE_PATH)
    # mqtt_client.set_args_of_configureLastWill()
    print("set args of connect function.")
    mqtt_client.set_args_of_connect(keepAliveIntervalSecond=KEEP_ALIVE_INTERVAL_SECOND)

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