#!/usr/bin/env python
# coding: utf-8

import rospy
from multiprocessing import Manager
from threading import Thread

from ams import AttrDict, logger
from ams.structures import CLIENT


def subscribe_loop(subscribers_lock, subscriber):
    rospy_rate = rospy.Rate(hz=subscriber["rate"], reset=False)
    while not rospy.is_shutdown():
        try:
            message_data = rospy.wait_for_message(topic=subscriber["topic"], topic_type=subscriber["structure"], timeout=None)
            subscribers_lock.acquire()
            ret = subscriber["callback"](rospy, subscriber["user_data"], subscriber["topic"], message_data)
            subscribers_lock.release()
            rospy_rate.sleep()
        except rospy.ROSException:
            pass
        except rospy.ROSInterruptException:
            break
        except IOError:
            pass
        except Exception as e:
            logger.error("e: {}".format(e))


class PubSubClient(object):
    CONST = CLIENT.PUBSUB.BASE_CLIENTS.ROS_NODE

    def __init__(self):
        self.__client = rospy
        self.__manager = Manager()
        self.__subscribers_lock = self.__manager.Lock()
        self.__subscribers = {}
        self.__publishers = {}

    def __delete__(self):
        self.disconnect()
        self.__manager.shutdown()

    def connect(self):
        self.__client.init_node(name="ros_ams_node", anonymous=True)
        for subscriber in self.__subscribers.values():
            if subscriber["thread"] is not None:
                subscriber["thread"].start()

    def disconnect(self):
        for subscriber in self.__subscribers.values():
            if subscriber["thread"] is not None:
                subscriber["thread"].terminate()
        self.__client.signal_shutdown("ros node is disconnected")

    def publish(self, topic, message, structure, qos=0, retain=False, wait=False):
        """
        publish the message to topic in ros node.

        :param topic: topic name to publish
        :param message: message to publish
        :param structure: message format type
        :param qos: quality of service(unused)
        :param retain: is retained message or not(unused)
        :param wait: is wait until publish or not(unused)
        :return: None
        """

        if topic not in self.__publishers:
            queue_size = None if wait else 2
            self.__publishers[topic] = self.__client.Publisher(topic, structure, queue_size=queue_size)
        attr_dict = AttrDict.set_recursively(message)
        self.__publishers[topic].publish(structure(**attr_dict))

    def subscribe(self, topic, callback, qos=0, user_data=None, structure=None, rate=None):
        """
        subscribe the message to topic in ros node.

        :param topic: topic name to subscribe
        :param callback: callback to subscribe
        :param qos: qos to subscribe
        :param user_data: user_data to subscribe
        :param structure: message format type
        :param rate: is rate for callback [Hz] or None(event driven)
        :return: None
        """
        def on_message(message_data):
            self.__subscribers_lock.acquire()
            callback(self.__client, user_data, topic, message_data)
            self.__subscribers_lock.release()

        self.__subscribers[topic] = {
            "topic": topic,
            "callback": callback,
            "qos": qos,
            "user_data": user_data,
            "structure": structure,
            "rate": rate,
            "thread": None
        }

        if self.__client is not None:
            if rate is None:
                self.__client.Subscriber(
                    topic,
                    structure,
                    on_message
                )
            else:
                self.__subscribers[topic]["thread"] = Thread(
                    target=subscribe_loop, args=(self.__subscribers_lock, self.__subscribers[topic]))

    def unsubscribe(self):
        pass

    def loop(self, sleep_time):
        while not rospy.is_shutdown():
            sleep(sleep_time)

