#!/usr/bin/env python
# coding: utf-8

import rospy
from autoware_msgs.msg import traffic_light as TrafficLight

from ams import Topic
from ams.nodes import EventLoop, Autoware
from ams.messages import autoware_message


class LightColorManagedPublisher(EventLoop):
    def __init__(self, name):
        super(LightColorManagedPublisher, self).__init__()

        self.autowarePublishTopic = Topic()
        self.autowarePublishTopic.set_id(name)
        self.autowarePublishTopic.set_root(Autoware.TOPIC.PUBLISH)
        self.autowarePublishTopic.set_message(autoware_message)

        self.add_on_message_function(self.publish_to_ros)
        self.set_subscriber(self.autowarePublishTopic.private+"/traffic_light")

        rospy.init_node("ams_traffic_light_publisher", anonymous=True)
        self.__ROSPublisher = rospy.Publisher(Autoware.ROSTOPIC.TRAFFIC_LIGHT, TrafficLight)

    def publish_to_ros(self, client, userdata, topic, payload):
        if topic == self.autowarePublishTopic.private+"/traffic_light":
            traffic_light_message = self.autowarePublishTopic.unserialize(payload)
            trafficLight = TrafficLight()
            trafficLight.traffic_light = traffic_light_message["traffic_light"]
            self.__ROSPublisher.publish(trafficLight)

