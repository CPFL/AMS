#!/usr/bin/env python
# coding: utf-8

import rospy
from autoware_msgs.msg import traffic_light as TrafficLight

from ams import Topic
from ams.nodes import EventLoop, Autoware
from ams.messages import LightColor


class LightColorManagedPublisher(EventLoop):
    def __init__(self, name):
        super(LightColorManagedPublisher, self).__init__()

        self.topicPubLightColor = Topic()
        self.topicPubLightColor.set_id(name)
        self.topicPubLightColor.set_root(Autoware.CONST.TOPIC.PUBLISH)

        self.add_on_message_function(self.publish_to_ros)
        self.set_subscriber(self.topicPubLightColor.private+Autoware.CONST.TOPIC.TRAFFIC_LIGHT)

        rospy.init_node(Autoware.CONST.ROSNODE.AMS_TRAFFIC_LIGHT_PUBLISHER, anonymous=True)
        self.__ROSPublisher = rospy.Publisher(Autoware.CONST.ROSTOPIC.LIGHT_COLOR, TrafficLight)

    def publish_to_ros(self, _client, _userdata, topic, payload):
        if topic == self.topicPubLightColor.private+Autoware.CONST.TOPIC.TRAFFIC_LIGHT:
            light_color = LightColor.new_data(**self.topicPubLightColor.unserialize(payload))
            traffic_light = TrafficLight()
            traffic_light.traffic_light = light_color.traffic_light
            self.__ROSPublisher.publish(traffic_light)
