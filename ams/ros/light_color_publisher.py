#!/usr/bin/env python
# coding: utf-8

import rospy
from autoware_msgs.msg import traffic_light as AWTrafficLight

from ams import Topic, Target
from ams.nodes import EventLoop
from ams.messages import LightColor
from ams.structures import LIGHT_COLOR_PUBLISHER, AUTOWARE


class LightColorPublisher(EventLoop):

    CONST = LIGHT_COLOR_PUBLISHER

    def __init__(self, _id):
        super(LightColorPublisher, self).__init__(_id)

        self.__topicSubLightColor = Topic()
        self.__topicSubLightColor.set_targets(
            Target.new_target(self.target.id, AUTOWARE.NODE_NAME), self.target
        )
        self.__topicSubLightColor.set_categories(AUTOWARE.TOPIC.CATEGORIES.LIGHT_COLOR)
        self.__topicSubLightColor.set_message(LightColor)
        self.set_subscriber(self.__topicSubLightColor, self.publish_to_ros)

        rospy.init_node(LIGHT_COLOR_PUBLISHER.ROSNODE, anonymous=True)
        self.__ROSPublisher = rospy.Publisher(LIGHT_COLOR_PUBLISHER.ROSTOPIC, AWTrafficLight, queue_size=1)

    def publish_to_ros(self, _client, _userdata, _topic, payload):
        light_color = self.__topicSubLightColor.unserialize(payload)
        aw_traffic_light = AWTrafficLight()
        aw_traffic_light.traffic_light = light_color.traffic_light
        self.__ROSPublisher.publish(aw_traffic_light)
