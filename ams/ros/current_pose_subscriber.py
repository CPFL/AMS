#!/usr/bin/env python
# coding: utf-8

from time import time

import rospy
import message_filters
from geometry_msgs.msg import PoseStamped, Point32

from ams import Topic
from ams.nodes import EventLoop, Autoware
from ams.messages import CurrentPose

import pprint
pp = pprint.PrettyPrinter(indent=2)


class CurrentPoseSubscriber(EventLoop):
    def __init__(self, name, period):
        super(CurrentPoseSubscriber, self).__init__()

        self.topicSubCurrentPose = Topic()
        self.topicSubCurrentPose.set_id(name)
        self.topicSubCurrentPose.set_root(Autoware.CONST.TOPIC.SUBSCRIBE)

        self.__name = name
        self.__previous_time = time()
        self.__period = period

        rospy.init_node(Autoware.CONST.ROSNODE.AMS_CURRENT_POSE_SUBSCRIBER, anonymous=True)

        if self.__period < 0.1:
            self.__publish_mqtt = self.publish
            self.set_main_loop(rospy.spin)

            self.__ROSSubscriber = message_filters.Subscriber(Autoware.CONST.ROSTOPIC.CURRENT_POSE, PoseStamped, queue_size=1)
            self.__ROSSubscriber.registerCallback(self.on_message_from_ros)
        else:
            self.set_main_loop(self.__main_loop)

    def on_message_from_ros(self, message_data, _user_data):
        current_time = time()
        if self.__period < current_time - self.__previous_time:
            self.__previous_time += (1+int((current_time - self.__previous_time)/self.__period)) * self.__period

            current_pose = CurrentPose.new_data(
                name=self.__name,
                time=message_data.header.stamp.secs + 0.000000001*message_data.header.stamp.nsecs,
                pose={
                    "position": {
                        "x": message_data.pose.position.x,
                        "y": message_data.pose.position.y,
                        "z": message_data.pose.position.z
                    },
                    "orientation": {
                        "quaternion": {
                            "x": message_data.pose.orientation.x,
                            "y": message_data.pose.orientation.y,
                            "z": message_data.pose.orientation.z,
                            "w": message_data.pose.orientation.w
                        },
                        "rpy": None
                    }
                }
            )
            payload = self.topicSubCurrentPose.serialize(current_pose)
            self.__publish_mqtt(self.topicSubCurrentPose.private + Autoware.CONST.TOPIC.CURRENT_POSE, payload)

    def __main_loop(self):
        r = rospy.Rate(1.0/self.__period)
        while not rospy.is_shutdown():
            try:
                message_data = rospy.wait_for_message(
                    Autoware.CONST.ROSTOPIC.CURRENT_POSE, PoseStamped, timeout=self.__period)

                current_pose = CurrentPose.new_data(
                    name=self.__name,
                    time=message_data.header.stamp.secs + 0.000000001 * message_data.header.stamp.nsecs,
                    pose={
                        "position": {
                            "x": message_data.pose.position.x,
                            "y": message_data.pose.position.y,
                            "z": message_data.pose.position.z
                        },
                        "orientation": {
                            "quaternion": {
                                "x": message_data.pose.orientation.x,
                                "y": message_data.pose.orientation.y,
                                "z": message_data.pose.orientation.z,
                                "w": message_data.pose.orientation.w
                            },
                            "rpy": None
                        }
                    }
                )
                payload = self.topicSubCurrentPose.serialize(current_pose)
                self.publish(self.topicSubCurrentPose.private + Autoware.CONST.TOPIC.CURRENT_POSE, payload)
            except rospy.ROSException as e:
                # print(e)
                pass
            except rospy.ROSInterruptException:
                break

            r.sleep()
