#!/usr/bin/env python
# coding: utf-8

from time import time

import rospy
import message_filters
from geometry_msgs.msg import PoseStamped, Point32

from ams import Topic, Target
from ams.nodes import EventLoop
from ams.messages import CurrentPose
from ams.structures import CURRENT_POSE_SUBSCRIBER, AUTOWARE

import pprint
pp = pprint.PrettyPrinter(indent=2)


class CurrentPoseSubscriber(EventLoop):
    
    CONST = CURRENT_POSE_SUBSCRIBER

    def __init__(self, _id, period):
        super(CurrentPoseSubscriber, self).__init__(_id)

        self.__previous_time = time()
        self.__period = period

        self.__topicPubCurrentPose = Topic()
        self.__topicPubCurrentPose.set_targets(
            self.target, Target.new_target(self.target.id, AUTOWARE.NODE_NAME)
        )
        self.__topicPubCurrentPose.set_categories(CURRENT_POSE_SUBSCRIBER.TOPIC_CATEGORIES)

        rospy.init_node(CURRENT_POSE_SUBSCRIBER.ROSNODE, anonymous=True)

        if self.__period < 0.1:
            self.__publish_mqtt = self.publish
            self.set_main_loop(rospy.spin)

            self.__ROSSubscriber = message_filters.Subscriber(
                CURRENT_POSE_SUBSCRIBER.ROSTOPIC, PoseStamped, queue_size=1)
            self.__ROSSubscriber.registerCallback(self.on_message_from_ros)
        else:
            self.set_main_loop(self.__main_loop)

    def on_message_from_ros(self, message_data, _user_data):
        current_time = time()
        if self.__period < current_time - self.__previous_time:
            self.__previous_time += (1+int((current_time - self.__previous_time)/self.__period)) * self.__period

            current_pose = CurrentPose.new_data(
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
            payload = self.__topicPubCurrentPose.serialize(current_pose)
            self.__publish_mqtt(self.__topicPubCurrentPose, payload)

    def __main_loop(self):
        r = rospy.Rate(1.0/self.__period)
        while not rospy.is_shutdown():
            try:
                message_data = rospy.wait_for_message(
                    CURRENT_POSE_SUBSCRIBER.ROSTOPIC, PoseStamped, timeout=self.__period)

                current_pose = CurrentPose.new_data(
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
                payload = self.__topicPubCurrentPose.serialize(current_pose)
                self.publish(self.__topicPubCurrentPose, payload)
            except rospy.ROSException as e:
                # print(e)
                pass
            except rospy.ROSInterruptException:
                break

            r.sleep()
