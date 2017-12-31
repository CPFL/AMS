#!/usr/bin/env python
# coding: utf-8

import rospy
import json
from eventLoop import EventLoop
from const import CONST

import message_filters
from geometry_msgs.msg import PoseStamped, Point32
from time import time

import pprint
pp = pprint.PrettyPrinter(indent=2)


class CurrentPoseSubscriber(EventLoop):
    def __init__(self):
        super(CurrentPoseSubscriber, self).__init__()
        self.__previous_time = time()
        self.__period = 1.0  # [sec]

        self.connect()
        self.setMainLoop(rospy.spin)

        rospy.init_node("ams_current_pose_subscriber", anonymous=True)
        self.__ROSSubscriber = message_filters.Subscriber('/current_pose', PoseStamped)
        self.__ROSSubscriber.registerCallback(self.onMessageFromROS, self.publish)

    def onMessageFromROS(self, messageData, publish):
        current_time = time()
        if self.__period < current_time - self.__previous_time:
            self.__previous_time += (1+int((current_time - self.__previous_time)/self.__period)) * self.__period
            message = json.dumps({
                "stamp": messageData.header.stamp.secs + 0.000000001*messageData.header.stamp.nsecs,
                "frame_id": messageData.header.frame_id,
                "position": {
                    "x": messageData.pose.position.x,
                    "y": messageData.pose.position.y,
                    "z": messageData.pose.position.z,
                },
                "orientation": {
                    "x": messageData.pose.orientation.x,
                    "y": messageData.pose.orientation.y,
                    "z": messageData.pose.orientation.z,
                    "w": messageData.pose.orientation.w,
                }
            })
            publish(CONST.TOPICS.AUTOWARE.POSE, message)


if __name__ == '__main__':
    currentPoseSubscriber = CurrentPoseSubscriber()
    print("currentPoseSubscriber {} on {}".format(currentPoseSubscriber.getEventLoopID(), currentPoseSubscriber.getPid()))
    currentPoseSubscriber.start()
