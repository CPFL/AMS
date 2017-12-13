#!/usr/bin/env python
# coding: utf-8

import rospy
from autoware_msgs.msg import LaneArray
from autoware_msgs.msg import lane as Lane
from autoware_msgs.msg import waypoint as Waypoint
import json
from eventLoop import EventLoop
from const import CONST


class LaneArrayPublisher(EventLoop):
    def __init__(self):
        super(LaneArrayPublisher, self).__init__()
        rospy.init_node("ams_lane_array_publisher", anonymous=True)
        self.__ROSPublisher = rospy.Publisher('/based/lane_waypoints_array', LaneArray)

        self.setOnMessageFunction(
            onMessageFunction=self.__onMessageFunction,
            userData={
                "publishToROS": self.publishToROS,
            }
        )
        self.setSubscriber(CONST.TOPICS.AUTOWARE.LANES)

    def __onMessageFunction(self, client, userdata, topic, message):
        if topic == CONST.TOPICS.AUTOWARE.LANES:
            userdata["publishToROS"](message)
        else:
            print("unknown topic message", topic, message)

    def publishToROS(self, message):
        # print(message)
        waypointMessages = json.loads(message)
        laneArray = LaneArray()

        # WaypointState.stopline_state = 2  # keep stop

        lane = Lane()
        for waypointMessage in waypointMessages:
            waypoint = Waypoint()
            waypoint.pose.pose.position.x = waypointMessage["position"]["x"]
            waypoint.pose.pose.position.y = waypointMessage["position"]["y"]
            waypoint.pose.pose.position.z = waypointMessage["position"]["z"]

            waypoint.pose.pose.orientation.z = waypointMessage["orientation"]["z"]
            waypoint.pose.pose.orientation.w = waypointMessage["orientation"]["w"]

            waypoint.twist.twist.linear.x = waypointMessage["velocity"]
            lane.waypoints.append(waypoint)

        # lane.waypoints[-1].wpstate.stopline_state = 2

        laneArray.lanes = [lane]
        self.__ROSPublisher.publish(laneArray)


if __name__ == '__main__':
    laneArrayPublisher = LaneArrayPublisher()
    print("laneArrayPublisher {} on {}".format(laneArrayPublisher.getEventLoopID(), laneArrayPublisher.getPid()))
    laneArrayPublisher.start()
