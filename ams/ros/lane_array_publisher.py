#!/usr/bin/env python
# coding: utf-8

import rospy
from autoware_msgs.msg import LaneArray
from autoware_msgs.msg import lane as Lane
from autoware_msgs.msg import waypoint as Waypoint

from ams import Topic
from ams.nodes import EventLoop, Autoware
from ams.messages import autoware_message


class LaneArrayPublisher(EventLoop):
    def __init__(self, name):
        super(LaneArrayPublisher, self).__init__()

        self.autowarePublishTopic = Topic()
        self.autowarePublishTopic.set_id(name)
        self.autowarePublishTopic.set_root(Autoware.TOPIC.PUBLISH)
        self.autowarePublishTopic.set_message(autoware_message)

        self.add_on_message_function(self.publish_to_ros)
        self.set_subscriber(self.autowarePublishTopic.private+"/waypoints")

        rospy.init_node("ams_lane_array_publisher", anonymous=True)
        self.__ROSPublisher = rospy.Publisher(Autoware.ROSTOPIC.PUBLISH, LaneArray)

    def publish_to_ros(self, client, userdata, topic, payload):
        if topic == self.autowarePublishTopic.private+"/waypoints":
            waypointMessages = self.autowarePublishTopic.unserialize(payload)
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

