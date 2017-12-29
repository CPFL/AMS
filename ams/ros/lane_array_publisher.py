#!/usr/bin/env python
# coding: utf-8

import rospy
from autoware_msgs.msg import LaneArray
from autoware_msgs.msg import lane as Lane
from autoware_msgs.msg import waypoint as Waypoint
from eventLoop import EventLoop
from topic import Topic
from const.autoware import AUTOWARE


class LaneArrayPublisher(EventLoop):
    def __init__(self, name):
        super(LaneArrayPublisher, self).__init__()

        self.autowarePublishTopic = Topic()
        self.autowarePublishTopic.setID(name)
        self.autowarePublishTopic.setRoot(AUTOWARE.TOPIC.PUBLISH)
        self.autowarePublishTopic.load(AUTOWARE.TOPIC.MESSAGE_FILE)

        self.add_on_message_function(self.publishToROS)
        self.setSubscriber(self.autowarePublishTopic.private+"/waypoints")

        rospy.init_node("ams_lane_array_publisher", anonymous=True)
        self.__ROSPublisher = rospy.Publisher(AUTOWARE.ROSTOPIC.PUBLISH, LaneArray)

    def publishToROS(self, client, userdata, topic, payload):
        if topic == self.autowarePublishTopic.private+"/waypoints":
            # print(message)
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


if __name__ == '__main__':
    import sys
    laneArrayPublisher = LaneArrayPublisher(name=sys.argv[1])
    print("laneArrayPublisher {} on {}".format(laneArrayPublisher.getEventLoopID(), laneArrayPublisher.getPid()))
    laneArrayPublisher.start()
