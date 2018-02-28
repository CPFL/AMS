#!/usr/bin/env python
# coding: utf-8

import rospy
from autoware_msgs.msg import LaneArray
from autoware_msgs.msg import lane as Lane
from autoware_msgs.msg import waypoint as Waypoint

from ams import Topic
from ams.nodes import EventLoop, Autoware
from ams.messages import LaneArray as AMSLaneArray


class LaneArrayPublisher(EventLoop):
    def __init__(self, name):
        super(LaneArrayPublisher, self).__init__()

        self.topicPubLaneArray = Topic()
        self.topicPubLaneArray.set_id(name)
        self.topicPubLaneArray.set_root(Autoware.CONST.TOPIC.PUBLISH)

        self.set_subscriber(self.topicPubLaneArray.private+Autoware.CONST.TOPIC.WAYPOINTS, self.publish_to_ros)

        rospy.init_node(Autoware.CONST.ROSNODE.AMS_LANE_ARRAY_PUBLISHER, anonymous=True)
        self.__ROSPublisher = rospy.Publisher(Autoware.CONST.ROSTOPIC.WAYPOINTS, LaneArray, queue_size=1)

    def publish_to_ros(self, _client, _userdata, topic, payload):
        if topic == self.topicPubLaneArray.private+Autoware.CONST.TOPIC.WAYPOINTS:
            ams_lane_array = AMSLaneArray.new_data(**self.topicPubLaneArray.unserialize(payload))
            lane_array = LaneArray()

            # WaypointState.stopline_state = 2  # keep stop

            for ams_lane in ams_lane_array.lanes:
                lane = Lane()
                for ams_waypoint in ams_lane.waypoints:
                    waypoint = Waypoint()
                    waypoint.pose.pose.position.x = ams_waypoint.pose.position.x
                    waypoint.pose.pose.position.y = ams_waypoint.pose.position.y
                    waypoint.pose.pose.position.z = ams_waypoint.pose.position.z
    
                    waypoint.pose.pose.orientation.z = ams_waypoint.pose.orientation.quaternion.z
                    waypoint.pose.pose.orientation.w = ams_waypoint.pose.orientation.quaternion.w
    
                    waypoint.twist.twist.linear.x = ams_waypoint.velocity
                    lane.waypoints.append(waypoint)

                lane_array.lanes.append(lane)

            # lane.waypoints[-1].wpstate.stopline_state = 2

            self.__ROSPublisher.publish(lane_array)
