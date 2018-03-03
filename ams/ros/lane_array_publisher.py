#!/usr/bin/env python
# coding: utf-8

import rospy
from autoware_msgs.msg import LaneArray as AWLaneArray
from autoware_msgs.msg import lane as AWLane
from autoware_msgs.msg import waypoint as AWWaypoint

from ams import Topic, Target
from ams.nodes import EventLoop
from ams.messages import LaneArray as LaneArray
from ams.structures import LANE_ARRAY_PUBLISHER, AUTOWARE


class LaneArrayPublisher(EventLoop):

    CONST = LANE_ARRAY_PUBLISHER

    def __init__(self, _id):
        super(LaneArrayPublisher, self).__init__(_id)

        self.__topicSubLaneArray = Topic()
        self.__topicSubLaneArray.set_targets(
            Target.new_target(self.target.id, AUTOWARE.NODE_NAME), self.target
        )
        self.__topicSubLaneArray.set_categories(AUTOWARE.TOPIC.CATEGORIES.LANE_ARRAY)
        self.__topicSubLaneArray.set_message(LaneArray)
        self.set_subscriber(self.__topicSubLaneArray, self.publish_to_ros)

        rospy.init_node(LANE_ARRAY_PUBLISHER.ROSNODE, anonymous=True)
        self.__ROSPublisher = rospy.Publisher(LANE_ARRAY_PUBLISHER.ROSTOPIC, AWLaneArray, queue_size=1)

    def publish_to_ros(self, _client, _userdata, _topic, payload):
            lane_array = self.__topicSubLaneArray.unserialize(payload)
            aw_lane_array = AWLaneArray()

            # WaypointState.stopline_state = 2  # keep stop

            for lane in lane_array.lanes:
                aw_lane = AWLane()
                for waypoint in lane.waypoints:
                    aw_waypoint = AWWaypoint()
                    aw_waypoint.pose.pose.position.x = waypoint.pose.position.x
                    aw_waypoint.pose.pose.position.y = waypoint.pose.position.y
                    aw_waypoint.pose.pose.position.z = waypoint.pose.position.z

                    aw_waypoint.pose.pose.orientation.z = waypoint.pose.orientation.quaternion.z
                    aw_waypoint.pose.pose.orientation.w = waypoint.pose.orientation.quaternion.w

                    aw_waypoint.twist.twist.linear.x = waypoint.velocity
                    aw_lane.waypoints.append(aw_waypoint)

                aw_lane_array.lanes.append(aw_lane)

            # lane.waypoints[-1].wpstate.stopline_state = 2

            self.__ROSPublisher.publish(aw_lane_array)
