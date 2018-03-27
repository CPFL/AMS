#!/usr/bin/env python
# coding: utf-8

import json
from time import time
from ams import Topic, Schedule, Target
from ams.nodes import Vehicle, Autoware
from ams.structures import AUTOWARE_VOICE_RECOGNITION, STATE_COMMAND_PUBLISHER, \
    CURRENT_POSE_SUBSCRIBER, Pose, LANE_ARRAY_PUBLISHER, \
    LIGHT_COLOR_PUBLISHER, CLOSEST_WAYPOINT_SUBSCRIBER, DECISION_MAKER_STATES_SUBSCRIBER, TRAFFIC_SIGNAL
from ams.messages import CurrentPoseRosMessageBasis, StateCommandRosMessageBasis, ClosestWaypointRosMessageBasis, \
    LightColorRosMessageBasis, DecisionMakerStatesRosMessageBasis, LaneArrayRosMessageBasis, \
    CurrentPose, TrafficSignalStatus, LaneRosMessageBasis, WaypointRosMessageBasis, HeaderRosMessageBasis, \
    ClosestWaypoint, DecisionMakerStates

from pprint import PrettyPrinter
pp = PrettyPrinter(indent=2).pprint


class AutowareVoiceRecognition(Autoware):

    CONST = AUTOWARE_VOICE_RECOGNITION

    def __init__(self, _id, name, waypoint, arrow, route, dt=1.0):
        super().__init__(_id, name, waypoint, arrow, route, dt=dt)

        self.__voice_recognition_state = dict()
        self.__previous_state_command = None

        self.__topicPubLaneArray = Topic()
        self.__topicPubLaneArray.set_targets(
            self.target, Target.new_target(self.target.id, LANE_ARRAY_PUBLISHER.NODE_NAME))
        self.__topicPubLaneArray.set_categories(AUTOWARE_VOICE_RECOGNITION.TOPIC.CATEGORIES.LANE_ARRAY)

        self.__topicPubStateCommand = Topic()
        self.__topicPubStateCommand.set_targets(
            self.target, Target.new_target(self.target.id, STATE_COMMAND_PUBLISHER.NODE_NAME))
        self.__topicPubStateCommand.set_categories(AUTOWARE_VOICE_RECOGNITION.TOPIC.CATEGORIES.STATE_COMMAND)

        self.__topicPubLightColor = Topic()
        self.__topicPubLightColor.set_targets(
        self.target, Target.new_target(self.target.id, LIGHT_COLOR_PUBLISHER.NODE_NAME))
        self.__topicPubLightColor.set_categories(AUTOWARE_VOICE_RECOGNITION.TOPIC.CATEGORIES.LIGHT_COLOR)

        self.__topicSubVoiceRecognition = Topic()
        self.__topicSubVoiceRecognition.set_targets(Target.new_target(None, "voice"), self.target)
        self.__topicSubVoiceRecognition.set_categories(AUTOWARE_VOICE_RECOGNITION.TOPIC.CATEGORIES.VOICE)
        self.set_subscriber(self.__topicSubVoiceRecognition, self.update_voice)

        self.__topicSubCurrentPose = Topic()
        self.__topicSubCurrentPose.set_targets(
            Target.new_target(self.target.id, CURRENT_POSE_SUBSCRIBER.NODE_NAME), self.target)
        self.__topicSubCurrentPose.set_categories(CURRENT_POSE_SUBSCRIBER.TOPIC_CATEGORIES)
        self.__topicSubCurrentPose.set_message(CurrentPoseRosMessageBasis)
        self.set_subscriber(self.__topicSubCurrentPose, self.update_current_pose)

        self.__topicSubClosestWaypoint = Topic()
        self.__topicSubClosestWaypoint.set_targets(
            Target.new_target(self.target.id, CLOSEST_WAYPOINT_SUBSCRIBER.NODE_NAME), self.target)
        self.__topicSubClosestWaypoint.set_categories(CLOSEST_WAYPOINT_SUBSCRIBER.TOPIC_CATEGORIES)
        self.__topicSubClosestWaypoint.set_message(ClosestWaypointRosMessageBasis)
        self.set_subscriber(self.__topicSubClosestWaypoint, self.update_closest_waypoint)

        self.__topicSubDecisionMakerStates = Topic()
        self.__topicSubDecisionMakerStates.set_targets(
            Target.new_target(self.target.id, DECISION_MAKER_STATES_SUBSCRIBER.NODE_NAME), self.target)
        self.__topicSubDecisionMakerStates.set_categories(
            DECISION_MAKER_STATES_SUBSCRIBER.TOPIC_CATEGORIES)
        self.__topicSubDecisionMakerStates.set_message(DecisionMakerStatesRosMessageBasis)
        self.set_subscriber(self.__topicSubDecisionMakerStates, self.update_decision_maker_states)

        self.__topicSubTrafficSignal = Topic()
        self.__topicSubTrafficSignal.set_targets(Target.new_target(None, TRAFFIC_SIGNAL.NODE_NAME), self.target)
        self.__topicSubTrafficSignal.set_categories(TRAFFIC_SIGNAL.TOPIC.CATEGORIES.STATUS)
        self.__topicSubTrafficSignal.set_message(TrafficSignalStatus)
        self.set_subscriber(self.__topicSubTrafficSignal, self.update_traffic_signals)

    def publish_lane_array(self):
        schedule = self.schedules[0]
        arrow_waypoint_array = self.route.get_arrow_waypoint_array(schedule.route)
        lane_array = self.get_lane_array_from_arrow_waypoint_array(arrow_waypoint_array)

        if 0 < len(lane_array.lanes[0].waypoints):
            num = min(10, len(lane_array.lanes[0].waypoints))
            for i in range(num-1, 0, -1):
                if i == 1:
                    lane_array.lanes[0].waypoints[-i].velocity = 0
                elif (i/num)*lane_array.lanes[0].waypoints[-i-1].velocity < 1.0:
                    lane_array.lanes[0].waypoints[-i].velocity = 1.0
                else:
                    lane_array.lanes[0].waypoints[-i].velocity = (i/num)*lane_array.lanes[0].waypoints[-i-1].velocity

            start_step = min(3, len(lane_array.lanes[0].waypoints))
            for i in range(0, start_step, 1):
                if (i / start_step) * lane_array.lanes[0].waypoints[i].velocity < 1.0:
                    lane_array.lanes[0].waypoints[i].velocity = 1.0
                else:
                    lane_array.lanes[0].waypoints[i].velocity = (i / start_step) * \
                                                                lane_array.lanes[0].waypoints[i].velocity

            aw_lane_array = LaneArrayRosMessageBasis.get_template()

            for lane in lane_array.lanes:
                aw_lane = LaneRosMessageBasis.get_template()
                for waypoint in lane.waypoints:
                    aw_waypoint = WaypointRosMessageBasis.get_template()
                    aw_waypoint.pose.pose.position.x = waypoint.pose.position.x
                    aw_waypoint.pose.pose.position.y = waypoint.pose.position.y
                    aw_waypoint.pose.pose.position.z = waypoint.pose.position.z

                    aw_waypoint.pose.pose.orientation.z = waypoint.pose.orientation.quaternion.z
                    aw_waypoint.pose.pose.orientation.w = waypoint.pose.orientation.quaternion.w

                    aw_waypoint.twist.twist.linear.x = waypoint.velocity
                    aw_lane.waypoints.append(aw_waypoint)

                aw_lane_array.lanes.append(aw_lane)

            self.current_arrow_waypoint_array = arrow_waypoint_array
            payload = self.__topicPubLaneArray.serialize(aw_lane_array)
            # payload = self.__topicPubLaneArray.serialize(lane_array)
            self.publish(self.__topicPubLaneArray, payload)

    def publish_state_command(self, state):
        if self.__previous_state_command != state or state == STATE_COMMAND_PUBLISHER.STATE_CMD.SUB.ACCELERATE \
                or state == STATE_COMMAND_PUBLISHER.STATE_CMD.SUB.DECELERATE:
            data = StateCommandRosMessageBasis.new_data(
                data=state
            )
            payload = json.dumps(data)
            self.__previous_state_command = state
            self.publish(self.__topicPubStateCommand, payload)

    def publish_light_color(self):
        if self.schedules[0].event == Vehicle.CONST.ACTION.MOVE:
            monitored_route = self.get_monitored_route()
            if monitored_route is None:
                traffic_light = LIGHT_COLOR_PUBLISHER.TRAFFIC_LIGHT.RED
            else:
                inter_traffic_signal_distance = self.get_inter_traffic_signal_distance(monitored_route)
                # print("inter_traffic_signal_distance", inter_traffic_signal_distance)
                if inter_traffic_signal_distance <= 20.0:
                    traffic_light = LIGHT_COLOR_PUBLISHER.TRAFFIC_LIGHT.RED
                else:
                    traffic_light = LIGHT_COLOR_PUBLISHER.TRAFFIC_LIGHT.GREEN

            header = HeaderRosMessageBasis.get_template()
            header.stamp.secs = int(time())
            header.stamp.nsecs = int((time() - int(time())) * 1000000000)

            payload = json.dumps(LightColorRosMessageBasis.new_data(
                header=header,
                traffic_light=traffic_light
            ))
            self.publish(self.__topicPubLightColor, payload)

    def publish_initial_pose(self):
        pass

    def update_current_pose(self, _client, _userdata, _topic, payload):
        current_pose = self.__topicSubCurrentPose.unserialize(payload)
        pose_ams = Pose.new_data(
            position=current_pose["pose"]["position"],
            orientation={
                "quaternion": current_pose["pose"]["orientation"],
                "rpy": None
            }
        )
        self.current_pose = CurrentPose.new_data(
            time=current_pose["header"]["stamp"]["secs"] + 0.000000001 * current_pose["header"]["stamp"]["nsecs"],
            pose=pose_ams
        )

    def update_closest_waypoint(self, _client, _userdata, _topic, payload):
        closest_waypoint = self.__topicSubClosestWaypoint.unserialize(payload)

        self.closest_waypoint = ClosestWaypoint.new_data(
                    time=time(),
                    index=closest_waypoint["data"]
                )

    def update_decision_maker_states(self, _client, _userdata, _topic, payload):
        decision_maker_states = self.__topicSubDecisionMakerStates.unserialize(payload)
        return DecisionMakerStates.new_data(
            time=decision_maker_states.header.stamp.secs + 0.000000001*decision_maker_states.header.stamp.nsecs,
            main=decision_maker_states.main_state,
            accel=decision_maker_states.acc_state,
            steer=decision_maker_states.str_state,
            behavior=decision_maker_states.behavior_state
        )

    def update_traffic_signals(self, _client, _user_data, _topic, payload):
        traffic_signal_status = self.__topicSubTrafficSignal.unserialize(payload)
        self.traffic_signals[traffic_signal_status.route_code] = traffic_signal_status

    def update_voice(self, _client, _user_data, _topic, payload):
        if self.__voice_recognition_state is None:
            self.__voice_recognition_state = json.loads(payload)

    def set_routes(self, start_waypoint_id, goal_waypoint_id):
        current_time = time()

        vehicle_schedules = list()
        vehicle_schedule = Schedule.new_schedule(
            [Target.new_node_target(self)],
            Autoware.CONST.STATE.STOP, current_time, current_time + 100,
            None)
        vehicle_schedules = Schedule.get_merged_schedules(
            vehicle_schedules, [vehicle_schedule])

        start_point = {
            "arrow_code": self.arrow.get_arrow_codes_from_waypoint_id(start_waypoint_id)[0],
            "waypoint_id": start_waypoint_id,
        }
        goal_id = "route" + goal_waypoint_id
        goal_points = [{
            "goal_id": goal_id,
            "arrow_code": self.arrow.get_arrow_codes_from_waypoint_id(goal_waypoint_id)[0],
            "waypoint_id": goal_waypoint_id,
        }]
        shortest_routes = self.route.get_shortest_routes(start_point, goal_points, reverse=False)
        shortest_route = shortest_routes[goal_id]
        shortest_route.pop("cost")
        shortest_route.pop("goal_id")

        vehicle_schedule = Schedule.new_schedule(
            [Target.new_node_target(self)],
            Vehicle.CONST.ACTION.MOVE, current_time, current_time + 100,
            shortest_route
        )

        vehicle_schedules = Schedule.get_merged_schedules(
            vehicle_schedules, [vehicle_schedule])

        return vehicle_schedules

    def update_pose(self):
        if self.waypoint_id is None and self.current_pose is not None:
            self.update_pose_from_current_pose()
            start_waypoint_id = self.waypoint_id
            goal_waypoint_id = start_waypoint_id
            if start_waypoint_id is not None:
                start_arrow_code = self.arrow.get_arrow_codes_from_waypoint_id(start_waypoint_id)[0]
                self.set_waypoint_id_and_arrow_code(start_waypoint_id, start_arrow_code)
                self.set_velocity(3.0)
                self.set_schedules(self.set_routes(start_waypoint_id, goal_waypoint_id))
        elif self.closest_waypoint is not None:
                if self.closest_waypoint.index == -1 or \
                        len(self.current_arrow_waypoint_array) <= self.closest_waypoint.index:
                        self.closest_waypoint.index = 0
                if 0 < len(self.current_arrow_waypoint_array):
                    self.update_pose_from_closest_arrow_waypoint(
                        self.current_arrow_waypoint_array[self.closest_waypoint.index])
                else:
                    print("Lost Autoware.")
        else:
            print("Lost Autoware.")

    def __is_arrived(self):
        if self.schedules[0].route is not None and self.waypoint_id is not None:
            return self.waypoint_id == self.schedules[0].route.goal_waypoint_id
        else:
            return False

    def is_arriving_soon(self):
        return False

    def update_status(self):

        self.update_pose()
        self.publish_light_color()

        if None not in [self.waypoint_id, self.arrow_code]:
            if self.state == Vehicle.CONST.STATE.LOG_IN:
                print("publish_state_command_stop")
                self.publish_initial_pose()
                self.publish_state_command(STATE_COMMAND_PUBLISHER.STATE_CMD.SUB.STOP)
                self.state = Vehicle.CONST.STATE.STOP
            elif self.state == Vehicle.CONST.STATE.STOP:
                if self.__voice_recognition_state is not None:
                    if self.__voice_recognition_state["type"] == AUTOWARE_VOICE_RECOGNITION.VOICE_TYPE.CAR_CONTROL:
                        details = self.__voice_recognition_state["details"]
                        if details["state"] == AUTOWARE_VOICE_RECOGNITION.STATE.MOVE:
                            if 1 < len(self.schedules):
                                self.schedules.pop(0)
                                self.publish_lane_array()
                            if self.current_arrow_waypoint_array is not None:
                                self.publish_state_command(STATE_COMMAND_PUBLISHER.STATE_CMD.SUB.KEEP)
                                self.state = Vehicle.CONST.STATE.MOVE
                        else:
                            pass
                    elif self.__voice_recognition_state["type"] == \
                            AUTOWARE_VOICE_RECOGNITION.VOICE_TYPE.DESTINATION_SET:
                        details = self.__voice_recognition_state["details"]
                        goal_waypoint_id = details["waypoint"]
                        start_waypoint_id = self.waypoint_id
                        if goal_waypoint_id is not None and start_waypoint_id is not None \
                                and start_waypoint_id != goal_waypoint_id:
                            start_arrow_code = self.arrow.get_arrow_codes_from_waypoint_id(start_waypoint_id)[0]
                            self.set_waypoint_id_and_arrow_code(start_waypoint_id, start_arrow_code)
                            self.set_velocity(3.0)
                            self.set_schedules(self.set_routes(start_waypoint_id, goal_waypoint_id))

            elif self.state == Vehicle.CONST.STATE.MOVE:
                if self.__is_arrived():
                    print("Goaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaal!")
                    self.publish_state_command(STATE_COMMAND_PUBLISHER.STATE_CMD.SUB.STOP)
                    self.state = Vehicle.CONST.STATE.STOP
                    self.publish_status()
                else:
                    if self.__voice_recognition_state is not None:
                        if self.__voice_recognition_state["type"] == AUTOWARE_VOICE_RECOGNITION.VOICE_TYPE.CAR_CONTROL:
                            details = self.__voice_recognition_state["details"]
                            if details["state"] == AUTOWARE_VOICE_RECOGNITION.STATE.STOP:
                                self.publish_state_command(STATE_COMMAND_PUBLISHER.STATE_CMD.SUB.STOP)
                                self.state = Vehicle.CONST.STATE.STOP
                            elif details["state"] == AUTOWARE_VOICE_RECOGNITION.STATE.ACCELERATE:
                                self.publish_state_command(STATE_COMMAND_PUBLISHER.STATE_CMD.SUB.ACCELERATE)
                                self.publish_state_command(STATE_COMMAND_PUBLISHER.STATE_CMD.SUB.ACCELERATE)
                            elif details["state"] == AUTOWARE_VOICE_RECOGNITION.STATE.DECELERATE:
                                self.publish_state_command(STATE_COMMAND_PUBLISHER.STATE_CMD.SUB.DECELERATE)
                                self.publish_state_command(STATE_COMMAND_PUBLISHER.STATE_CMD.SUB.DECELERATE)
                            else:
                                pass

                # if self.is_arriving_soon():
                #     self.publish_state_command(STATE_COMMAND_PUBLISHER.STATE_CMD.SUB.STOP)
        else:
            if self.state == Vehicle.CONST.STATE.LOG_IN:
                print("publish_state_command_stop")
                self.publish_state_command(STATE_COMMAND_PUBLISHER.STATE_CMD.SUB.STOP)
                self.state = Vehicle.CONST.STATE.STOP

        self.__voice_recognition_state = None

