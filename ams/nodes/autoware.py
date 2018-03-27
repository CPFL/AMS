#!/usr/bin/env python
# coding: utf-8

from time import time

from ams import Topic, Route, Schedule, MapMatch, Target
from ams.nodes import Vehicle
from ams.messages import CurrentPoseRosMessageBasis, ClosestWaypoint, DecisionMakerStates, \
    LaneArray, StateCommand, LightColor, TrafficSignalStatus, LaneArrayRosMessageBasis, \
    LaneRosMessageBasis, WaypointRosMessageBasis, ClosestWaypointRosMessageBasis, DecisionMakerStatesRosMessageBasis, \
    StateCommandRosMessageBasis, HeaderRosMessageBasis, LightColorRosMessageBasis, CurrentPose
from ams.structures import AUTOWARE, TRAFFIC_SIGNAL,\
    LANE_ARRAY_PUBLISHER, STATE_COMMAND_PUBLISHER, LIGHT_COLOR_PUBLISHER,\
    CURRENT_POSE_SUBSCRIBER, CLOSEST_WAYPOINT_SUBSCRIBER, DECISION_MAKER_STATES_SUBSCRIBER


from pprint import PrettyPrinter
pp = PrettyPrinter(indent=2).pprint


class Autoware(Vehicle):

    CONST = AUTOWARE

    def __init__(self, _id, name, waypoint, arrow, route, dt=1.0):
        super().__init__(_id, name, waypoint, arrow, route, dt=dt)

        self.name = name
        self.__map_match = MapMatch()
        self.__map_match.set_waypoint(self.waypoint)
        self.__map_match.set_arrow(self.arrow)

        self.current_pose = None
        self.closest_waypoint = None
        self.decision_maker_states = None
        self.current_arrow_waypoint_array = []
        self.__previous_state_command = None
        self.traffic_signals = {}

        self.__topicPubLaneArray = Topic()
        self.__topicPubLaneArray.set_targets(
            self.target, Target.new_target(self.target.id, LANE_ARRAY_PUBLISHER.NODE_NAME))
        self.__topicPubLaneArray.set_categories(AUTOWARE.TOPIC.CATEGORIES.LANE_ARRAY)

        self.__topicPubStateCommand = Topic()
        self.__topicPubStateCommand.set_targets(
            self.target, Target.new_target(self.target.id, STATE_COMMAND_PUBLISHER.NODE_NAME))
        self.__topicPubStateCommand.set_categories(AUTOWARE.TOPIC.CATEGORIES.STATE_COMMAND)

        self.__topicPubLightColor = Topic()
        self.__topicPubLightColor.set_targets(
            self.target, Target.new_target(self.target.id, LIGHT_COLOR_PUBLISHER.NODE_NAME))
        self.__topicPubLightColor.set_categories(AUTOWARE.TOPIC.CATEGORIES.LIGHT_COLOR)

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
                lane_array.lanes[0].waypoints[-i].velocity = (i/num)*lane_array.lanes[0].waypoints[-i-1].velocity

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
            self.publish(self.__topicPubLaneArray, payload)

    def publish_state_command(self, state):
        if self.__previous_state_command != state:
            data = StateCommandRosMessageBasis.new_data(
                data=state
            )
            payload = self.__topicPubStateCommand.serialize(data)
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

            payload = self.__topicPubLightColor.serialize(LightColorRosMessageBasis.new_data(
                header=header,
                traffic_light=traffic_light
            ))
            self.publish(self.__topicPubLightColor, payload)

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

    def update_pose_from_current_pose(self):
        location = self.__map_match.get_matched_location_on_arrows(
            self.current_pose.pose, self.arrow.get_arrow_codes())
        # print("update_pose_from_current_pose", location)
        self.arrow_code = location.arrow_code
        self.waypoint_id = location.waypoint_id
        self.np_position = self.waypoint.get_np_position(self.waypoint_id)
        self.yaw = self.arrow.get_yaw(self.arrow_code, self.waypoint_id)

    def update_pose_from_closest_arrow_waypoint(self, closest_arrow_waypoint):
        # print("update_pose_from_closest_arrow_waypoint", closest_arrow_waypoint)
        self.arrow_code = closest_arrow_waypoint["arrow_code"]
        self.waypoint_id = closest_arrow_waypoint["waypoint_id"]
        self.np_position = self.waypoint.get_np_position(self.waypoint_id)
        self.yaw = self.arrow.get_yaw(self.arrow_code, self.waypoint_id)

    def get_lane_array_from_arrow_waypoint_array(self, arrow_waypoint_array):
        waypoints = []
        for arrow_waypoint in arrow_waypoint_array:
            waypoints.append({
                "pose": self.waypoint.get_pose(arrow_waypoint["waypoint_id"]),
                "velocity": self.velocity
            })
        lanes = [{"waypoints": waypoints}]
        return LaneArray.new_data(
            time=time(),
            lanes=lanes
        )

    def get_inter_traffic_signal_distance(self, monitored_route):
        monitored_arrow_codes = monitored_route.arrow_codes
        inter_traffic_signal_distance = AUTOWARE.FLOAT_MAX

        not_green_traffic_signal_route_codes = list(map(
            lambda x: x.route_code, filter(
                lambda x: x.state in [
                    TRAFFIC_SIGNAL.STATE.UNKNOWN, TRAFFIC_SIGNAL.STATE.YELLOW, TRAFFIC_SIGNAL.STATE.RED],
                self.traffic_signals.values())))

        new_monitored_route = None
        for i, monitored_arrow_code in enumerate(monitored_arrow_codes):
            for not_green_traffic_signal_route_code in not_green_traffic_signal_route_codes:
                if monitored_arrow_code in not_green_traffic_signal_route_code:
                    not_green_traffic_signal_route = Route.decode_route_code(not_green_traffic_signal_route_code)
                    if monitored_arrow_code == not_green_traffic_signal_route.arrow_codes[0]:
                        waypoint_ids = self.arrow.get_waypoint_ids(monitored_arrow_code)
                        if self.waypoint_id not in waypoint_ids or \
                                waypoint_ids.index(self.waypoint_id) <= waypoint_ids.index(
                                    not_green_traffic_signal_route.start_waypoint_id):
                            new_monitored_route = Route.new_route(
                                monitored_route.start_waypoint_id,
                                not_green_traffic_signal_route.start_waypoint_id,
                                monitored_arrow_codes[:i+1])
                            break
            if new_monitored_route is not None:
                break

        if new_monitored_route is not None:
            inter_traffic_signal_distance = self.route.get_route_length(new_monitored_route)

        return inter_traffic_signal_distance

    def get_monitored_route(self, distance=100.0):
        if distance <= 0:
            return None

        arrow_codes = self.schedules[0].route.arrow_codes
        if arrow_codes is None:
            return None

        i_s = 0
        if self.arrow_code in arrow_codes:
            i_s = arrow_codes.index(self.arrow_code)
        arrow_codes = arrow_codes[i_s:]
        route = Route.new_route(
            self.waypoint_id,
            self.arrow.get_waypoint_ids(self.schedules[0].route.arrow_codes[-1])[-1],
            arrow_codes)
        return self.route.get_sliced_route(route, distance)

    def update_pose(self):
        if self.closest_waypoint is None or self.closest_waypoint.index == -1:
            if self.current_pose is not None:
                self.update_pose_from_current_pose()
            else:
                print("Lost Autoware.")
        else:
            if 0 < len(self.current_arrow_waypoint_array):
                self.update_pose_from_closest_arrow_waypoint(
                    self.current_arrow_waypoint_array[self.closest_waypoint.index])
            else:
                print("Lost Autoware.")

    def __is_arrived(self):
        if self.decision_maker_states is not None:
            if self.decision_maker_states.behavior == DECISION_MAKER_STATES_SUBSCRIBER.BEHAVIOR.WAIT_ORDERS:
                return True
        return False

    def is_arriving_soon(self):
        return False

    def update_status(self):

        print(self.waypoint_id, self.arrow_code, self.state, self.schedules)
        self.update_pose()
        self.publish_light_color()

        if None not in [self.waypoint_id, self.arrow_code]:
            current_time = time()

            if self.state == Vehicle.CONST.STATE.LOG_IN:
                print("publish_state_command_stop")
                self.publish_state_command(STATE_COMMAND_PUBLISHER.STATE_CMD.SUB.STOP)
                self.state = Vehicle.CONST.STATE.STOP
            elif self.state == Vehicle.CONST.STATE.STOP:
                if 1 < len(self.schedules) and self.schedules[0].period.end < current_time:
                    self.schedules.pop(0)

                    self.publish_lane_array()

                    # update next schedule
                    dif_time = current_time - self.schedules[0].period.start
                    self.schedules = Schedule.get_shifted_schedules(self.schedules, dif_time)

                    self.state = Vehicle.CONST.STATE.MOVE
                    self.publish_state_command(STATE_COMMAND_PUBLISHER.STATE_CMD.SUB.KEEP)
            elif self.state == Vehicle.CONST.STATE.MOVE:
                if self.__is_arrived():
                    pass
                # if self.is_arriving_soon():
                #     self.publish_state_command(STATE_COMMAND_PUBLISHER.STATE_CMD.SUB.STOP)
        else:
            if self.state == Vehicle.CONST.STATE.LOG_IN:
                print("publish_state_command_stop")
                self.publish_state_command(STATE_COMMAND_PUBLISHER.STATE_CMD.SUB.STOP)
                self.state = Vehicle.CONST.STATE.STOP
