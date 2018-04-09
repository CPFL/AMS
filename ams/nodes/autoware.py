#!/usr/bin/env python
# coding: utf-8

from time import time
from copy import deepcopy

from transitions import Machine

from ams import logger, Topic, Route, Schedule, MapMatch, Target
from ams.nodes import Vehicle, SimCar
from ams.messages import TrafficSignalStatus, ROSMessage
from ams.structures import AUTOWARE, TRAFFIC_SIGNAL, Pose, Position, Orientation, Quaternion


class Autoware(Vehicle):

    CONST = AUTOWARE

    def __init__(self, _id, name, waypoint, arrow, route, dt=1.0):
        super().__init__(_id, name, waypoint, arrow, route, dt=dt)

        self.upper_distance_from_stopline = AUTOWARE.DEFAULT_UPPER_DISTANCE_FROM_STOPLINE
        self.state_machine = self.get_state_machine(AUTOWARE.STATE.LAUNCHED)

        self.__map_match = MapMatch()
        self.__map_match.set_waypoint(self.waypoint)
        self.__map_match.set_arrow(self.arrow)

        self.ros_closest_waypoint = None
        self.ros_closest_waypoint_lock = self.manager.Lock()

        self.ros_current_pose = None
        self.ros_current_pose_lock = self.manager.Lock()

        self.ros_decisionmaker_states = None
        self.ros_decisionmaker_states_lock = self.manager.Lock()

        self.current_arrow_waypoint_array = []

        self.traffic_signals = self.manager.dict()
        self.traffic_signals_lock = self.manager.Lock()

        self.__previous_state_command = None

        self.__topicPubBasedLaneWaypointsArray = Topic()
        self.__topicPubBasedLaneWaypointsArray.set_targets(
            self.target, Target.new_target(self.target.id, AUTOWARE.TOPIC.ROS_NODE_NAME))
        self.__topicPubBasedLaneWaypointsArray.set_categories(AUTOWARE.TOPIC.CATEGORIES.BASED_LANE_WAYPOINTS_ARRAY)

        self.__topicPubStateCmd = Topic()
        self.__topicPubStateCmd.set_targets(
            self.target, Target.new_target(self.target.id, AUTOWARE.TOPIC.ROS_NODE_NAME))
        self.__topicPubStateCmd.set_categories(AUTOWARE.TOPIC.CATEGORIES.STATE_CMD)

        self.__topicPubLightColor = Topic()
        self.__topicPubLightColor.set_targets(
            self.target, Target.new_target(self.target.id, AUTOWARE.TOPIC.ROS_NODE_NAME))
        self.__topicPubLightColor.set_categories(AUTOWARE.TOPIC.CATEGORIES.LIGHT_COLOR)

        self.__topicSubCurrentPose = Topic()
        self.__topicSubCurrentPose.set_targets(
            Target.new_target(self.target.id, AUTOWARE.TOPIC.ROS_NODE_NAME), self.target)
        self.__topicSubCurrentPose.set_categories(AUTOWARE.TOPIC.CATEGORIES.CURRENT_POSE)
        self.__topicSubCurrentPose.set_message(ROSMessage.CurrentPose)
        self.set_subscriber(self.__topicSubCurrentPose, self.update_current_pose)

        self.__topicSubClosestWaypoint = Topic()
        self.__topicSubClosestWaypoint.set_targets(
            Target.new_target(self.target.id, AUTOWARE.TOPIC.ROS_NODE_NAME), self.target)
        self.__topicSubClosestWaypoint.set_categories(AUTOWARE.TOPIC.CATEGORIES.CLOSEST_WAYPOINT)
        self.__topicSubClosestWaypoint.set_message(ROSMessage.ClosestWaypoint)
        self.set_subscriber(self.__topicSubClosestWaypoint, self.update_closest_waypoint)

        self.__topicSubDecisionMakerStates = Topic()
        self.__topicSubDecisionMakerStates.set_targets(
            Target.new_target(self.target.id, AUTOWARE.TOPIC.ROS_NODE_NAME), self.target)
        self.__topicSubDecisionMakerStates.set_categories(AUTOWARE.TOPIC.CATEGORIES.DECISION_MAKER_STATES)
        self.__topicSubDecisionMakerStates.set_message(ROSMessage.DecisionMakerStates)
        self.set_subscriber(self.__topicSubDecisionMakerStates, self.update_decisionmaker_states)

        self.__topicSubTrafficSignalStatus = Topic()
        self.__topicSubTrafficSignalStatus.set_targets(Target.new_target(None, TRAFFIC_SIGNAL.NODE_NAME))
        self.__topicSubTrafficSignalStatus.set_categories(TRAFFIC_SIGNAL.TOPIC.CATEGORIES.STATUS)
        self.__topicSubTrafficSignalStatus.set_message(TrafficSignalStatus)
        self.set_subscriber(self.__topicSubTrafficSignalStatus, self.update_traffic_signals)

    def set_upper_distance_from_stopline(self, distance_from_stopline):
        self.upper_distance_from_stopline = distance_from_stopline

    def publish_lane_array(self, route):
        arrow_waypoint_array = self.route.get_arrow_waypoint_array(route)
        ros_lane_array = self.get_ros_lane_array_from_arrow_waypoint_array(arrow_waypoint_array)

        if ros_lane_array is not None:
            self.current_arrow_waypoint_array = arrow_waypoint_array
            payload = self.__topicPubBasedLaneWaypointsArray.serialize(ros_lane_array)
            self.publish(self.__topicPubBasedLaneWaypointsArray, payload)

    def publish_state_command(self, state_command):
        # if self.__previous_state_command != state_command:
        if True:
            payload = self.__topicPubStateCmd.serialize(state_command)
            self.__previous_state_command = state_command
            self.publish(self.__topicPubStateCmd, payload)

    def publish_init_state_command(self, decisionmaker_states):
        if decisionmaker_states.main_state != AUTOWARE.ROS.DECISION_MAKER_STATES.MAIN.INITIAL:
            state_command = ROSMessage.StateCommand.new_data()
            state_command.data = AUTOWARE.ROS.STATE_CMD.MAIN.INIT
            self.publish_state_command(state_command)

    def publish_drive_state_command(self, decisionmaker_states):
        if any([
            all([
                decisionmaker_states.main_state == AUTOWARE.ROS.DECISION_MAKER_STATES.MAIN.MISSION_COMPLETE,
                decisionmaker_states.behavior_state == AUTOWARE.ROS.DECISION_MAKER_STATES.BEHAVIOR.WAIT_ORDERS
            ]),
            decisionmaker_states.main_state == AUTOWARE.ROS.DECISION_MAKER_STATES.MAIN.INITIAL
        ]):
            state_command = ROSMessage.StateCommand.new_data()
            state_command.data = AUTOWARE.ROS.STATE_CMD.MAIN.DRIVE
            self.publish_state_command(state_command)

    def publish_stop_state_command(self, decisionmaker_states):
        if decisionmaker_states.acc_state != AUTOWARE.ROS.DECISION_MAKER_STATES.ACC.STOP:
            state_command = ROSMessage.StateCommand.new_data()
            state_command.data = AUTOWARE.ROS.STATE_CMD.SUB.STOP
            self.publish_state_command(state_command)

    def publish_light_color(self):
        monitored_route = self.get_monitored_route()
        if monitored_route is None:
            traffic_light = AUTOWARE.ROS.TRAFFIC_LIGHT.RED
        else:
            distance_from_stopline = self.get_distance_from_stopline(monitored_route)
            if distance_from_stopline <= self.upper_distance_from_stopline:
                traffic_light = AUTOWARE.ROS.TRAFFIC_LIGHT.RED
            else:
                traffic_light = AUTOWARE.ROS.TRAFFIC_LIGHT.GREEN
        header = ROSMessage.Header.get_template()
        header.stamp.secs = int(time())
        header.stamp.nsecs = int((time() - int(time())) * 1000000000)

        payload = self.__topicPubLightColor.serialize(ROSMessage.LightColor.new_data(
            header=header,
            traffic_light=traffic_light
        ))
        self.publish(self.__topicPubLightColor, payload)

    def update_current_pose(self, _client, _userdata, _topic, payload):
        self.ros_current_pose_lock.acquire()
        self.ros_current_pose = ROSMessage.CurrentPose.new_data(**self.__topicSubCurrentPose.unserialize(payload))
        self.ros_current_pose_lock.release()

    def update_closest_waypoint(self, _client, _userdata, _topic, payload):
        self.ros_closest_waypoint_lock.acquire()
        self.ros_closest_waypoint = \
            ROSMessage.ClosestWaypoint.new_data(**self.__topicSubClosestWaypoint.unserialize(payload))
        self.ros_closest_waypoint_lock.release()

    def update_decisionmaker_states(self, _client, _userdata, _topic, payload):
        self.ros_decisionmaker_states_lock.acquire()
        self.ros_decisionmaker_states = \
            ROSMessage.DecisionMakerStates.new_data(**self.__topicSubDecisionMakerStates.unserialize(payload))
        self.ros_decisionmaker_states_lock.release()

    def update_traffic_signals(self, _client, _user_data, _topic, payload):
        # todo: localize
        traffic_signal_status = self.__topicSubTrafficSignalStatus.unserialize(payload)

        self.traffic_signals_lock.acquire()
        self.traffic_signals[traffic_signal_status.route_code] = traffic_signal_status
        self.traffic_signals_lock.release()

    @staticmethod
    def get_current_pose_from_ros_current_pose(ros_current_pose):
        return Pose.new_data(
            position=Position.new_data(**ros_current_pose.pose.position),
            orientation=Orientation.new_data(
                quaternion=Quaternion.new_data(**ros_current_pose.pose.orientation),
            )
        )

    def update_pose_from_current_pose(self):
        self.ros_current_pose_lock.acquire()
        ros_current_pose = deepcopy(self.ros_current_pose)
        self.ros_current_pose_lock.release()

        if ros_current_pose is not None:
            current_pose = Autoware.get_current_pose_from_ros_current_pose(ros_current_pose)
            self.set_location(
                self.__map_match.get_matched_location_on_arrows(current_pose, self.arrow.get_arrow_codes()))
            self.remove_subscriber(self.__topicSubCurrentPose)

    def update_pose_from_closest_arrow_waypoint(self):
        self.ros_closest_waypoint_lock.acquire()
        ros_closest_waypoint = deepcopy(self.ros_closest_waypoint)
        self.ros_closest_waypoint_lock.release()

        if ros_closest_waypoint is not None and \
                0 <= ros_closest_waypoint.data < len(self.current_arrow_waypoint_array):
            closest_arrow_waypoint = self.current_arrow_waypoint_array[ros_closest_waypoint.data]
            self.set_waypoint_id_and_arrow_code(
                closest_arrow_waypoint["waypoint_id"], closest_arrow_waypoint["arrow_code"])

    def get_ros_lane_array_from_arrow_waypoint_array(self, arrow_waypoint_array):
        if 0 == len(arrow_waypoint_array):
            return None

        ros_lane_array = ROSMessage.LaneArray.new_data()
        ros_lane = ROSMessage.Lane.new_data()
        for arrow_waypoint in arrow_waypoint_array:
            pose = self.waypoint.get_pose(arrow_waypoint["waypoint_id"])
            ros_waypoint = ROSMessage.Waypoint.new_data()
            ros_waypoint.pose.pose.position.x = pose.position.x
            ros_waypoint.pose.pose.position.y = pose.position.y
            ros_waypoint.pose.pose.position.z = pose.position.z

            ros_waypoint.pose.pose.orientation.z = pose.orientation.quaternion.z
            ros_waypoint.pose.pose.orientation.w = pose.orientation.quaternion.w

            ros_waypoint.twist.twist.linear.x = 0.2*self.waypoint.get_speed_limit(arrow_waypoint["waypoint_id"])
            ros_lane.waypoints.append(ros_waypoint)

        ros_lane.header.stamp.secs = int(time()+1)
        ros_lane_array.lanes.append(ros_lane)
        return ros_lane_array

    get_distance_from_stopline = SimCar.get_distance_from_stopline

    get_monitored_route = SimCar.get_monitored_route

    def get_random_route(self):
        import random
        start_point = {
            "arrow_code": self.status.location.arrow_code,
            "waypoint_id": self.status.location.waypoint_id,
        }
        while True:
            while True:
                goal_arrow_code = random.choice(self.arrow.get_arrow_codes())
                goal_waypoint_id = random.choice(self.arrow.get_waypoint_ids(goal_arrow_code))
                if goal_waypoint_id != self.status.location.waypoint_id:
                    break

            goal_id = None
            goal_points = [{
                "goal_id": goal_id,
                "arrow_code": goal_arrow_code,
                "waypoint_id": goal_waypoint_id,
            }]

            shortest_routes = self.route.get_shortest_routes(start_point, goal_points, reverse=False)
            if 0 == len(shortest_routes):
                continue
            shortest_route = shortest_routes[goal_id]
            shortest_route.pop("cost")
            shortest_route.pop("goal_id")
            break

        return Route.new_route(self.status.location.waypoint_id, goal_waypoint_id, shortest_route.arrow_codes)

    def add_random_schedule(self, current_time, schedules):
        synchronize_route_schedule = Schedule.new_schedule(
            [self.target],
            AUTOWARE.TRIGGER.SYNCHRONIZE_ROUTE, current_time, current_time + 5,
            self.route.new_point_route(
                self.status.location.waypoint_id,
                self.status.location.arrow_code
            )
        )
        random_route = self.get_random_route()
        move_schedule = Schedule.new_schedule(
            [self.target],
            AUTOWARE.TRIGGER.MOVE, synchronize_route_schedule.period.end, synchronize_route_schedule.period.end + 100,
            random_route
        )
        stop_schedule = Schedule.new_schedule(
            [self.target],
            AUTOWARE.TRIGGER.STOP, move_schedule.period.end, move_schedule.period.end + 10,
            self.route.new_point_route(
                move_schedule.route.goal_waypoint_id,
                move_schedule.route.arrow_codes[-1]
            )
        )
        get_ready_schedule = Schedule.new_schedule(
            [self.target],
            AUTOWARE.TRIGGER.GET_READY, move_schedule.period.end, move_schedule.period.end + 1,
            self.route.new_point_route(
                move_schedule.route.goal_waypoint_id,
                move_schedule.route.arrow_codes[-1]
            )
        )
        reschedule_schedule = Schedule.new_schedule(
            [self.target],
            AUTOWARE.TRIGGER.SCHEDULE, move_schedule.period.end, move_schedule.period.end + 1,
            self.route.new_point_route(
                move_schedule.route.goal_waypoint_id,
                move_schedule.route.arrow_codes[-1]
            )
        )
        schedules[:] = Schedule.get_merged_schedules(
            schedules,
            [
                synchronize_route_schedule, move_schedule, stop_schedule, get_ready_schedule, reschedule_schedule
            ]
        )

    def get_state_machine(self, initial_state):
        machine = Machine(
            states=list(AUTOWARE.STATE),
            initial=initial_state,
        )
        machine.add_transitions([
            {
                "trigger": AUTOWARE.TRIGGER.ACTIVATE,
                "source": AUTOWARE.STATE.LAUNCHED, "dest": AUTOWARE.STATE.STAND_BY,
                "conditions": [self.condition_activated_and_update_schedules]
            },
            {
                "trigger": AUTOWARE.TRIGGER.SCHEDULE,
                "source": AUTOWARE.STATE.STAND_BY, "dest": AUTOWARE.STATE.SCHEDULE_UPDATED,
                "conditions": [self.condition_expected_schedules_length_and_update_schedules]
            },
            {
                "trigger": AUTOWARE.TRIGGER.SYNCHRONIZE_ROUTE,
                "source": AUTOWARE.STATE.SCHEDULE_UPDATED, "dest": AUTOWARE.STATE.READY_TO_MOVE,
                "conditions": [self.condition_route_synchronized_and_update_schedules]
            },
            {
                "trigger": AUTOWARE.TRIGGER.MOVE,
                "source": AUTOWARE.STATE.READY_TO_MOVE, "dest": AUTOWARE.STATE.MOVE,
                "conditions": [self.condition_decisionmaker_states_changed_to_drive_and_update_schedules]
            },
            {
                "trigger": AUTOWARE.TRIGGER.STOP,
                "source": AUTOWARE.STATE.MOVE, "dest": AUTOWARE.STATE.STOP,
                "conditions": [self.condition_achieved_and_update_schedules]
            },
            {
                "trigger": AUTOWARE.TRIGGER.GET_READY,
                "source": AUTOWARE.STATE.STOP, "dest": AUTOWARE.STATE.STAND_BY,
                "conditions": [self.condition_time_limit_devisionmaker_states_change_to_initial_and_update_schedules]
            },
        ])
        return machine

    def condition_activated(self, decisionmaker_states):
        return self.condition_location() and \
               self.condition_decisionmaker_states_changed_to_initial(decisionmaker_states)

    def condition_location(self):
        return self.status.location is not None

    @staticmethod
    def condition_decisionmaker_states_changed_to_initial(decisionmaker_states):
        if decisionmaker_states is not None:
            if decisionmaker_states.main_state == AUTOWARE.ROS.DECISION_MAKER_STATES.MAIN.INITIAL:
                return True

    @staticmethod
    def condition_expected_schedules_length(schedules, expected):
        return expected == len(schedules)

    condition_time_limit = SimCar.condition_time_limit

    @staticmethod
    def condition_decisionmaker_states_changed_to_mission_complete(decisionmaker_states):
        if decisionmaker_states is not None:
            if all([
                decisionmaker_states.main_state == AUTOWARE.ROS.DECISION_MAKER_STATES.MAIN.MISSION_COMPLETE,
                decisionmaker_states.behavior_state == AUTOWARE.ROS.DECISION_MAKER_STATES.BEHAVIOR.WAIT_ORDERS
            ]):
                return True
        return False

    def condition_route_synchronized(self, route):
        self.publish_lane_array(route)
        return True

    @staticmethod
    def condition_decisionmaker_states_changed_to_drive(decisionmaker_states):
        if decisionmaker_states is not None:
            if decisionmaker_states.main_state == AUTOWARE.ROS.DECISION_MAKER_STATES.MAIN.DRIVE:
                return True
        return False

    def after_state_change_update_schedules(self, current_time, schedules, _decisionmaker_states=None):
        schedules[:] = self.get_next_schedules(schedules, current_time)
        return True

    def after_state_change_publish_drive(self, decisionmaker_states):
        self.publish_drive_state_command(decisionmaker_states)

    def condition_activated_and_update_schedules(self, current_time, schedules):
        if self.condition_location():
            self.after_state_change_update_schedules(current_time, schedules)
            return True
        else:
            if not self.condition_location():
                self.update_pose_from_current_pose()
            return False

    def condition_expected_schedules_length_and_update_schedules(
            self, current_time, schedules, expected_schedules_length):
        if self.condition_expected_schedules_length(schedules, expected_schedules_length):
            self.after_state_change_update_schedules(current_time, schedules)
            return True
        else:
            self.add_random_schedule(current_time, schedules)
            return False

    def condition_route_synchronized_and_update_schedules(self, current_time, schedules):
        if self.condition_route_synchronized(schedules[2].route):
            self.after_state_change_update_schedules(current_time, schedules)
            return True
        else:
            return False

    def condition_decisionmaker_states_changed_to_drive_and_update_schedules(
            self, current_time, schedules, decisionmaker_states):
        if self.condition_decisionmaker_states_changed_to_drive(decisionmaker_states):
            self.after_state_change_update_schedules(current_time, schedules)
            return True
        else:
            self.publish_drive_state_command(decisionmaker_states)
            return False

    def condition_achieved_and_update_schedules(self, current_time, schedules, decisionmaker_states):
        if self.condition_decisionmaker_states_changed_to_mission_complete(decisionmaker_states):
            self.after_state_change_update_schedules(current_time, schedules)
            return True
        return False

    def condition_time_limit_devisionmaker_states_change_to_initial_and_update_schedules(
            self, current_time, schedules, decisionmaker_states):
        if self.condition_time_limit(current_time):
            if True:
                self.after_state_change_update_schedules(current_time, schedules)
                return True
            else:
                self.publish_drive_state_command(decisionmaker_states)
        return False

    def update_status(self):
        self.update_pose_from_closest_arrow_waypoint()

        schedules = self.get_schedules_and_lock()

        self.ros_decisionmaker_states_lock.acquire()
        decisionmaker_states = deepcopy(self.ros_decisionmaker_states)
        self.ros_decisionmaker_states_lock.release()

        current_time = time()
        next_event = schedules[1].event

        # logger.pp({
        #     "state": self.state_machine.state,
        #     "next_event": next_event,
        #     "location": self.status.location,
        #     "decisionmaker_states": decisionmaker_states,
        #     "len(schedules)": len(schedules)})

        if next_event == AUTOWARE.TRIGGER.ACTIVATE:
            self.state_machine.activate(current_time, schedules)
        elif next_event == AUTOWARE.TRIGGER.SCHEDULE:
            self.state_machine.schedule(current_time, schedules, 7)
        elif next_event == AUTOWARE.TRIGGER.SYNCHRONIZE_ROUTE:
            self.state_machine.synchronize_route(current_time, schedules)
        elif next_event == AUTOWARE.TRIGGER.MOVE:
            self.state_machine.move(current_time, schedules, decisionmaker_states)
        elif next_event == AUTOWARE.TRIGGER.STOP:
            self.state_machine.stop(current_time, schedules, decisionmaker_states)
        elif next_event == AUTOWARE.TRIGGER.GET_READY:
            self.state_machine.get_ready(current_time, schedules, decisionmaker_states)

        self.set_schedules_and_unlock(schedules)
