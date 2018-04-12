#!/usr/bin/env python
# coding: utf-8

from multiprocessing import Process
from argparse import ArgumentParser

from ros_mqtt_bridge import MQTTToROS, ROSToMQTT
from config.env import env

parser = ArgumentParser()
parser.add_argument("-NN", "--node_name", type=str, required=True, help="node name")
parser.add_argument("-ID", "--id", type=str, required=True, help="node id")
args = parser.parse_args()


def launch_ros_to_mqtt_bridge(node_name, from_topic, to_topic, message_module_name, message_class_name, rospy_rate=1):
    ros_to_mqtt = ROSToMQTT(
        env["MQTT_BROKER_HOST"], int(env["MQTT_BROKER_PORT"]),
        from_topic, to_topic, message_module_name, message_class_name,
        node_name=node_name, rospy_rate=rospy_rate
    )
    print("start ros_to_mqtt_bridge {} -> {}.".format(from_topic, to_topic))
    ros_to_mqtt.start()


def launch_mqtt_to_ros_bridge(node_name, from_topic, to_topic, message_module_name, message_class_name, rospy_rate=1):
    mqtt_to_ros = MQTTToROS(
        env["MQTT_BROKER_HOST"], int(env["MQTT_BROKER_PORT"]),
        from_topic, to_topic, message_module_name, message_class_name,
        node_name=node_name, rospy_rate=rospy_rate
    )
    print("start mqtt_to_ros_bridge {} -> {}.".format(from_topic, to_topic))
    mqtt_to_ros.start()


if __name__ == '__main__':

    ros_to_ams_base_topic = "/ams/ros/" + args.id + "/" + args.node_name + "/" + args.id
    ams_to_ros_base_topic = "/ams/" + args.node_name + "/" + args.id + "/ros/" + args.id
    process_current_pose_ros_to_mqtt = Process(target=launch_ros_to_mqtt_bridge, args=[
        "ros_to_ams_current_pose",
        "/current_pose", ros_to_ams_base_topic + "/current_pose",
        "geometry_msgs.msg", "msg.PoseStamped"
    ])
    process_closest_waypoint_ros_to_mqtt = Process(target=launch_ros_to_mqtt_bridge, args=[
        "ros_to_ams_closest_waypoint",
        "/closest_waypoint", ros_to_ams_base_topic + "/closest_waypoint",
        "std_msgs.msg", "msg.Int32"
    ])
    process_decision_maker_states_ros_to_mqtt = Process(target=launch_ros_to_mqtt_bridge, args=[
        "ros_to_ams_decision_maker_states",
        "/decisionmaker/states", ros_to_ams_base_topic + "/decisionmaker/states",
        "autoware_msgs.msg", "msg.state"
    ])
    process_based_lane_waypoints_array_mqtt_to_ros = Process(target=launch_mqtt_to_ros_bridge, args=[
        "ams_to_ros_based_lane_waypoints_array",
        ams_to_ros_base_topic + "/based/lane_waypoints_array", "/based/lane_waypoints_array",
        "autoware_msgs.msg", "msg.LaneArray"
    ])
    process_state_cmd_mqtt_to_ros = Process(target=launch_mqtt_to_ros_bridge, args=[
        "ams_to_ros_state_cmd",
         ams_to_ros_base_topic + "/state_cmd", "/state_cmd",
        "std_msgs.msg", "msg.Int32"
    ])
    process_light_color_mqtt_to_ros = Process(target=launch_mqtt_to_ros_bridge, args=[
        "ams_to_ros_light_color",
        ams_to_ros_base_topic + "/light_color", "/light_color",
        "autoware_msgs.msg", "msg.traffic_light"
    ])

    try:
        process_current_pose_ros_to_mqtt.start()
        process_closest_waypoint_ros_to_mqtt.start()
        process_decision_maker_states_ros_to_mqtt.start()
        process_based_lane_waypoints_array_mqtt_to_ros.start()
        process_state_cmd_mqtt_to_ros.start()
        process_light_color_mqtt_to_ros.start()

    except KeyboardInterrupt:
        process_current_pose_ros_to_mqtt.terminate()
        process_closest_waypoint_ros_to_mqtt.terminate()
        process_decision_maker_states_ros_to_mqtt.terminate()
        process_based_lane_waypoints_array_mqtt_to_ros.terminate()
        process_state_cmd_mqtt_to_ros.terminate()
        process_light_color_mqtt_to_ros.terminate()
