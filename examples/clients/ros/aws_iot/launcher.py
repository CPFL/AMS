#!/usr/bin/env python
# coding: utf-8

from multiprocessing import Process
from argparse import ArgumentParser
from uuid import uuid4 as uid

import rospy
import AWSIoTPythonSDK.MQTTLib

from ros_mqtt_bridge import AWSIoTToROS, ROSToAWSIoT

from config.env import env
from ams.clients import MapsClient, get_pubsub_client_class
from ams.nodes.ros_bridge import EventLoop as RosBridge


def launch_route_code_to_lane_array_bridge(
        waypoint_json_path, arrow_json_path, ros_node_name, ros_id, bridge_node_name, bridge_node_id, message_type, rospy_rate,
        ca_file_path, key_file_path, certificate_file_path):
    maps_client = MapsClient()
    maps_client.load_waypoint_json_file(waypoint_json_path)
    maps_client.load_arrow_json_file(arrow_json_path)

    AWSIoTCleint = get_pubsub_client_class(AWSIoTPythonSDK.MQTTLib)
    aws_iot_client = AWSIoTCleint()
    aws_iot_client.set_args_of_AWSIoTMQTTClient(str(uid()))
    aws_iot_client.set_args_of_configureEndpoint(env["AWS_IOT_ENDPOINT"], 8883)
    aws_iot_client.set_args_of_configureCredentials(
        CAFilePath=ca_file_path, KeyPath=key_file_path, CertificatePath=certificate_file_path)
    aws_iot_client.set_args_of_connect()

    ROSClient = get_pubsub_client_class(rospy)
    ros_client = ROSClient()

    ros_bridge = RosBridge(group=bridge_node_name, _id=bridge_node_id)
    ros_bridge.set_maps_client(maps_client)
    ros_bridge.set_pubsub_clients(aws_iot_client, ros_client)
    ros_bridge.start()


def launch_ros_to_aws_iot_bridge(
        ros_node_name, from_topic, to_topic, message_type, rospy_rate,
        ca_file_path, key_file_path, certificate_file_path
):
    ros_to_aws_iot = ROSToAWSIoT(from_topic, to_topic, message_type)
    ros_to_aws_iot.set_aws_iot_AWSIoTMQTTClient(clientID=str(uid()))
    ros_to_aws_iot.set_aws_iot_configureEndpoint(env["AWS_IOT_ENDPOINT"], 8883)
    ros_to_aws_iot.set_aws_iot_configureCredentials(ca_file_path, key_file_path, certificate_file_path)
    ros_to_aws_iot.set_aws_iot_connect()
    ros_to_aws_iot.set_ros_init_node_args(name=ros_node_name)
    ros_to_aws_iot.set_ros_rate_args(hz=rospy_rate)
    print("start ros_to_aws_iot_bridge {} -> {}.".format(from_topic, to_topic))
    ros_to_aws_iot.start()


def launch_aws_iot_to_ros_bridge(
        ros_node_name, from_topic, to_topic, message_type, rospy_rate,
        ca_file_path, key_file_path, certificate_file_path
):
    aws_iot_to_ros = AWSIoTToROS(from_topic, to_topic, message_type)
    aws_iot_to_ros.set_aws_iot_AWSIoTMQTTClient(clientID=str(uid()))
    aws_iot_to_ros.set_aws_iot_configureEndpoint(env["AWS_IOT_ENDPOINT"], 8883)
    aws_iot_to_ros.set_aws_iot_configureCredentials(ca_file_path, key_file_path, certificate_file_path)
    aws_iot_to_ros.set_aws_iot_connect()
    aws_iot_to_ros.set_ros_init_node_args(name=ros_node_name)
    aws_iot_to_ros.set_ros_rate_args(hz=rospy_rate)
    print("start mqtt_to_ros_bridge {} -> {}.".format(from_topic, to_topic))
    aws_iot_to_ros.start()


if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument("-RNN", "--ros_node_name", type=str, required=True, help="ros node name")
    parser.add_argument("-RID", "--ros_id", type=str, required=True, help="ros id")
    parser.add_argument("-BNN", "--bridge_node_name", type=str, required=True, help="ros_bridge node name")
    parser.add_argument("-BID", "--bridge_node_id", type=str, required=True, help="ros_bridge node id")
    parser.add_argument("-AID", "--autoware_id", type=str, required=True, help="autoware id")
    parser.add_argument("-TD", "--topic_domain", type=str, default="ams", help="topic domain")

    parser.add_argument("-CAP", "--ca_file_path", type=str, default=None, help="./secrets/root-ca.crt")
    parser.add_argument("-KP", "--key_file_path", type=str, default="", help="./secrets/private.key")
    parser.add_argument("-CP", "--certificate_file_path", type=str, default="", help="./secrets/cert.pem")
    args = parser.parse_args()

    ros_to_ams_base_topic = "/".join(["", args.topic_domain, "ros", args.ros_id, args.ros_node_name, args.autoware_id])
    ams_to_ros_base_topic = "/".join(["", args.topic_domain, args.ros_node_name, args.autoware_id, "ros", args.ros_id])
    rospy_rate = 1
    process_current_pose_ros_to_mqtt = Process(target=launch_ros_to_aws_iot_bridge, args=[
        "ros_to_ams_current_pose",
        "/current_pose", ros_to_ams_base_topic + "/current_pose",
        "geometry_msgs/PoseStamped",
        rospy_rate,
        args.ca_file_path, args.key_file_path, args.certificate_file_path
    ])
    process_closest_waypoint_ros_to_mqtt = Process(target=launch_ros_to_aws_iot_bridge, args=[
        "ros_to_ams_closest_waypoint",
        "/closest_waypoint", ros_to_ams_base_topic + "/closest_waypoint",
        "std_msgs/Int32",
        rospy_rate,
        args.ca_file_path, args.key_file_path, args.certificate_file_path
    ])
    process_decision_maker_state_ros_to_mqtt = Process(target=launch_ros_to_aws_iot_bridge, args=[
        "ros_to_ams_decision_maker_state",
        "/decision_maker/state", ros_to_ams_base_topic + "/decision_maker/state",
        "std_msgs/String",
        rospy_rate,
        args.ca_file_path, args.key_file_path, args.certificate_file_path
    ])
    # process_based_lane_waypoints_array_mqtt_to_ros = Process(target=launch_aws_iot_to_ros_bridge, args=[
    #     "ams_to_ros_based_lane_waypoints_array",
    #     ams_to_ros_base_topic + "/based/lane_waypoints_array", "/based/lane_waypoints_raw",
    #     "autoware_msgs/LaneArray",
    #     rospy_rate,
    #     args.ca_file_path, args.key_file_path, args.certificate_file_path
    # ])
    process_state_cmd_mqtt_to_ros = Process(target=launch_aws_iot_to_ros_bridge, args=[
        "ams_to_ros_state_cmd",
         ams_to_ros_base_topic + "/state_cmd", "/state_cmd",
        "std_msgs/String",
        rospy_rate,
        args.ca_file_path, args.key_file_path, args.certificate_file_path
    ])
    process_light_color_mqtt_to_ros = Process(target=launch_aws_iot_to_ros_bridge, args=[
        "ams_to_ros_light_color",
        ams_to_ros_base_topic + "/light_color", "/light_color",
        "autoware_msgs/traffic_light",
        rospy_rate,
        args.ca_file_path, args.key_file_path, args.certificate_file_path
    ])

    try:
        process_current_pose_ros_to_mqtt.start()
        process_closest_waypoint_ros_to_mqtt.start()
        process_decision_maker_state_ros_to_mqtt.start()
        # process_based_lane_waypoints_array_mqtt_to_ros.start()
        process_state_cmd_mqtt_to_ros.start()
        process_light_color_mqtt_to_ros.start()

    except KeyboardInterrupt:
        process_current_pose_ros_to_mqtt.terminate()
        process_closest_waypoint_ros_to_mqtt.terminate()
        process_decision_maker_state_ros_to_mqtt.terminate()
        # process_based_lane_waypoints_array_mqtt_to_ros.terminate()
        process_state_cmd_mqtt_to_ros.terminate()
        process_light_color_mqtt_to_ros.terminate()
