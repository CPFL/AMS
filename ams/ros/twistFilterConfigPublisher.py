#!/usr/bin/env python
# coding: utf-8

import rospy
from autoware_msgs.msg import ConfigTwistFilter


if __name__ == '__main__':
    rospy.init_node("ams_twist_filter_config_publisher", anonymous=True)
    rosPublisher = rospy.Publisher('/config/twist_filter', ConfigTwistFilter, queue_size=3)

    configTwistFilter = ConfigTwistFilter()
    configTwistFilter.lateral_accel_limit = 5.0
    configTwistFilter.lowpass_gain_linear_x = 0.0
    configTwistFilter.lowpass_gain_angular_z = 0.0

    rosPublisher.publish(configTwistFilter)
