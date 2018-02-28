#!/usr/bin/env python
# coding: utf-8

import rospy
from autoware_msgs.msg import state_cmd as StateCommand
from std_msgs.msg import Int32

from ams import Topic
from ams.nodes import EventLoop, Autoware
from ams.messages import StateCommand as AMSStateCommand


class StateCommandPublisher(EventLoop):
    def __init__(self, name):
        super(StateCommandPublisher, self).__init__()

        self.topicPubStateCommand = Topic()
        self.topicPubStateCommand.set_id(name)
        self.topicPubStateCommand.set_root(Autoware.CONST.TOPIC.PUBLISH)

        self.set_subscriber(self.topicPubStateCommand.private + Autoware.CONST.TOPIC.STATE_COMMAND, self.publish_to_ros)

        rospy.init_node(Autoware.CONST.ROSNODE.AMS_STATE_COMMAND_PUBLISHER, anonymous=True)
        # self.__ROSPublisher = rospy.Publisher(Autoware.CONST.ROSTOPIC.STATE_COMMAND, StateCommand, queue_size=1)
        self.__ROSPublisher = rospy.Publisher(Autoware.CONST.ROSTOPIC.STATE_COMMAND, Int32, queue_size=1)

    def publish_to_ros(self, _client, _userdata, topic, payload):
        if topic == self.topicPubStateCommand.private + Autoware.CONST.TOPIC.STATE_COMMAND:
            ams_state_command = AMSStateCommand.new_data(**self.topicPubStateCommand.unserialize(payload))
            state_command = StateCommand()
            state_command.cmd = ams_state_command.state

            # self.__ROSPublisher.publish(state_command)
            self.__ROSPublisher.publish(ams_state_command.state)

