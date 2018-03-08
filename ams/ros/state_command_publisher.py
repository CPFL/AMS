#!/usr/bin/env python
# coding: utf-8

import rospy
from autoware_msgs.msg import state_cmd as AWStateCommand
from std_msgs.msg import Int32

from ams import Topic, Target
from ams.nodes import EventLoop
from ams.messages import StateCommand
from ams.structures import STATE_COMMAND_PUBLISHER, AUTOWARE


class StateCommandPublisher(EventLoop):
    
    CONST = STATE_COMMAND_PUBLISHER

    def __init__(self, _id):
        super(StateCommandPublisher, self).__init__(_id)

        self.__topicSubStateCommand = Topic()
        self.__topicSubStateCommand.set_targets(
            Target.new_target(self.target.id, AUTOWARE.NODE_NAME), self.target
        )
        self.__topicSubStateCommand.set_categories(AUTOWARE.TOPIC.CATEGORIES.STATE_COMMAND)
        self.__topicSubStateCommand.set_message(StateCommand)
        self.set_subscriber(self.__topicSubStateCommand, self.publish_to_ros)

        rospy.init_node(STATE_COMMAND_PUBLISHER.ROSNODE, anonymous=True)
        # self.__ROSPublisher = rospy.Publisher(STATE_COMMAND_PUBLISHER.ROSTOPIC, AWStateCommand, queue_size=1)
        self.__ROSPublisher = rospy.Publisher(STATE_COMMAND_PUBLISHER.ROSTOPIC, Int32, queue_size=1)

    def publish_to_ros(self, _client, _userdata, _topic, payload):
        state_command = self.__topicSubStateCommand.unserialize(payload)
        # aw_state_command = AWStateCommand()
        # aw_state_command.cmd = state_command.state
        # self.__ROSPublisher.publish(aw_state_command)
        self.__ROSPublisher.publish(state_command.state)
