#!/usr/bin/env python
# coding: utf-8

from time import time, sleep
from copy import deepcopy

from ams import logger, Topic, Schedule
from ams.nodes import EventLoop
from ams.messages import UserStatus
from ams.structures import Schedules, USER, FLEET_MANAGER


class User(EventLoop):

    CONST = USER

    def __init__(self, _id, name, dt=1.0):
        super().__init__(_id)

        self.status = UserStatus.new_data(
            name=name,
            time=time(),
            trip_schedules=None,
            state=USER.STATE.LOG_IN,
            schedule=None
        )

        self.state_machine = None
        self.dt = dt

        self.schedules = self.manager.list()
        self.schedules_lock = self.manager.Lock()

        self.__topicPubStatus = Topic()
        self.__topicPubStatus.set_targets(self.target)
        self.__topicPubStatus.set_categories(USER.TOPIC.CATEGORIES.STATUS)

        self.__topicSubSchedules = Topic()
        self.__topicSubSchedules.set_targets(None, self.target)
        self.__topicSubSchedules.set_categories(FLEET_MANAGER.TOPIC.CATEGORIES.SCHEDULES)
        self.__topicSubSchedules.set_message(Schedules)
        self.set_subscriber(self.__topicSubSchedules, self.update_schedules)

        self.set_main_loop(self.__main_loop)

    def set_trip_schedules(self, trip_schedules):
        self.status.trip_schedules = trip_schedules
        self.set_schedules([Schedule.new_schedule(
            targets=[self.target],
            event=USER.TRIGGER.LOG_IN,
            start_time=trip_schedules[0].period.start,
            end_time=trip_schedules[0].period.end
        )])

    def set_schedules(self, schedules):
        self.schedules_lock.acquire()
        self.schedules[:] = schedules
        self.status.schedule = deepcopy(self.schedules[0])
        self.schedules_lock.release()

    def publish_status(self):
        self.status.time = time()
        self.status.state = self.state_machine.state
        payload = self.__topicPubStatus.serialize(self.status)
        self.publish(self.__topicPubStatus, payload)

    def update_status_schedule(self):
        pass

    def update_schedules(self, _client, _userdata, _topic, payload):
        new_schedules = self.__topicSubSchedules.unserialize(payload)
        # logger.pp(new_schedules)

        self.schedules_lock.acquire()
        index = list(map(lambda x: x.id, new_schedules)).index(self.schedules[0].id)
        self.schedules[:] = new_schedules[index:]
        self.update_status_schedule()
        self.schedules_lock.release()

    def get_next_schedules(self, schedules, current_time):
        schedules.pop(0)
        dif_time = current_time - schedules[0].period.start
        schedules = Schedule.get_shifted_schedules(schedules, dif_time)
        self.status.schedule = schedules[0]
        return schedules

    def condition_time_limit(self, current_time, _schedules):
        return self.status.schedule.period.end < current_time

    def after_state_change_update_schedules(self, current_time, schedules):
        schedules[:] = self.get_next_schedules(schedules, current_time)
        return True

    def after_state_change_update_time_limit(self, current_time, duration):
        self.status.schedule.period.start = current_time
        self.status.schedule.period.end = current_time + duration
        return True

    def condition_time_limit_and_update_schedules(self, current_time, schedules):
        if self.condition_time_limit(current_time, schedules):
            self.after_state_change_update_schedules(current_time, schedules)
            return True
        return False

    def get_schedules_and_lock(self):
        self.schedules_lock.acquire()
        return deepcopy(self.schedules)

    def set_schedules_and_unlock(self, schedules):
        self.schedules[:] = schedules
        self.schedules_lock.release()

    def update_status(self):
        schedules = self.get_schedules_and_lock()

        self.set_schedules_and_unlock(schedules)
        return

    def __main_loop(self):

        while self.status.state != USER.STATE.LOG_OUT:
            sleep(self.dt)
            self.update_status()
            self.publish_status()

        return True
