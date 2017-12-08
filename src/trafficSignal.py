#!/usr/bin/env python
# coding: utf-8

import json
from eventLoop import EventLoop
from time import time, sleep
from const import CONST


class TrafficSignal(EventLoop):
    def __init__(self, intersection):
        super().__init__()
        self.__intersectionID = intersection["intersectionID"]
        self.__signals = self.__getSignals(intersection)
        self.__cycles = None
        self.setOnMessageFunction(
            onMessageFunction=self.__onMessageFunction,
            userData={
                "updateState": self.updateState,
            }
        )
        self.setSubscriber(CONST.TOPICS.TRAFFICSIGNAL_DIRECTIONS)
        self.setMainLoop(self.__mainLoop)

    def __onMessageFunction(self, client, userdata, topic, message):
        if topic == CONST.TOPICS.NOTICE:
            userdata["updateState"](message)

    def __getSignals(self, intersection):
        signals = {}
        for entryExitRouteID in intersection["entryExitRoutes"]:
            signals[entryExitRouteID] = {
                "entryExitRouteID": entryExitRouteID,
                "cycleID": None,
                "schedules": [],
                "state": None,
            }
        return signals

    def setCycles(self, cycles):
        self.__cycles = cycles
        for cycleID, cycle in cycles.items():
            for entryExitRouteID in cycle["entryExitRouteIDs"]:
                self.__signals[entryExitRouteID]["cycleID"] = cycleID
        return True

    def setSchedules(self, schedules):
        for entryExitRouteID in self.__signals:
            self.__signals[entryExitRouteID]["schedules"] = schedules[entryExitRouteID]

    def updateState(self, message):
        # when get new schedules, update schedules and status
        return None

    def getCycleToSchedule(self, cycle, startTime):
        baseTime = cycle["baseTime"]
        period = cycle["period"]
        phaseTime = (startTime - baseTime) % period
        elapseTime = 0
        for phase in cycle["phases"]:
            elapseTime += phase["duration"]
            if phaseTime < elapseTime:
                state = phase["state"]
                endTime = startTime + (elapseTime - phaseTime)
                break
        else:
            state = phase["state"]
            endTime = startTime + (elapseTime - phaseTime)

        schedule = {
            "startTime": startTime,
            "endTime": endTime,
            "state": state
        }
        return schedule

    def __updateSchedule(self, currentTime):
        for entryExitRouteID in self.__signals:
            self.__signals[entryExitRouteID]["schedules"] = list(
                filter(lambda x: currentTime <= x["endTime"], self.__signals[entryExitRouteID]["schedules"]))

            if self.__cycles is not None:
                if len(self.__signals[entryExitRouteID]["schedules"]) < 3:
                    signal = self.__signals[entryExitRouteID]
                    cycle = self.__cycles[signal["cycleID"]]
                    startTime = signal["schedules"][-1]["endTime"]
                    schedule = self.getCycleToSchedule(cycle, startTime)
                    self.__signals[entryExitRouteID]["schedules"].append(schedule)
                print(self.__signals[entryExitRouteID]["schedules"][-1], len(self.__signals[entryExitRouteID]["schedules"][-1]))

    def __mainLoop(self):
        while True:
            sleepTime = 1000
            currentTime = time()
            for entryExitRouteID, signal in self.__signals.items():
                self.__updateSchedule(currentTime)
                if len(signal["schedules"]) == 0:
                    if signal["state"] is not None:
                        signal["state"] = None
                        self.publish(CONST.TOPICS.TRAFFICSIGNAL_STATUS, json.dumps({
                            "intersectionID": self.__intersectionID,
                            "entryExitRouteID": entryExitRouteID,
                            "state": None,
                            "endTime": None,
                        }))
                else:
                    currentSchedule = signal["schedules"][0]
                    state = currentSchedule["state"]
                    endTime = currentSchedule["endTime"]
                    if signal["state"] != state:
                        signal["state"] = state
                        self.publish(CONST.TOPICS.TRAFFICSIGNAL_STATUS, json.dumps({
                            "intersectionID": self.__intersectionID,
                            "entryExitRouteID": entryExitRouteID,
                            "state": state,
                            "endTime": endTime,
                        }))
                    sleepTime = min(sleepTime, endTime - time())

            print("sleep {}[sec]".format(sleepTime))
            sleep(sleepTime)


if __name__ == '__main__':
    from sys import argv
    from intersection import Intersection
    intersection = Intersection()
    intersection.load("../res/intersection.json")

    intersectionID = argv[1]

    # todo: signal grouping
    # todo: signal cycle
    currentTime = int(time())
    cycles = {
        "A": {
            "baseTime": currentTime,
            "period": 124.0,
            "phases": [
                {
                    "state": CONST.TRAFFICSIGNAL_STATE.GREEN,
                    "duration": 70.0,
                },
                {
                    "state": CONST.TRAFFICSIGNAL_STATE.YELLOW,
                    "duration": 5.0,
                },
                {
                    "state": CONST.TRAFFICSIGNAL_STATE.RED,
                    "duration": 49.0,
                },
            ],
        },
        "B": {
            "baseTime": currentTime,
            "period": 124.0,
            "phases": [
                {
                    "state": CONST.TRAFFICSIGNAL_STATE.GREEN,
                    "duration": 80.0,
                },
                {
                    "state": CONST.TRAFFICSIGNAL_STATE.YELLOW,
                    "duration": 5.0,
                },
                {
                    "state": CONST.TRAFFICSIGNAL_STATE.RED,
                    "duration": 39.0,
                },
            ],
        },
        "C": {
            "baseTime": currentTime + 87,
            "period": 124.0,
            "phases": [
                {
                    "state": CONST.TRAFFICSIGNAL_STATE.GREEN,
                    "duration": 30.0,
                },
                {
                    "state": CONST.TRAFFICSIGNAL_STATE.YELLOW,
                    "duration": 5.0,
                },
                {
                    "state": CONST.TRAFFICSIGNAL_STATE.RED,
                    "duration": 89.0,
                },
            ],
        },
    }

    cycleGroups = {
        "A": {
            "groupID": "A",
            "entryExitRouteIDs": [
                "9315/10471/9686",
                "10350/10374/10388/10391",
                "10350/10374/9591/9600",
                "10067/10106/10121",
                "9988/10010/10027/9335",
                "9988/10010/9176/9180/9192",
                "9988/10010/9176/9180/9908",
            ]
        },
        "B": {
            "groupID": "B",
            "entryExitRouteIDs": [
                "9316/9176/9180/9192",
                "9316/9176/9180/9908",
                "9059/9587/9591/9600",
            ]
        },
        "C": {
            "groupID": "C",
            "entryExitRouteIDs": [
                "9143/9151/10388/10391",
                "9143/9151/9155/9159/9176/9180/9192",
                "9143/9151/9155/9159/9176/9180/9908",
                "9553/9563/9568/10027/9335",
                "9887/9563/9568/10027/9335",
                "9553/9563/9568/10106/10121",
                "9887/9563/9568/10106/10121",
                "9553/9563/9568/9572/9587/9591/9600",
                "9887/9563/9568/9572/9587/9591/9600",
                "9143/9151/9155/10106/10121",
                "9143/9151/9155/9159/10027/9335",
                "9553/9563/9568/9572/10578/10471/9686",
                "9887/9563/9568/9572/10578/10471/9686",
                "9553/9563/9568/9572/10578/10391",
                "9887/9563/9568/9572/10578/10391",
            ]
        }
    }

    for cycleID in cycles:
        entryExitRouteIDs = cycles[cycleID].get("entryExitRouteIDs", [])
        entryExitRouteIDs.extend(cycleGroups[cycleID]["entryExitRouteIDs"])
        cycles[cycleID]["entryExitRouteIDs"] = entryExitRouteIDs

    currentTime = int(time())
    schedules = {}
    for entryExitRouteID in intersection.getEntryExitRouteIDs(intersectionID):
        schedules[entryExitRouteID] = [
            {
                "startTime": currentTime,
                "endTime": currentTime + 3,
                "state": "green"
            },
            {
                "startTime": currentTime + 3,
                "endTime": currentTime + 6,
                "state": "yellow"
            },
            {
                "startTime": currentTime + 6,
                "endTime": currentTime + 9,
                "state": "red"
            },
        ]


    trafficSignal = TrafficSignal(intersection=intersection.getIntersection(intersectionID))  # 559
    trafficSignal.setSchedules(schedules)
    trafficSignal.setCycles(cycles)

    # from pprint import PrettyPrinter
    # pp = PrettyPrinter(indent=2).pprint
    # pp(cycles[intersectionID+"_A"])
    # pp(trafficSignal.getCycleToSchedule(cycles[intersectionID+"_A"], currentTime+51))

    print("start processes", trafficSignal.start())



