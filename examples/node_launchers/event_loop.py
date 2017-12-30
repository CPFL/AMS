#!/usr/bin/env python
# coding: utf-8

from src.event_loop import EventLoop


if __name__ == '__main__':
    eventLoop = EventLoop()
    eventLoop.start()
    print("event_loop_id {} on {}".format(eventLoop.event_loop_id, eventLoop.get_pid()))
