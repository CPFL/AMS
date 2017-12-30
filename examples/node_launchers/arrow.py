#!/usr/bin/env python
# coding: utf-8


if __name__ == '__main__':
    from pprint import PrettyPrinter
    pp = PrettyPrinter(indent=2).pprint

    arrow = Arrow()
    arrow.load("./res/arrow.json")
    # pp(arrow.getArrows())

    # pp(arrows.getPointToArrows("35.647441", "139.772060"))

    # pp(arrows.getPointToArrows(35.65121944507907, 139.77862046921132))
    #
    # pp(arrows.getShortestRoutes(
    #     35.6504049, 139.7772154,
    #     [{'lat': 35.650421105409954,
    #     'lng': 139.7772497249228,
    #     'vehicleID': 'vehicle1'}],
    #     reverse=True))
