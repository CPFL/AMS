#!/usr/bin/env python
# coding: utf-8


if __name__ == '__main__':
    from pprint import PrettyPrinter
    pp = PrettyPrinter(indent=2).pprint

    intersection = Intersection()
    intersection.load("./res/intersection.json")

    intersectionIDs = intersection.get_intersection_ids()
    intersections = intersection.get_intersections()
    pp(list(map(lambda x: (intersections[x]["lat"], intersections[x]["lng"]), intersectionIDs)))

    # with open("./intersectionsLatLng.json", "w") as f:
    #     json.dump(list(map(
    #       lambda x: (intersections[x]["lat"], intersections[x]["lng"]), intersectionIDs)), f, indent="  ")
