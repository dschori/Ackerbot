#!/usr/bin/env python3
import sys, argparse
from commonroad import schema
from commonroad.generator import road_generation, preset_parser
import pkg_resources
from lxml import etree

SCHEMA = etree.XMLSchema(etree.parse(pkg_resources.resource_stream(
    "commonroad.generator", "template-schema.xsd")))

class Config:
    def __init__(self):
        self.road_width = 0.4

def main():
    parser = argparse.ArgumentParser(
        description="Generate a randomized CommonRoad XML from a preset file")
    parser.add_argument("input", nargs="?", type=argparse.FileType("r"),
        default=sys.stdin)
    parser.add_argument("--output", "-o", type=argparse.FileType("w"),
        default=sys.stdout)
    args = parser.parse_args()

    parser = etree.XMLParser(schema=SCHEMA)
    root = etree.parse(args.input, parser)

    primitives = road_generation.generate(root)

    doc = schema.commonRoad()
    doc.commonRoadVersion = "1.0"
    #doc.append(ego_vehicle())
    id = 0
    lanelet_pairs = []
    for p in primitives:
        export = p.export(Config())
        lanelet_pairs += export.lanelet_pairs
        for obj in export.objects:
            id -= 1
            obj.id = id
            doc.append(obj)

    # adjacents
    for pair in lanelet_pairs:
        pair[0].adjacentLeft = schema.laneletAdjacentRef(ref=pair[1].id, drivingDir="opposite")
        pair[1].adjacentLeft = schema.laneletAdjacentRef(ref=pair[0].id, drivingDir="opposite")
        pair[0].successor = schema.laneletRefList()
        pair[0].predecessor = schema.laneletRefList()
        pair[1].successor = schema.laneletRefList()
        pair[1].predecessor = schema.laneletRefList()

    # right lanes
    for i in range(len(lanelet_pairs)-1):
        lanelet_pairs[i][0].successor.lanelet.append(schema.laneletRef(ref=lanelet_pairs[i+1][0].id))
        lanelet_pairs[i+1][0].predecessor.lanelet.append(schema.laneletRef(ref=lanelet_pairs[i][0].id))

    # left lanes
    for i in range(len(lanelet_pairs)-1, 0, -1):
        lanelet_pairs[i][1].successor.lanelet.append(schema.laneletRef(ref=lanelet_pairs[i-1][1].id))
        lanelet_pairs[i-1][1].predecessor.lanelet.append(schema.laneletRef(ref=lanelet_pairs[i][1].id))

    with args.output as file:
        file.write(doc.toxml())

def ego_vehicle():
    shape = schema.shape()
    shape.rectangle.append(schema.rectangle(length=0.4, width=0.4,
        orientation=0, centerPoint=schema.point(x=0, y=0)))
    state = schema.state(orientation=schema.floatInterval(exact=0),
        time=schema.floatInterval(exact=0))
    state.position = schema.point(x=0, y=0)
    return schema.egoVehicle(id=0, type="car", shape=shape, initialState=state)

if __name__ == "__main__":
    main()
