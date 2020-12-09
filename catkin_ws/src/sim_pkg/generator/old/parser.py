from lxml import etree
from commonroad import CommonRoad, Point, Lanelet, Obstacle, Rectangle

class Parser:
    def __init__(self):
        pass

    def parse_rectangle(self, node):
        return Rectangle(
            length=float(node.find("length").text),
            width=float(node.find(""))
        )

    def parse_shape(self, node):


    def parse(self, file):
        tree = etree.parse(file)
        road = CommonRoad()

        for child in tree.getroot():
            id = int(child.get("id"))
            road_element = None

            if child.tag == "lanelet":
                left = []
                right = []
                for point in child.find("leftBoundary").findall("point"):
                    left.append(Point(float(point.find("x").text), float(point.find("y").text)))
                for point in child.find("rightBoundary").findall("point"):
                    right.append(Point(float(point.find("x").text), float(point.find("y").text)))

                left_line_marking = None
                right_line_marking = None
                if child.find("leftBoundary").find("lineMarking") is not None:
                    left_line_marking = child.find("leftBoundary").find("lineMarking").text
                if child.find("rightBoundary").find("lineMarking") is not None:
                    right_line_marking = child.find("rightBoundary").find("lineMarking").text
                road_element = Lanelet(left, right, left_line_marking, right_line_marking)
            elif child.tag == "obstacle":
                road_element = Obstacle(
                    role=child.find("role").text,
                    type=child.find("type").text)

            if road_element is not None:
                road.add(id, road_element)
        return road
