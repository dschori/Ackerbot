from functools import reduce

class Point:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def __repr__(self):
        return "Point({0}, {1})".format(self.x, self.y)

class Rectangle:
    def __init__(self, length=0, width=0, orientation=0, center_point=None):
        self.length = length
        self.width = width
        self.orientation = orientation
        self.center_point = center_point

    def __repr__(self):
        return "Rectangle(l={0}, w={1}, o={2}, cp={3})".format(self.length, self.width,
            self.orientation, self.center_point)

class Lanelet:
    def __init__(self, left_boundary, right_boundary, left_boundary_marking, right_boundary_marking):
        self.left_boundary = left_boundary
        self.right_boundary = right_boundary
        self.left_boundary_marking = left_boundary_marking
        self.right_boundary_marking = right_boundary_marking

    def get_bounding_box(self):
        all_x = list(map(lambda p: p.x, self.left_boundary)) + list(map(lambda p: p.x, self.right_boundary))
        all_y = list(map(lambda p: p.y, self.left_boundary)) + list(map(lambda p: p.y, self.right_boundary))
        return BoundingBox(min(all_x), min(all_y), max(all_x), max(all_y))

    def __repr__(self):
        return "Lanelet({0}, {1})".format(self.left_boundary,
            self.right_boundary)

class Obstacle:
    def __init__(self, role="", type=""):
        self.role = role
        self.type = type
        self.shape = []

class CommonRoad:
    def __init__(self):
        self._elements = {}
        self._lanelets = []

    def add(self, id, element):
        if not isinstance(id, int):
            raise ValueError("id is not an integer: {0}".format(id))
        if isinstance(element, Lanelet):
            self._elements[id] = element
            self._lanelets.append(element)

    def get(self, id):
        return self._elements[id]

    def get_lanelets(self):
        return self._lanelets

    def get_bounding_box(self):
        return reduce(lambda x,y: x.union(y), map(lambda key: self._elements[key].get_bounding_box(), self._elements))
