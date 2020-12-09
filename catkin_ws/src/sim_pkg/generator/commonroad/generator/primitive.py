import numpy as np
import math
from shapely.geometry import LineString, CAP_STYLE, JOIN_STYLE
import scipy.integrate as integrate
from commonroad import schema

class MissingPointsException(Exception):
    pass

def circle_from_points(x1, y1, x2, y2, x3, y3):
    s1 = np.array([[y2 - y1], [- (x2 - x1)]])
    s2 = np.array([[y3 - y2], [- (x3 - x2)]])
    mid1 = 0.5*np.array([[x1 + x2], [y1 + y2]])
    mid2 = 0.5*np.array([[x2 + x3], [y2 + y3]])
    b = mid2 - mid1
    A = np.hstack((s1, s2))
    if np.linalg.matrix_rank(A) == 2 :
        result = np.linalg.solve(A, b)
        circle_mid = mid1 + result[0] * s1
        radius = np.linalg.norm(circle_mid - [[x1], [y1]])
        return (circle_mid, radius)
    else:
        return None

def is_left(a, b, c):
    x = b - a
    y = c - a
    return np.cross(x, y) > 0

def convert_line_marking(marking):
    if marking is None or marking == "missing":
        return None
    else:
        return marking

class Export:
    def __init__(self, objects, lanelet_pairs):
        self.objects = objects
        self.lanelet_pairs = lanelet_pairs

class Primitive:
    def get_points(self):
        return []

    def get_bounding_box(self, street_width):
        points = self.get_points()
        if len(points) == 0:
            raise MissingPointsException("get_points() returned empty array")
        line = LineString(points)
        polygon = line.buffer(street_width, cap_style=CAP_STYLE.flat, join_style=JOIN_STYLE.round)
        return polygon

    def get_beginning(self):
        points = self.get_points()
        p1 = np.array(points[0])
        p2 = np.array(points[1])
        dir = p1 - p2
        circle_mid, radius = circle_from_points(points[0][0], points[0][1],
            points[1][0], points[1][1], points[2][0], points[2][1])
        if not is_left(np.array(points[1]), np.array(points[0]), circle_mid.reshape(2)):
            radius = - radius # rechtskrümmung
        return (p1, math.atan2(dir[1], dir[0]), 1 / radius)

    def get_ending(self):
        points = self.get_points()
        p1 = np.array(points[-1])
        p2 = np.array(points[-2])
        dir = p1 - p2
        circle_mid, radius = circle_from_points(points[-1][0], points[-1][1],
            points[-2][0], points[-2][1], points[-3][0], points[-3][1])
        if not is_left(np.array(points[-2]), np.array(points[-1]), circle_mid.reshape(2)):
            radius = - radius # rechtskrümmung
        return (p1, math.atan2(dir[1], dir[0]), 1 / radius)

    def export(self, config):
        points = self.get_points()
        lanelet1 = schema.lanelet(leftBoundary=schema.boundary(), rightBoundary=schema.boundary())
        lanelet2 = schema.lanelet(leftBoundary=schema.boundary(), rightBoundary=schema.boundary())
        if hasattr(self, "_is_start") and self._is_start:
            lanelet1.isStart = True

        lanelet1.rightBoundary.lineMarking = convert_line_marking(self._right_line if hasattr(self, "_right_line") else None)
        lanelet1.leftBoundary.lineMarking = convert_line_marking(self._middle_line if hasattr(self, "_middle_line") else None)
        lanelet2.rightBoundary.lineMarking = convert_line_marking(self._left_line if hasattr(self, "_left_line") else None)

        for i in range(len(points)):
            if i != len(points) - 1:
                p1 = np.array(points[i])
                p2 = np.array(points[i+1])
                ortho_left = np.array([-(p2[1] - p1[1]), p2[0] - p1[0]])
                ortho_left = ortho_left / np.linalg.norm(ortho_left) * config.road_width
                ortho_right = ortho_left * (-1)
            else:
                p1 = np.array(points[i])

            left = p1 + ortho_left
            right = p1 + ortho_right

            lanelet1.leftBoundary.point.append(
                schema.point(x=points[i][0], y=points[i][1]))
            lanelet1.rightBoundary.point.append(
                schema.point(x=right[0], y=right[1]))
            lanelet2.leftBoundary.point.append(
                schema.point(x=points[i][0], y=points[i][1]))
            lanelet2.rightBoundary.point.append(
                schema.point(x=left[0], y=left[1]))
        # TODO add last point

        # reverse boundary of left lanelet to match driving direction
        lanelet2.leftBoundary.point.reverse()
        lanelet2.rightBoundary.point.reverse()

        return Export([lanelet1, lanelet2], [(lanelet1, lanelet2)])

class TransrotPrimitive(Primitive):
    def __init__(self, child, translation, angle):
        self._child = child
        self._angle = angle
        self._translation = translation

    def __repr__(self):
        return "TransrotPrimitive(translation={}, angle={}, child={})".format(
            self._translation, self._angle, self._child)

    def _get_matrix(self):
        cos = math.cos(self._angle)
        sin = math.sin(self._angle)
        begin = self._child.get_beginning()
        return np.array([
            [cos, -sin, self._translation[0] + begin[0][0]],
            [sin, cos, self._translation[1] + begin[0][1]],
            [0, 0, 1]
        ]).dot(np.array([
            [1, 0, -begin[0][0]],
            [0, 1, -begin[0][1]],
            [0, 0, 1]
        ]))

    def _transform_point(self, point):
        return (self._get_matrix().dot(np.append(point, 1)))[0:2]

    def get_points(self):
        return list(map(self._transform_point, self._child.get_points()))

    def get_beginning(self):
        begin = self._child.get_beginning()
        return (self._transform_point(begin[0]), begin[1] + self._angle, begin[2])

    def get_ending(self):
        end = self._child.get_ending()
        return (self._transform_point(end[0]), end[1] + self._angle, end[2])

    def export(self, config):
        export = self._child.export(config)
        objects = export.objects

        for obj in objects:
            if isinstance(obj, schema.lanelet):
                for i in range(len(obj.leftBoundary.point)):
                    x = obj.leftBoundary.point[i].x
                    y = obj.leftBoundary.point[i].y
                    transformed = self._transform_point([x, y])
                    obj.leftBoundary.point[i].x = transformed[0]
                    obj.leftBoundary.point[i].y = transformed[1]
                for i in range(len(obj.rightBoundary.point)):
                    x = obj.rightBoundary.point[i].x
                    y = obj.rightBoundary.point[i].y
                    transformed = self._transform_point([x, y])
                    obj.rightBoundary.point[i].x = transformed[0]
                    obj.rightBoundary.point[i].y = transformed[1]
            elif isinstance(obj, schema.obstacle):
                for rect in obj.shape.rectangle:
                    x = rect.centerPoint.x
                    y = rect.centerPoint.y
                    transformed = self._transform_point([x, y])
                    rect.orientation -= self._angle
                    rect.centerPoint = schema.point(x=transformed[0], y=transformed[1])
            elif isinstance(obj, schema.trafficSign):
                x = obj.centerPoint.x
                y = obj.centerPoint.y
                transformed = self._transform_point([x, y])
                obj.orientation += self._angle
                obj.centerPoint = schema.point(x=transformed[0], y=transformed[1])

        return export

class StraightLine(Primitive):
    def __init__(self, args):
        self._length = float(args["length"])
        self._left_line = args.get("leftLine", "solid")
        self._middle_line = args.get("middleLine", "dashed")
        self._right_line = args.get("rightLine", "solid")
        self._is_start = args.get("isStart", False)

    def __repr__(self):
        return "StraightLine(length={})".format(self._length)

    def get_points(self):
        return [[0, 0], [self._length, 0]]

    def get_beginning(self):
        return (np.array([0, 0]), math.pi, 0)

    def get_ending(self):
        return (np.array([self._length, 0]), 0, 0)

class LeftCircularArc(Primitive):
    def __init__(self, args):
        self._radius = float(args["radius"])
        self._angle = math.radians(float(args["angle"]))
        self._left_line = args.get("leftLine", "solid")
        self._middle_line = args.get("middleLine", "dashed")
        self._right_line = args.get("rightLine", "solid")

    def __repr__(self):
        return "LeftCircularArc(radius={}, angle={})".format(self._radius, self._angle)

    def get_points(self):
        points = []
        current_angle = 0
        while current_angle <= self._angle:
            points.append([
                math.cos(current_angle - math.pi/2) * self._radius,
                self._radius + math.sin(current_angle - math.pi/2) * self._radius
            ])
            current_angle += 0.01 # TODO what else
        return points

    def get_beginning(self):
        return (np.array([0, 0]), math.pi, 1 / self._radius)

    def get_ending(self):
        return (np.array([
            math.cos(self._angle - math.pi/2) * self._radius,
            self._radius + math.sin(self._angle - math.pi/2) * self._radius
        ]), self._angle, - 1 / self._radius)

class RightCircularArc(Primitive):
    def __init__(self, args):
        self._radius = float(args["radius"])
        self._angle = math.radians(float(args["angle"]))
        self._left_line = args.get("leftLine", "solid")
        self._middle_line = args.get("middleLine", "dashed")
        self._right_line = args.get("rightLine", "solid")

    def __repr__(self):
        return "RightCircularArc(radius={}, angle={})".format(self._radius, self._angle)

    def get_points(self):
        points = []
        current_angle = 0
        while current_angle <= self._angle:
            points.append([
                math.cos(math.pi/2 - current_angle) * self._radius,
                - self._radius + math.sin(math.pi/2 - current_angle) * self._radius
            ])
            current_angle += 0.01 # TODO what else
        return points

    def get_beginning(self):
        return (np.array([0, 0]), math.pi, - 1 / self._radius)

    def get_ending(self):
        return (np.array([
            math.cos(math.pi/2 - self._angle) * self._radius,
            - self._radius + math.sin(math.pi/2 - self._angle) * self._radius
        ]), - self._angle, 1 / self._radius)

class QuadBezier(Primitive):
    def __init__(self, args):
        self._p0 = np.array([0, 0])
        self._p1 = np.array([float(args["p1x"]), float(args["p1y"])])
        self._p2 = np.array([float(args["p2x"]), float(args["p2y"])])
        self._left_line = args.get("leftLine", "solid")
        self._middle_line = args.get("middleLine", "dashed")
        self._right_line = args.get("rightLine", "solid")

        self._points = []
        t = 0
        while t <= 1:
            c0 = (1-t) * self._p0 + t * self._p1
            c1 = (1-t) * self._p1 + t * self._p2
            x = (1-t) * c0 + t * c1
            self._points.append(x)
            t += 0.01

    def get_points(self):
        return self._points

class CubicBezier(Primitive):
    def __init__(self, args):
        self._p0 = np.array([0, 0])
        self._p1 = np.array([float(args["p1x"]), float(args["p1y"])])
        self._p2 = np.array([float(args["p2x"]), float(args["p2y"])])
        self._p3 = np.array([float(args["p3x"]), float(args["p3y"])])
        self._left_line = args.get("leftLine", "solid")
        self._middle_line = args.get("middleLine", "dashed")
        self._right_line = args.get("rightLine", "solid")

        self._points = []
        t = 0
        while t <= 1:
            c0 = (1-t) * self._p0 + t * self._p1
            c1 = (1-t) * self._p1 + t * self._p2
            c2 = (1-t) * self._p2 + t * self._p3
            d0 = (1-t) * c0 + t * c1
            d1 = (1-t) * c1 + t * c2
            x = (1-t) * d0 + t * d1
            self._points.append(x)
            t += 0.01

    def get_points(self):
        return self._points

def euler_spiral(l, A):
    factor = A * math.sqrt(math.pi)
    return [factor * integrate.quad(lambda t: math.cos(math.pi * t * t / 2), 0, l)[0],
        factor * integrate.quad(lambda t: math.sin(math.pi * t * t / 2), 0, l)[0]]

class Clothoid(Primitive):
    def __init__(self, curvature_begin, curvature_end, a):
        self._curv_begin = curvature_begin
        self._curv_end = curvature_end
        self._a = a # clothoid parameter A

        len_begin = math.fabs(curvature_begin) * a / math.sqrt(math.pi)
        len_end = math.fabs(curvature_end) * a / math.sqrt(math.pi)

        begin_points = []
        for l in np.arange(-len_begin, 0, 0.01):
            p = euler_spiral(l, a)
            if curvature_begin < 0: # nach rechts drehen
                p[1] = - p[1] # -> y-achse spiegeln
            begin_points.append(p)
        end_points = []
        for l in np.arange(0, len_end, 0.01):
            p = euler_spiral(l, a)
            if curvature_end < 0:
                p[1] = - p[1]
            end_points.append(p)
        self._points = begin_points + end_points
        if len(self._points) < 2:
            #print(curvature_begin, curvature_end, a)
            pass

    def get_points(self):
        return self._points

    def get_beginning(self):
        dir = np.array(self._points[0]) - np.array(self._points[1])
        return (np.array(self._points[0]), math.atan2(dir[1], dir[0]), self._curv_begin)

    def get_ending(self):
        dir = np.array(self._points[-1]) - np.array(self._points[-2])
        return (np.array(self._points[-1]), math.atan2(dir[1], dir[0]), self._curv_end)

class Intersection(Primitive):
    def __init__(self, args):
        self._size = 0.9 # TODO
        self._target_dir = args["turn"]
        self._rule = args["rule"]
        if self._target_dir == "left":
            self._points = [[0, -self._size], [0, 0], [-self._size, 0]]
        elif self._target_dir == "right":
            self._points = [[0, -self._size], [0, 0], [self._size, 0]]
        elif self._target_dir == "straight":
            self._points = [[0, -self._size], [0, 0], [0, self._size]]

    def get_points(self):
        return self._points

    def get_beginning(self):
        return (np.array([0, -self._size]), 1.5 * math.pi, 0)

    def get_ending(self):
        if self._target_dir == "left":
            return (np.array([-self._size, 0]), math.pi, 0)
        elif self._target_dir == "right":
            return (np.array([self._size, 0]), 0, 0)
        elif self._target_dir == "straight":
            return (np.array([0, self._size]), 0.5 * math.pi, 0)

    def export(self, config):
        southRight = schema.lanelet(leftBoundary=schema.boundary(), rightBoundary=schema.boundary())
        southRight.leftBoundary.lineMarking = "dashed"
        southRight.rightBoundary.lineMarking = "solid"
        southRight.leftBoundary.point.append(schema.point(x=0, y=-self._size))
        southRight.leftBoundary.point.append(schema.point(x=0, y=-config.road_width))
        southRight.rightBoundary.point.append(schema.point(x=config.road_width, y=-self._size))
        southRight.rightBoundary.point.append(schema.point(x=config.road_width, y=-config.road_width))

        southLeft = schema.lanelet(leftBoundary=schema.boundary(), rightBoundary=schema.boundary())
        southLeft.rightBoundary.lineMarking = "solid"
        southLeft.leftBoundary.point.append(schema.point(x=0, y=-config.road_width))
        southLeft.leftBoundary.point.append(schema.point(x=0, y=-self._size))
        southLeft.rightBoundary.point.append(schema.point(x=-config.road_width, y=-config.road_width))
        southLeft.rightBoundary.point.append(schema.point(x=-config.road_width, y=-self._size))

        northRight = schema.lanelet(leftBoundary=schema.boundary(), rightBoundary=schema.boundary())
        northRight.leftBoundary.lineMarking = "dashed"
        northRight.rightBoundary.lineMarking = "solid"
        northRight.leftBoundary.point.append(schema.point(x=0, y=self._size))
        northRight.leftBoundary.point.append(schema.point(x=0, y=config.road_width))
        northRight.rightBoundary.point.append(schema.point(x=-config.road_width, y=self._size))
        northRight.rightBoundary.point.append(schema.point(x=-config.road_width, y=config.road_width))

        northLeft = schema.lanelet(leftBoundary=schema.boundary(), rightBoundary=schema.boundary())
        northLeft.rightBoundary.lineMarking = "solid"
        northLeft.leftBoundary.point.append(schema.point(x=0, y=config.road_width))
        northLeft.leftBoundary.point.append(schema.point(x=0, y=self._size))
        northLeft.rightBoundary.point.append(schema.point(x=config.road_width, y=config.road_width))
        northLeft.rightBoundary.point.append(schema.point(x=config.road_width, y=self._size))

        eastRight = schema.lanelet(leftBoundary=schema.boundary(), rightBoundary=schema.boundary())
        eastRight.rightBoundary.lineMarking = "solid"
        eastRight.leftBoundary.point.append(schema.point(x=self._size, y=0))
        eastRight.leftBoundary.point.append(schema.point(x=config.road_width, y=0))
        eastRight.rightBoundary.point.append(schema.point(x=self._size, y=config.road_width))
        eastRight.rightBoundary.point.append(schema.point(x=config.road_width, y=config.road_width))

        eastLeft = schema.lanelet(leftBoundary=schema.boundary(), rightBoundary=schema.boundary())
        eastLeft.rightBoundary.lineMarking = "solid"
        eastLeft.leftBoundary.lineMarking = "dashed"
        eastLeft.leftBoundary.point.append(schema.point(x=config.road_width, y=0))
        eastLeft.leftBoundary.point.append(schema.point(x=self._size, y=0))
        eastLeft.rightBoundary.point.append(schema.point(x=config.road_width, y=-config.road_width))
        eastLeft.rightBoundary.point.append(schema.point(x=self._size, y=-config.road_width))

        westRight = schema.lanelet(leftBoundary=schema.boundary(), rightBoundary=schema.boundary())
        westRight.rightBoundary.lineMarking = "solid"
        westRight.leftBoundary.point.append(schema.point(x=-self._size, y=0))
        westRight.leftBoundary.point.append(schema.point(x=-config.road_width, y=0))
        westRight.rightBoundary.point.append(schema.point(x=-self._size, y=-config.road_width))
        westRight.rightBoundary.point.append(schema.point(x=-config.road_width, y=-config.road_width))

        westLeft = schema.lanelet(leftBoundary=schema.boundary(), rightBoundary=schema.boundary())
        westLeft.leftBoundary.lineMarking = "dashed"
        westLeft.rightBoundary.lineMarking = "solid"
        westLeft.leftBoundary.point.append(schema.point(x=-config.road_width, y=0))
        westLeft.leftBoundary.point.append(schema.point(x=-self._size, y=0))
        westLeft.rightBoundary.point.append(schema.point(x=-config.road_width, y=config.road_width))
        westLeft.rightBoundary.point.append(schema.point(x=-self._size, y=config.road_width))

        if self._rule == "equal":
            northRight.stopLine = "dashed"
            westRight.stopLine = "dashed"
            eastRight.stopLine = "dashed"
            southRight.stopLine = "dashed"
        elif self._rule == "priority-yield" and self._target_dir == "straight":
            westRight.stopLine = "dashed"
            eastRight.stopLine = "dashed"
        elif self._rule == "priority-stop" and self._target_dir == "straight":
            westRight.stopLine = "solid"
            eastRight.stopLine = "solid"
        elif self._rule == "yield":
            northRight.stopLine = "dashed"
            southRight.stopLine = "dashed"
        elif self._rule == "stop":
            northRight.stopLine = "solid"
            southRight.stopLine = "solid"

        result = [southRight, southLeft, northLeft, northRight,
            eastLeft, eastRight, westLeft, westRight]
        pairs = [(southRight, southLeft)]

        if self._target_dir == "left":
            right_lanelet = schema.lanelet(leftBoundary=schema.boundary(), rightBoundary=schema.boundary())
            right_lanelet.rightBoundary.lineMarking = "dashed"
            right_lanelet.leftBoundary.lineMarking = "dashed"
            for angle in np.arange(0, math.pi/2, math.pi/20):
                right_lanelet.rightBoundary.point.append(schema.point(x=-config.road_width + math.cos(angle) * config.road_width * 2,
                    y=-config.road_width + math.sin(angle) * config.road_width * 2))
                right_lanelet.leftBoundary.point.append(schema.point(x=-config.road_width + math.cos(angle) * config.road_width,
                    y=-config.road_width + math.sin(angle) * config.road_width))
            left_lanelet = schema.lanelet(leftBoundary=schema.boundary(), rightBoundary=schema.boundary())
            for angle in np.arange(math.pi/2, 0, -math.pi/20):
                left_lanelet.leftBoundary.point.append(schema.point(x=-config.road_width + math.cos(angle) * config.road_width,
                    y=-config.road_width + math.sin(angle) * config.road_width))
                left_lanelet.rightBoundary.point.append(schema.point(x=-config.road_width,
                    y=-config.road_width))
            result.append(right_lanelet)
            result.append(left_lanelet)
            pairs.append((right_lanelet, left_lanelet))
            pairs.append((westLeft, westRight))
            result.append(schema.trafficSign(type="stvo-209-10",
                orientation=math.pi*1.5, centerPoint=schema.point(
                x=config.road_width + 0.1, y= -config.road_width - 0.25)))
        elif self._target_dir == "right":
            right_lanelet = schema.lanelet(leftBoundary=schema.boundary(), rightBoundary=schema.boundary())
            right_lanelet.leftBoundary.lineMarking = "dashed"
            for angle in np.arange(math.pi, math.pi/2, -math.pi/20):
                right_lanelet.rightBoundary.point.append(schema.point(x=config.road_width,
                    y=-config.road_width))
                right_lanelet.leftBoundary.point.append(schema.point(x=config.road_width + math.cos(angle) * config.road_width,
                    y=-config.road_width + math.sin(angle) * config.road_width))
            left_lanelet = schema.lanelet(leftBoundary=schema.boundary(), rightBoundary=schema.boundary())
            left_lanelet.rightBoundary.lineMarking = "dashed"
            for angle in np.arange(math.pi/2, math.pi, math.pi/20):
                left_lanelet.leftBoundary.point.append(schema.point(x=config.road_width + math.cos(angle) * config.road_width,
                    y=-config.road_width + math.sin(angle) * config.road_width))
                left_lanelet.rightBoundary.point.append(schema.point(x=config.road_width + math.cos(angle) * config.road_width * 2,
                    y=-config.road_width + math.sin(angle) * config.road_width * 2))
            result.append(right_lanelet)
            result.append(left_lanelet)
            pairs.append((right_lanelet, left_lanelet))
            pairs.append((eastLeft, eastRight))
            result.append(schema.trafficSign(type="stvo-209-20",
                orientation=math.pi*1.5, centerPoint=schema.point(
                x=config.road_width + 0.1, y= -config.road_width - 0.25)))
        elif self._target_dir == "straight":
            right_lanelet = schema.lanelet(leftBoundary=schema.boundary(), rightBoundary=schema.boundary())
            right_lanelet.rightBoundary.point.append(schema.point(x=config.road_width, y=-config.road_width))
            right_lanelet.rightBoundary.point.append(schema.point(x=config.road_width, y=config.road_width))
            right_lanelet.leftBoundary.point.append(schema.point(x=0, y=-config.road_width))
            right_lanelet.leftBoundary.point.append(schema.point(x=0, y=config.road_width))
            left_lanelet = schema.lanelet(leftBoundary=schema.boundary(), rightBoundary=schema.boundary())
            left_lanelet.rightBoundary.point.append(schema.point(x=-config.road_width, y=config.road_width))
            left_lanelet.rightBoundary.point.append(schema.point(x=-config.road_width, y=-config.road_width))
            left_lanelet.leftBoundary.point.append(schema.point(x=0, y=config.road_width))
            left_lanelet.leftBoundary.point.append(schema.point(x=0, y=-config.road_width))
            result.append(right_lanelet)
            result.append(left_lanelet)
            pairs.append((right_lanelet, left_lanelet))
            pairs.append((northLeft, northRight))

        type_map = {"priority-yield":"stvo-306", "priority-stop":"stvo-306",
            "yield":"stvo-205", "stop":"stvo-206"}
        if self._rule in type_map:
            result.append(schema.trafficSign(type=type_map[self._rule],
                orientation=math.pi*1.5, centerPoint=schema.point(
                x=config.road_width + 0.1, y= -config.road_width - 0.5)))

        return Export(result, pairs)

class StraightLineObstacle(StraightLine):
    def __init__(self, args):
        super().__init__(args)
        self._width = float(args["width"])
        self._position = float(args["position"])
        self._anchor = args["anchor"]

    def export(self, config):
        y = self._position * config.road_width
        if self._anchor == "left":
            y -= self._width / 2
        elif self._anchor == "right":
            y += self._width / 2
        rect = schema.rectangle(length=self._length,
            width=self._width, orientation=0,
            centerPoint=schema.point(x=self._length / 2, y=y))
        obstacle = schema.obstacle(role="static", type="parkedVehicle", shape=schema.shape())
        obstacle.shape.rectangle.append(rect)

        export = super().export(config)
        export.objects.append(obstacle)
        return export

class ParkingObstacle(StraightLine):
    def __init__(self, args):
        super().__init__(args)
        self._width = float(args["width"])

    def export(self, config):
        y = - config.road_width - 0.3 + self._width / 2
        rect = schema.rectangle(length=self._length,
            width=self._width, orientation=0,
            centerPoint=schema.point(x=self._length / 2, y=y))
        obstacle = schema.obstacle(role="static", type="parkedVehicle", shape=schema.shape())
        obstacle.shape.rectangle.append(rect)

        parking_lane = schema.lanelet(leftBoundary=schema.boundary(), rightBoundary=schema.boundary())
        parking_lane.leftBoundary.point.append(schema.point(x=0, y=-config.road_width))
        parking_lane.leftBoundary.point.append(schema.point(x=self._length, y=-config.road_width))
        parking_lane.rightBoundary.point.append(schema.point(x=0, y=-config.road_width - 0.3))
        parking_lane.rightBoundary.point.append(schema.point(x=self._length, y=-config.road_width - 0.3))
        parking_lane.rightBoundary.lineMarking = "solid"

        export = super().export(config)
        export.objects.append(obstacle)
        export.objects.append(parking_lane)
        return export

class ParkingLot(StraightLine):
    def __init__(self, args):
        super().__init__(args)

    def export(self, config):
        parking_lane = schema.lanelet(leftBoundary=schema.boundary(), rightBoundary=schema.boundary())
        parking_lane.leftBoundary.point.append(schema.point(x=0, y=-config.road_width))
        parking_lane.leftBoundary.point.append(schema.point(x=self._length, y=-config.road_width))
        parking_lane.rightBoundary.point.append(schema.point(x=0, y=-config.road_width - 0.3))
        parking_lane.rightBoundary.point.append(schema.point(x=self._length, y=-config.road_width - 0.3))
        parking_lane.rightBoundary.lineMarking = "solid"

        export = super().export(config)
        export.objects.append(parking_lane)
        return export

class BlockedAreaObstacle(StraightLine):
    def __init__(self, args):
        super().__init__(args)
        self._obst_width = float(args["width"])

    def export(self, config):
        rect = schema.rectangle(length=self._length, width=self._obst_width,
            orientation=0, centerPoint=schema.point(x=self._length/2,
            y=-config.road_width+self._obst_width/2))
        obstacle = schema.obstacle(role="static", type="blockedArea",
            shape=schema.shape())
        obstacle.shape.rectangle.append(rect)

        export = super().export(config)
        export.objects.append(obstacle)
        return export

class ZebraCrossing(StraightLine):
    def __init__(self, args):
        super().__init__({
            "length": args["length"],
            "leftLine": "solid",
            "rightLine": "solid",
            "middleLine": "missing"
        })

    def export(self, config):
        zebra = schema.lanelet(leftBoundary=schema.boundary(), rightBoundary=schema.boundary())
        zebra.type = "zebraCrossing"
        zebra.leftBoundary.point.append(schema.point(x=0, y=-config.road_width))
        zebra.leftBoundary.point.append(schema.point(x=0, y=config.road_width))
        zebra.rightBoundary.point.append(schema.point(x=self._length, y=-config.road_width))
        zebra.rightBoundary.point.append(schema.point(x=self._length, y=config.road_width))

        export = super().export(config)
        export.objects.append(zebra)
        return export

class TrafficSign(StraightLine):
    def __init__(self, args):
        super().__init__(dict(length=0.01))
        self._traffic_sign = args["type"]

    def export(self, config):
        traffic_sign = schema.trafficSign(type=self._traffic_sign,
            orientation=math.pi, centerPoint=schema.point(x=self._length / 2,
            y=-config.road_width - 0.15))

        export = super().export(config)
        export.objects.append(traffic_sign)
        return export
