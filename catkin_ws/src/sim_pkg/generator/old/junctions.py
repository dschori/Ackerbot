#!/usr/bin/env python3
import math
import matplotlib.pyplot as plt
import snakes
import random

def normalize_angle(angle):
    while angle < 0:
        angle += 2 * math.pi
    while angle > 2 * math.pi:
        angle -= 2 * math.pi
    return angle

def angle_diff(angle1, angle2):
    difference = angle2 - angle1
    while difference < - math.pi:
        difference += 2 * math.pi
    while difference > math.pi:
        difference -= 2 * math.pi
    return math.fabs(difference)

class Junction:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.angles = None

    def __repr__(self):
        return "Junction ({0:.2f}, {1:.2f})".format(self.x, self.y)

    def init_angles(self, angle):
        # private
        self.angles = [
            normalize_angle(angle),
            normalize_angle(angle + math.pi),
            normalize_angle(angle + math.pi/2),
            normalize_angle(angle - math.pi/2)
        ]

    def get_angle(self, target_x, target_y):
        target_angle = math.atan2(target_y - self.y, target_x - self.x)
        if self.angles is None:
            self.init_angles(target_angle)
        result = self.angles[0]
        del self.angles[0]
        return result
        #best_diff = 2 * math.pi
        #best_index = None
        #for index in range(len(self.angles)):
        #    diff = angle_diff(target_angle, self.angles[index])
        #    if diff < best_diff:
        #        best_diff = diff
        #        best_index = index
        #if best_index is not None:
        #    result = self.angles[best_index]
        #    del self.angles[best_index]
        #    return result

def generate_junctions_circle(n, is_random):
    junctions = []
    angle = math.pi * 2 / n

    for i in range(n):
        offset = random.uniform(-math.pi * 1 /5, math.pi * 1 /5) if is_random else 0
        junctions.append(Junction(math.cos(angle * i + offset) * 30, math.sin(angle * i + offset) * 30))
    return junctions

def generate_junctions_random(n):
    junctions = []
    for i in range(n):
        junctions.append(Junction(random.uniform(-100,100), random.uniform(-100,100)))
    return junctions

def depth_search(currentPath, nodes):
    lastNode = currentPath[-1]

    shortest_path_len = 10000
    shortest_path_node = None
    for n in nodes:
        if lastNode == n:
            continue
        if currentPath.count(n) >= 2:
            continue
        if len(currentPath) >= 2 and currentPath[-2] == n:
            continue
        path_len = (n.x - lastNode.x)**2 + (n.y - lastNode.y)**2
        if path_len < shortest_path_len:
            shortest_path_len = path_len
            shortest_path_node = n
    if shortest_path_node is not None:
        currentPath.append(shortest_path_node)
        depth_search(currentPath, nodes)


if __name__ == "__main__":
    random.seed()
    #junctions = generate_junctions_circle(3, True)
    junctions = generate_junctions_random(6)
    #print(junctions)
    path = [junctions[0]]
    depth_search(path, junctions)
    path.append(path[0])

    for (n1, n2) in zip(path, path[1:]):
        begin_angle = n1.get_angle(n2.x, n2.y)
        end_angle = n2.get_angle(n1.x, n1.y)
        street = snakes.generate_street(n1.x, n1.y, begin_angle, n2.x, n2.y, end_angle)
        plt.plot(street[0], street[1])
    plt.show()
