#!/usr/bin/env python3
import math
import matplotlib.pyplot as plt
import numpy as np

ANGULAR_ACC = 0.1
MIN_RADIUS = 5

sign = lambda x: math.copysign(1, x)

class EulerSnake:
    def __init__(self, name, start_x, start_y, angle):
        self.name = name
        self.pos_x = start_x
        self.pos_y = start_y
        self.angle = angle
        self.angular_vel = 0
        self.vel = 1
        self.finished = False
        self.x = [start_x]
        self.y = [start_y]

    def get_delta_angle(self, target_x, target_y):
        dir_x = target_x - self.pos_x
        dir_y = target_y - self.pos_y
        dir_len = math.sqrt(dir_x * dir_x + dir_y * dir_y)
        scalar_prod = dir_x * math.cos(self.angle) + dir_y * math.sin(self.angle)
        #angle_to_target = math.atan2(dir_y, dir_x)
        #delta_angle = angle_to_target - self.angle
        delta_angle = math.acos(np.clip(scalar_prod / dir_len, -1, 1))
        orient = dir_x *  math.sin(self.angle) - dir_y * math.cos(self.angle)
        if orient > 0:
            delta_angle = - delta_angle
        return delta_angle

    def get_new_point(self, angular_vel, dt):
        drive_angle = angular_vel * dt
        return (self.pos_x + math.cos(self.angle+drive_angle) * dt * self.vel,
            self.pos_y + math.sin(self.angle+drive_angle) * dt * self.vel)

    def step(self, target_x, target_y, dt):
        delta_angle = self.get_delta_angle(target_x, target_y)
        tmp_angular_velocity_test = self.angular_vel
        tmp_angular_velocity = self.angular_vel
        if tmp_angular_velocity == 0:
            k_tilde = 0
        else:
            k_tilde = delta_angle / (tmp_angular_velocity * dt)
        k_star = tmp_angular_velocity / (ANGULAR_ACC * dt)
        if math.fabs(k_star) > math.fabs(k_tilde):# and sign(k_star) == sign(k_tilde) and False:
            tmp_angular_velocity -= ANGULAR_ACC * sign(tmp_angular_velocity) * dt
            #print(self.name, k_star, k_tilde)
        else:
            #print(self.name, "not", k_star, k_tilde)
            if delta_angle > 0:
                tmp_angular_velocity += ANGULAR_ACC * dt
            else:
                tmp_angular_velocity -= ANGULAR_ACC * dt
        #max_angular_vel = (self.vel * dt) / (MIN_RADIUS / (2 * math.pi))
        #self.angular_vel = np.clip(self.angular_vel, -max_angular_vel, max_angular_vel)
        #TEST new angular velocity

        #check if the minRadius is  fine

        #if it fails, take old one
        drive_angle = tmp_angular_velocity * dt
        if math.fabs(delta_angle) < math.fabs(drive_angle):
            drive_angle = delta_angle
            tmp_angular_velocity = delta_angle/dt #This cuts the angular velocity!

        new_point = self.get_new_point(tmp_angular_velocity, dt)
        new_point_valid = True
        if len(self.x) >= 2:
            circle = circle_from_points(self.x[-2], self.y[-2], self.x[-1], self.y[-1], new_point[0], new_point[1])
            if circle is not None:
                if circle[1] < MIN_RADIUS:
                    new_point_valid = False

        #new_point_valid = True #TODO
        if not new_point_valid:
            new_point = self.get_new_point(self.angular_vel, dt)
            #print("Not valid ", tmp_angular_velocity)
        else:
            #print("valid ", tmp_angular_velocity,self.angular_vel)
            self.angular_vel=tmp_angular_velocity

        drive_angle = self.angular_vel * dt
        self.angle += drive_angle
        self.pos_x = new_point[0]
        self.pos_y = new_point[1]
        self.x.append(self.pos_x)
        self.y.append(self.pos_y)

        if math.fabs(self.pos_x - target_x) < self.vel * dt and math.fabs(self.pos_y - target_y) < self.vel * dt:
            self.finished = True

def circle_from_points(x1, y1, x2, y2, x3, y3):
    s1 = np.array([[y2 - y1], [- (x2 - x1)]])
    s2 = np.array([[y3 - y2], [- (x3 - x2)]])
    mid1 = 0.5*np.array([[x1 + x2], [y1 + y2]])
    mid2 = 0.5*np.array([[x2 + x3], [y2 + y3]])
    b = mid2 - mid1
    A = np.hstack((s1, s2))
    # print(s1, s2)
    #print(A, b)
    if np.linalg.matrix_rank(A) == 2 :
        result = np.linalg.solve(A, b)
        circle_mid = mid1 + result[0] * s1
        #print(circle_mid)
        radius = np.linalg.norm(circle_mid - [[x1], [y1]])
        #print("radius ",radius,[x1, y1],circle_mid)
        return (circle_mid, radius)
    else:
        #print('x', end='')
        #print("NO MID FOUND")
        return None

def generate_street(begin_x, begin_y, begin_angle, end_x, end_y, end_angle):
    s1 = EulerSnake("L", begin_x, begin_y, begin_angle)
    s2 = EulerSnake("R", end_x, end_y, end_angle)
    while not s1.finished and not s2.finished:
        if math.fabs(s1.get_delta_angle(s2.pos_x, s2.pos_y)) > math.fabs(s2.get_delta_angle(s1.pos_x, s1.pos_y)):
            s1.step(s2.pos_x, s2.pos_y, 0.1)
        else:
            s2.step(s1.pos_x, s1.pos_y, 0.1)
    return (s1.x + list(reversed(s2.x)), s1.y + list(reversed(s2.y)))

if __name__ == "__main__":
    v = []
    radius = []
    circles_x = []
    circles_y = []
    plt.ion()
    s1 = EulerSnake("L", 0, 0, -math.pi*9/10)
    s2 = EulerSnake("R", 20, 0, -math.pi/4)
    f = plt.figure()
    #plt.xlim([-1,15])
    #plt.ylim([-1,15])

    for i in range(18000):
        if math.fabs(s1.get_delta_angle(s2.pos_x, s2.pos_y)) > math.fabs(s2.get_delta_angle(s1.pos_x, s1.pos_y)):
            s1.step(s2.pos_x, s2.pos_y, 0.05)
            if len(s1.x) >= 3:
                # print("RADIUS", x[-1], y[-1], x[-2], y[-2], x[-3], y[-3])
                mid_and_radius = circle_from_points(s1.x[-1], s1.y[-1], s1.x[-2], s1.y[-2], s1.x[-3], s1.y[-3])
                v.append(s1.angular_vel)
                if mid_and_radius is not None:
                    radius.append(mid_and_radius[1])
                    #print(mid_and_radius[0])
                    if not (mid_and_radius[0][0] > 50 or mid_and_radius[0][0] < -50 or mid_and_radius[0][1] > 50 or mid_and_radius[0][1] < -50):
                        circles_x.append(mid_and_radius[0][0])
                        circles_y.append(mid_and_radius[0][1])
                else:
                    radius.append(900001)
        else:
            s2.step(s1.pos_x, s1.pos_y, 0.05)
        if s1.finished or s2.finished:
            break
    f = plt.figure(1)
    f.clear()
    plt.plot(s1.x,s1.y)
    plt.plot(s2.x,s2.y)
    #circles_numpy = np.squeeze(np.array(circles))
    #print(circles_numpy.shape)
    plt.plot(circles_x, circles_y)
    plt.axis('equal')
    plt.grid()
    #plt.pause(0.000001)
    plt.figure(2)
    # print(radius)
    radius_np = np.array(radius)
    #radius[radius > 20] = -1
    plt.plot(1/radius_np)
    plt.plot(v)
    #plt.pause(0.000001)
    #plt.pause(0.00001)
    #plt.ylim([-1,15])
    plt.pause(1000)
