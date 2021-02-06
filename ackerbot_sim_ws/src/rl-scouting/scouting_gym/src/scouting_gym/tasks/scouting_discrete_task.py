import time
import sys
import math
import rospy

from scouting_gym import scouting_env

from gym import spaces
from gym.envs.registration import register

import numpy as np

np.set_printoptions(threshold=sys.maxsize)

# timestep_limit_per_episode = 10000 # Can be any Value

default_sleep = 2

print(register(
    id='Scouting-v0',
    entry_point='scouting_gym.tasks.scouting_discrete_task:ScoutingDiscreteTask',
    # timestep_limit=timestep_limit_per_episode,
))


class ScoutingDiscreteTask(scouting_env.ScoutingEnv):
    def __init__(self, env_config=None):
        rospy.init_node('neuroracer_qlearn2', anonymous=True, log_level=rospy.INFO)
        self.last_action = 1
        self.right_left = 0
        self.action_space = spaces.Discrete(3)
        #self.action_space = spaces.Box(low=np.array([-0.6, 0.2]), high=np.array([0.6, 1.0]), shape=(2, ), dtype=np.float32)
        #self.action_space = spaces.Box(low=-0.5, high=0.5, shape=(1, ), dtype=np.float32)
        self.rate = None
        self.speed = 1
        self.set_sleep_rate(100)
        self.number_of_sleeps = 25
        super(ScoutingDiscreteTask, self).__init__()

    def set_sleep_rate(self, hz):
        self.rate = None
        if hz > 0:
            self.rate = rospy.Rate(hz)

    def _set_init_pose(self):
        self.steering(0, speed=0)
        return True

    def _init_env_variables(self):
        self.cumulated_reward = 0.0
        self.last_action = 1
        #         self.right_left = False
        self._episode_done = False

    def _get_distances(self):
        ranges = self.get_laser_scan()
        rigth_distance = np.clip(ranges[175:185], None, 10).mean()
        left_distance = np.clip(ranges[895:905], None, 10).mean()
        middle_distance = np.clip(ranges[525:555], None, 10).mean()
        return rigth_distance, left_distance, middle_distance

    def _set_action_cont(self, action):
        self.cumulated_steps += 1
        self._update_dyn1()
        steering_angle = action[0]
        self.speed = 0.8

        self.last_action = action
        self.steering(steering_angle, self.speed)
        if self.rate:
            for i in range(int(self.number_of_sleeps)):
                self.rate.sleep()
                self.steering(steering_angle, self.speed)

    def _set_action(self, action):
        self.cumulated_steps += 1
        #self._update_dyn1()
        steering_angle = 0
        self.speed = 0.6
        if action == 0:  # right
            steering_angle = -0.45
        elif action == 1:  # middle
            steering_angle = 0
        elif action == 2:  # left
            steering_angle = 0.45

        self.last_action = action
        self.steering(steering_angle, self.speed)
        if self.rate:
            for i in range(int(self.number_of_sleeps)):
                self.rate.sleep()
                self.steering(steering_angle, self.speed)

    def _compute_reward_old(self, obs, action, done, finished):
        reward1 = 0.
        reward2 = 0.
        reward3 = 0.
        reward4 = 0.
        ranges = self.get_laser_scan()
        if not done:
            if np.min(ranges) < 0.4:
                reward1 = -0.25 * (1 - np.min(ranges))
        else:
            reward1 = -10.
        right_dist = np.mean(ranges[200:300])
        if 0.6 < right_dist < 0.9:
            reward2 = 0.25
        else:
            reward2 = -0.05

        pos_x, pos_y = self._get_pos_x_y()
        if math.dist([pos_x, pos_y], [self.target_p[0], self.target_p[1]]) < 0.4:
            reward3 = 10
        else:
            #reward3 = -0.01 * (math.dist([pos_x, pos_y], [self.target_p[0], self.target_p[1]]) / math.dist([self.initial_position['p_x'], self.initial_position['p_y']], [self.target_p[0], self.target_p[1]]))
            reward3 = -0.01 * (math.dist([self.target_p[0], self.target_p[1]], [pos_x, pos_y]) / math.dist([self.initial_position['p_x'], self.initial_position['p_y']], [pos_x, pos_y]))

        if math.dist([pos_x, pos_y], [self.target_p[0], self.target_p[1]]) > 0.4 and action == 3:
            reward4 = -.1

        reward5 = 0.
        if action == 1:
            reward5 = 0.01

        reward = reward1 + reward3 + reward4
        reward = reward1 + reward2
        self.reward_publisher.publish(reward)
        return reward

    def _compute_reward(self, obs, action, done, finished):
        reward1, reward2 = .0, .0
        if finished:
            self.reward_publisher.publish(100.)
            return 100.
        if not done:
            pos_x, pos_y = self._get_pos_x_y()

            d = self._get_distance((pos_x, pos_y), (self.target_p[0], self.target_p[1]))

            reward1 = 2.*(self.last_d - d)
            self.last_d = d

            ranges = self.get_laser_scan()
            if np.min(ranges) < 1.0:
                reward2 = -0.5 * (1. - np.min(ranges))

            self.reward_publisher.publish(reward1 + reward2)
            return reward1 + reward2
        else:
            self.reward_publisher.publish(-100.)
            return -100.

    def _compute_reward_old2(self, obs, action, done, finished):
        if finished:
            self.reward_publisher.publish(100.)
            return 100.
        if not done:
            pos_x, pos_y = self._get_pos_x_y()
            d = self._get_distance((pos_x, pos_y), (self.target_p[0], self.target_p[1]))
            p_x, p_y = abs(self.target_p[0] - pos_x), abs(self.target_p[1] - pos_y)
            if d < self.last_d:
                dt = self.last_d - d
                self.last_d = d
                self.reward_publisher.publish(dt*2)
                return dt*2
            else:
                self.reward_publisher.publish(-0.001)
                return -0.001
        else:
            self.reward_publisher.publish(-100.)
            return -100.
