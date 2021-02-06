import time
import sys
import math
import rospy

from scouting_gym import scouting_env

from gym import spaces
from gym.envs.registration import register

import numpy as np

np.set_printoptions(threshold=sys.maxsize)

default_sleep = 2

print(register(
    id='Scouting-v0',
    entry_point='scouting_gym.tasks.scouting_discrete_task:ScoutingDiscreteTask',
))


class ScoutingDiscreteTask(scouting_env.ScoutingEnv):
    def __init__(self, env_config=None):
        rospy.init_node('neuroracer_qlearn2', anonymous=True, log_level=rospy.INFO)
        self.env_config = env_config
        self.last_action = 1
        self.right_left = 0
        self.action_space = spaces.Discrete(3)
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
