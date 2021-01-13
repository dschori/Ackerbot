import time
import sys

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
        self.cumulated_steps = 0.0
        self.last_action = 1
        self.right_left = 0
        self.action_space = spaces.Discrete(3)
        self.rate = None
        self.speed = 1
        self.set_sleep_rate(100)
        self.number_of_sleeps = 4
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

    def _compute_reward_old(self, observations, done):
        if not done:
            rigth_distance, left_distance, middle_distance = self._get_distances()
            #             print(left_distance, middle_distance, rigth_distance)
            reward = (middle_distance - 3) - np.abs(left_distance - rigth_distance)
        #             if self.last_action!=1:
        #                 reward-=0.001*(1.7**self.right_left - 1)
        else:
            reward = -100

        self.cumulated_reward += reward
        self.cumulated_steps += 1

        return reward

    def _set_action(self, action):
        steering_angle = 0
        self.speed = 1
        if action == 0:  # right
            steering_angle = -1
        if action == 2:  # left
            steering_angle = 1
        if action == 3:  # stop
            steering_angle = 0
            self.speed = 0

        if action == 1:
            self.right_left = 0
        else:
            self.right_left += 1

        #         self.right_left =  action != 1 & self.last_action != 1 & self.last_action != action

        self.last_action = action
        self.steering(steering_angle, self.speed)
        if self.rate:
            for i in range(int(self.number_of_sleeps)):
                self.rate.sleep()
                self.steering(steering_angle, self.speed)

    def _compute_reward_align(self, obs, done):
        # drive along right side
        if not done:
            #ranges = self.get_laser_scan()
            #left_dist = np.clip(ranges[730:770], None, 10).mean()
            # for 200 laserscan
            left_dist = obs[148:152].mean()
            right_dist = obs[48:52].mean()
            front_dist = obs[98:102].mean()
            if 0.45 < right_dist < 1.5:
                if 0.6 < right_dist < 0.9:
                    return 1.0
                else:
                    return 0.25
            else:
                return -0.05
        else:
            return -200.

    def _compute_reward_area(self, obs, done):
        if not done:
            pos = self._get_pos_x_y()
            if self.x_min > pos[0]:
                self.x_min = pos[0]
            elif self.x_max < pos[0]:
                self.x_max = pos[0]
            if self.y_min > pos[1]:
                self.y_min = pos[1]
            elif self.y_max < pos[1]:
                self.y_max = pos[1]
            area = abs((self.x_max - self.x_min)) * abs((self.y_max - self.y_min))
            if area > self.max_area:
                return_value = area - self.max_area
                self.max_area = area
                return return_value*20.
            else:
                return -0.05
        else:
            return -200.

    def _compute_reward(self, obs, action, finished, done):
        # drive along right side
        if finished:
            return 500.
        if not done:
            # for 300 laserscan
            left_dist = obs[223:227].mean()
            right_dist = obs[73:77].mean()
            front_dist = obs[148:152].mean()
            if left_dist < 0.6 or right_dist < 0.6 or front_dist < 0.6:
                return -1.5
            elif action == 1:
                return 1.
            else:
                return -0.01
        else:
            return -200.
