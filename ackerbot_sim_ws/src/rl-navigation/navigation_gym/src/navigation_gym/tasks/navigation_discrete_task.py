import sys
import rospy

from navigation_gym import navigation_env

from gym import spaces
from gym.envs.registration import register
import numpy as np

np.set_printoptions(threshold=sys.maxsize)

register(
    id='Navigation-v0',
    entry_point='navigation_gym.tasks.navigation_discrete_task:NavigationDiscreteTask',
)


class NavigationDiscreteTask(navigation_env.NavigationEnv):
    def __init__(self, env_config=None):
        rospy.init_node('rl-navigation', anonymous=True, log_level=rospy.INFO)
        self.env_config = env_config
        self.last_action = 1
        self.right_left = 0
        self.action_space = spaces.Discrete(3)
        self.rate = None
        self.set_sleep_rate(100)
        self.number_of_sleeps = 25
        super(NavigationDiscreteTask, self).__init__()

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

    def _set_action(self, action):
        self.cumulated_steps += 1
        steering_angle = 0
        speed = self.speed
        speed = np.clip(speed, 0.1, 1.)
        if action == 0:  # right
            steering_angle = self.right_steering
            steering_angle = np.clip(steering_angle, -1., 0.)
        elif action == 1:  # middle
            steering_angle = 0
        elif action == 2:  # left
            steering_angle = self.left_steering
            steering_angle = np.clip(steering_angle, 0., 1.)

        self.last_action = action
        self.steering(steering_angle, speed)
        if self.rate:
            for i in range(int(self.number_of_sleeps)):
                self.rate.sleep()
                self.steering(steering_angle, speed)

    def _compute_reward(self, obs, action, done, finished):
        reward1, reward2 = .0, .0

        # If goal reached, return here:
        if finished:
            reward = 100.
            self.reward_publisher.publish(reward)
            return reward

        # If crashed, return here:
        if done:
            reward = -100.
            self.reward_publisher.publish(reward)
            return reward

        # Else ,cumulate distance to target and distance to obstacle reward
        pos_x, pos_y = self._get_pos_x_y()

        d = self._get_distance((pos_x, pos_y), (self.target_p[0], self.target_p[1]))

        reward1 = 2.*(self.last_d - d)
        self.last_d = d

        ranges = self.get_laser_scan()
        if np.min(ranges) < 1.0:
            reward2 = -0.5 * (1. - np.min(ranges))

        self.reward_publisher.publish(reward1 + reward2)
        return reward1 + reward2
