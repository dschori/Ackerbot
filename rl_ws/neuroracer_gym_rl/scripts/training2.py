import gym
import rospy
import numpy as np

from gym.envs.registration import register
from neuroracer_gym import neuroracer_env

# just to register env:
#from neuroracer_discrete import NeuroRacer
from neuroracer_gym.tasks.neuroracer_discrete_task import NeuroRacerTfAgents

import numpy as np

rospy.init_node('neuroracer_qlearn', anonymous=True, log_level=rospy.INFO)

#env = gym.make('NeuroRacer-v0')
env = NeuroRacerTfAgents()

env.initial_position = {'p_x': np.random.uniform(1,4), 'p_y': 3.7, 'p_z': 0.05, 'o_x': 0, 'o_y': 0.0, 'o_z': np.random.uniform(0.4,1), 'o_w': 0.855}

time_step = env.reset()

time_step = env.step(np.array(0, dtype=np.int32))

#state = self.env.reset()

cumulative_reward = time_step.reward
# print(time_step)

for _ in range(3):
    time_step = env.step(np.array(0, dtype=np.int32))
    print(time_step.reward)
