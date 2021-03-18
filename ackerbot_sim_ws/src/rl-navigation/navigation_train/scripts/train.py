#!/usr/bin/python

import gym
import ray
from ray import tune
from ray.rllib.agents.ppo import PPOTrainer
from navigation_gym.tasks.navigation_discrete_task import NavigationDiscreteTask

env = gym.make('Navigation-v0')

print("Observation Space: {}".format(env.observation_space))
print("Action Space: {}".format(env.action_space))

obs = env.reset()

train_config = {
    "env": NavigationDiscreteTask,
    "lr": 0.0001,
    "gamma": 0.99,
    "clip_rewards": None,
    "clip_actions": True,
    "num_gpus": 1,
    "num_workers": 1,
    "model": {"dim": 84,
              "conv_filters":
                  [[16, [8, 8], 4], [32, [4, 4], 2], [256, [11, 11], 1]]}
}

stop = {
    "episodes_total": 5000,
}

ray.init()


def train(stop_criteria, config):
    analysis = ray.tune.run(PPOTrainer, config=config,
                            stop=stop_criteria,
                            checkpoint_freq=1,
                            checkpoint_at_end=True)
    checkpoints = analysis.get_trial_checkpoints_paths(trial=analysis.get_best_trial('episode_reward_mean', mode='max'),
                                                       metric='episode_reward_mean',
                                                       )
    checkpoint_path = checkpoints[0][0]
    return checkpoint_path, analysis


checkpoint_path, analysis = train(stop_criteria=stop,
                                  config=train_config)
