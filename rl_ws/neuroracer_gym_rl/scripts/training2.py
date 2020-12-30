import gym
import rospy
import numpy as np
import tensorflow as tf

from gym.envs.registration import register
from neuroracer_gym import neuroracer_env
from tf_agents.environments import tf_py_environment, utils
from tf_agents.networks import q_network
from tf_agents.agents.dqn import dqn_agent
from tf_agents.agents.ddpg import ddpg_agent
from tf_agents.utils import common
from tf_agents.policies import random_tf_policy
from tf_agents.replay_buffers import tf_uniform_replay_buffer
from tf_agents.trajectories import trajectory
from tf_agents.trajectories import time_step as ts

# just to register env:
from neuroracer_gym.tasks.neuroracer_discrete_task import NeuroRacerTfAgents

import numpy as np

rospy.init_node('neuroracer_qlearn', anonymous=True, log_level=rospy.INFO)

env = NeuroRacerTfAgents()

env.initial_position = {'p_x': np.random.uniform(1, 4), 'p_y': 3.7, 'p_z': 0.05, 'o_x': 0, 'o_y': 0.0,
                'o_z': np.random.uniform(0.4, 1), 'o_w': 0.855}

print('action_spec:', env.action_spec())
print('time_step_spec.observation:', env.time_step_spec().observation)
print('time_step_spec.step_type:', env.time_step_spec().step_type)
print('time_step_spec.discount:', env.time_step_spec().discount)
print('time_step_spec.reward:', env.time_step_spec().reward)

utils.validate_py_environment(env, episodes=5)

time_step = env.reset()

next_time_step = env.step(np.array(0, dtype=np.int32))

# state = self.env.reset()

cumulative_reward = time_step.reward
# print(time_step)

for _ in range(1):
    time_step = env.step(np.array(2, dtype=np.int32))
    print(time_step.reward)

print(env.observation_spec())
print(env.action_spec())

env = tf_py_environment.TFPyEnvironment(env)

fc_layer_params = (75, 40)
dropout_layer_params = (0.25, 0.25)

q_net = q_network.QNetwork(
    env.observation_spec(),
    env.action_spec(),
    fc_layer_params=fc_layer_params,
    dropout_layer_params=dropout_layer_params)

optimizer = tf.compat.v1.train.AdamOptimizer(learning_rate=1e-4)

train_step_counter = tf.Variable(0)

agent = dqn_agent.DqnAgent(
    env.time_step_spec(),
    env.action_spec(),
    q_network=q_net,
    optimizer=optimizer,
    td_errors_loss_fn=common.element_wise_squared_loss,
    train_step_counter=train_step_counter)

agent.initialize()

print(env.batch_size)

print(agent.collect_data_spec)

# Working until here

def collect_step(environment, policy, buffer):
    time_step = environment.current_time_step()
    # observation = tf.ones((1080))
    # observation = tf.reshape(time_step.observation, [1080])
    # time_step = ts.restart(observation)
    # time_step = ts.restart(time_step.observation, 1)
    action_step = policy.action(time_step)
    next_time_step = environment.step(action_step.action)
    traj = trajectory.from_transition(time_step, action_step, next_time_step)

    # Add trajectory to the replay buffer
    buffer.add_batch(traj)


def collect_data(env, policy, buffer, steps):
    for _ in range(steps):
        collect_step(env, policy, buffer)


def compute_avg_return(environment, policy, num_episodes=10):
    total_return = 0.0
    for _ in range(num_episodes):

        time_step = environment.reset()
        episode_return = 0.0

        while not time_step.is_last():
            action_step = policy.action(time_step)
            time_step = environment.step(action_step.action)
            episode_return += time_step.reward
        total_return += episode_return

    avg_return = total_return / num_episodes
    return avg_return.numpy()[0]


random_policy = random_tf_policy.RandomTFPolicy(env.time_step_spec(),
                                                env.action_spec())

replay_buffer = tf_uniform_replay_buffer.TFUniformReplayBuffer(
    data_spec=agent.collect_data_spec,
    batch_size=env.batch_size,
    max_length=1000)

#print(env.reset())
print(random_policy.action(time_step=env.reset()))

collect_data(env, random_policy, replay_buffer, 10)

#print(compute_avg_return(env, random_policy, 2))

dataset = replay_buffer.as_dataset(
    num_parallel_calls=1,
    sample_batch_size=1,
    num_steps=2).prefetch(3)

print(dataset)

# Reset the train step
agent.train_step_counter.assign(0)

# Evaluate the agent's policy once before training.
avg_return = compute_avg_return(env, agent.policy, 2)
returns = [avg_return]
iterator = iter(dataset)
for _ in range(10000):

    # Collect a few steps using collect_policy and save to the replay buffer.
    # collect_data(env, agent.collect_policy, replay_buffer, 1)
    collect_data(env, random_policy, replay_buffer, 1)

    # Sample a batch of data from the buffer and update the agent's network.
    experience, unused_info = next(iterator)
    # print(experience.observation.numpy())
    j = 0
    train_loss = agent.train(experience).loss

    step = agent.train_step_counter.numpy()

    if step % 5 == 0:
        print('step = {0}: loss = {1}'.format(step, train_loss))

    if step % 400 == 0:
        avg_return = compute_avg_return(env, agent.policy, 2)
        print('step = {0}: Average Return = {1}'.format(step, avg_return))
        returns.append(avg_return)
