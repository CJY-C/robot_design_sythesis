# from spinup import dqn_simple_pytorch as dqn
from spinup import dqn_2015_pytorch as dqn
import torch
import os
import tensorflow as tf
import gym
from robotConfigDesign.envs import RobotConfigDesignEnv

dir = os.path.dirname(os.path.realpath(__file__))

envName = 'RobotConfigDesign-v0'
# envName = 'LunarLander-v2'
# envName = 'MountainCar-v0'
# envName = 'CartPole-v1'
# envName = 'SimpleDriving-v0'
# envName = 'CartPoleBulletEnv-v1'
# envName = 'HumanoidBulletEnv-v0'
# env_fn = lambda : gym.make('SimpleDriving-v0')
env_fn = lambda : gym.make(envName)

# ac_kwargs = dict(hidden_sizes=[24,24], activation=tf.nn.relu)
ac_kwargs = dict(hidden_sizes=[64, 64, 64], activation=torch.nn.ReLU)
# ac_kwargs = dict()

logger_kwargs = dict(output_dir=dir + '/data/' + envName, exp_name=envName[:-3])

# ppo(env_fn=env_fn, ac_kwargs=ac_kwargs, steps_per_epoch=5000, epochs=50, logger_kwargs=logger_kwargs)
dqn(
  env_fn=env_fn, 
  ac_kwargs=ac_kwargs, 
  seed=0,
  steps_per_epoch=40,
  epochs=16000, 
  replay_size=int(1e6),
  gamma=0.99,
  epsilon_start=1,
  epsilon_decay=1e-4,
  epsilon_end=0.1,
  q_lr=1e-4,
  batch_size=int(100),
  start_steps=1000,
  max_ep_len=20,
  logger_kwargs=logger_kwargs,
  update_freq=1000,
  save_freq=int(1)
  )
# dqn(
#   env_fn=env_fn, 
#   ac_kwargs=ac_kwargs, 
#   seed=0,
#   steps_per_epoch=500,
#   epochs=100, 
#   replay_size=int(1e6),
#   gamma=0.99,
#   epsilon_start=1,
#   epsilon_decay=1e-3,
#   epsilon_end=0.1,
#   q_lr=1e-3,
#   batch_size=int(32),
#   start_steps=500,
#   max_ep_len=500,
#   logger_kwargs=logger_kwargs,
#   update_freq=100,
#   save_freq=int(1)
#   )
