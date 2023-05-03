from spinup import dqn_2015_pytorch as dqn
import torch
import os
import gym
from robotConfigDesign.envs import RobotConfigDesignEnv

def train(envName, ac_kwargs, logger_kwargs, seed, steps_per_epoch, epochs, replay_size, gamma, epsilon_start, epsilon_decay, epsilon_end, q_lr, batch_size, start_steps, max_ep_len, update_freq, save_freq):
  env_fn = lambda : gym.make(envName)

  dqn(
    env_fn=env_fn, 
    ac_kwargs=ac_kwargs, 
    seed=seed,
    steps_per_epoch=steps_per_epoch,
    epochs=epochs, 
    replay_size=int(replay_size),
    gamma=gamma,
    epsilon_start=epsilon_start,
    epsilon_decay=epsilon_decay,
    epsilon_end=epsilon_end,
    q_lr=q_lr,
    batch_size=int(batch_size),
    start_steps=start_steps,
    max_ep_len=max_ep_len,
    logger_kwargs=logger_kwargs,
    update_freq=update_freq,
    save_freq=int(save_freq)
    )


if __name__ == '__main__':
  # alphaList = [0.0001, 0.0002, 0.0005]
  # alphaList = [0.0001, 0.0002, 0.0005, 0.001]
  alphaList = [0.00025]
  alphaList.reverse()
  gammaList = [0.95]

  envName = 'RobotConfigDesign-v0'

  ac_kwargs = dict(hidden_sizes=[64, 64, 64], activation=torch.nn.LeakyReLU)

  dir = os.path.dirname(os.path.realpath(__file__))

  for alpha in alphaList:
    for gamma in gammaList:
      # logger_kwargs = dict(output_dir=dir + '/localdt/' + envName[:-3] + f'random', exp_name=f'random')
      logger_kwargs = dict(output_dir=dir + '/pipidt/' + envName[:-3] + f'-gamma({gamma})-alpha({alpha})', exp_name=f'DQN-gamma({gamma})-alpha({alpha})')
      train(
        envName=envName,
        ac_kwargs=ac_kwargs,
        logger_kwargs=logger_kwargs,
        seed=0,
        steps_per_epoch=1500,
        epochs=100,
        replay_size=int(1e5),
        gamma=gamma,
        epsilon_start=1,
        # epsilon_decay=0,
        epsilon_decay=1e-5,
        epsilon_end=0.1,
        q_lr=alpha,
        batch_size=int(15),
        # start_steps=15,
        start_steps=1500,
        max_ep_len=15,
        update_freq=100,
        save_freq=int(1)
      )

