from gym.envs.registration import register
register(
    id='RobotConfigDesign-v0',
    entry_point='robotConfigDesign.envs:RobotConfigDesignEnv',
)