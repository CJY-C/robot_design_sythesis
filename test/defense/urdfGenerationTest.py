from robotConfigDesign.envs import register
from robotConfigDesign.envs.utils import getPath
register.PathRegister.add_path(getPath(__file__) + '/envConfig.json')

from urdfGenerator.Enums import ModuleType
from urdfGenerator.ModuleConfig import moduleType2action

# 动态拼接机器人测试
def test_dynamic():
    import gym
    from time import sleep

    env = gym.make('RobotConfigDesign-v0')

    env.render()
    env.reset()

    for _ in range(1):
        env.render()
        s, r, d, _ = env.step(moduleType2action(ModuleType.BASEL))
        s, r, d, _ = env.step(moduleType2action(ModuleType.JOINTL))
        s, r, d, _ = env.step(moduleType2action(ModuleType.JOINTL))
        s, r, d, _ = env.step(moduleType2action(ModuleType.STRAIGHTLINKLM))
        s, r, d, _ = env.step(moduleType2action(ModuleType.JOINTM))
        s, r, d, _ = env.step(moduleType2action(ModuleType.CORNERLINKMM))
        s, r, d, _ = env.step(moduleType2action(ModuleType.STRAIGHTLINKMS))
        s, r, d, _ = env.step(moduleType2action(ModuleType.JOINTS))
        s, r, d, _ = env.step(moduleType2action(ModuleType.JOINTS))
        s, r, d, _ = env.step(moduleType2action(ModuleType.JOINTS))
        s, r, d, _ = env.step(moduleType2action(ModuleType.ENDEFFECTORS))

    while True:
        pass

if __name__ == "__main__":
    test_dynamic()