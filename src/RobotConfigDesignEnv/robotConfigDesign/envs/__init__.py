from robotConfigDesign.envs.utils import getConfig, getPath
from robotConfigDesign.envs.register import PathRegister
PathRegister.add_path(getPath(__file__, level=1) + '/res/envConfig.json')
from robotConfigDesign.envs.robotConfigDesignEnv import RobotConfigDesignEnv


if __name__ == "__main__":
    print(PathRegister.get_paths())