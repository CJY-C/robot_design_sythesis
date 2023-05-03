from robotConfigDesign.envs import register
import json
import os
import sys
import numpy as np

def getConfig(default=False):
    config = None
    with open(register.PathRegister.get_paths(default), 'r') as f:
        config = json.load(f)
    return config


def getPath(filepath, level=0):
    current_dir = os.path.dirname(os.path.abspath(filepath))  # 获取执行文件所在目录的绝对路径

    for i in range(level):
        current_dir = os.path.abspath(os.path.join(current_dir, os.pardir))  # 获取上级目录的绝对路径

    return current_dir

# Function to disable and enable low-level file descriptors for stdout and stderr

class HiddenOutputs:
    def __enter__(self):
        self._original_stdout_fd = os.dup(sys.stdout.fileno())
        self._original_stderr_fd = os.dup(sys.stderr.fileno())
        sys.stdout.flush()
        sys.stderr.flush()
        devnull = os.open(os.devnull, os.O_WRONLY)
        os.dup2(devnull, sys.stdout.fileno())
        os.dup2(devnull, sys.stderr.fileno())
        os.close(devnull)

    def __exit__(self, exc_type, exc_val, exc_tb):
        sys.stdout.flush()
        sys.stderr.flush()
        os.dup2(self._original_stdout_fd, sys.stdout.fileno())
        os.dup2(self._original_stderr_fd, sys.stderr.fileno())
        os.close(self._original_stdout_fd)
        os.close(self._original_stderr_fd)

def generate_action_mask(state, action_space_size, base_action_index=None):
    from urdfGenerator.Arrangement import Arrangement
    # 根据状态生成动作掩码
    # 根据action_space_size的长度初始化一个numpy数组值全为-1e9

    a = Arrangement()
    a.createFromModuleTypeList(state)

    action_mask = np.ones(action_space_size) * -1e9

    if a.moduleCnt == 0:
        if base_action_index is not None:
            # 将base_action_index中索引的位置设置为0
            action_mask[base_action_index] = 0
            # action_mask[base_action_mask == 1] = 0
        else:
            raise ValueError('base_action_mask is None')
    else:
        if base_action_index is not None:
            action_mask[base_action_index] = 0
        else:
            mask = a.getAttachableSubModuleActions()
            if mask.size > 0:
                action_mask[mask] = 0
            else:
                # 全部置零
                action_mask[:] = 0
    
    del a

    return action_mask


if __name__ == '__main__':
    register.PathRegister.add_path('/home/masa/learning/rl/undergraduate/cjy/robot_design_sythesis/test/defense/envConfig.json')

    print(register.PathRegister.get_paths())

    config = getConfig()
    print(config['ROBOT_POS'])
    print(config['EX'])

    from urdfGenerator.Arrangement import Arrangement
    from urdfGenerator.Enums import ModuleType
    a = Arrangement()
    # a.addModule(ModuleType.BASEL)
    a.addModule(ModuleType.STRAIGHTLINKMS)
    a.addModule(ModuleType.JOINTS)
    a.addModule(ModuleType.ENDEFFECTORS)

    state = {
        'A': a.getModuleTypeList(15)
    }

    action_mask = (generate_action_mask(state['A'], 16))
    print(action_mask)

    valid_actions = np.where(action_mask == 0)[0]
    action = np.random.choice(valid_actions)
    print(action)
