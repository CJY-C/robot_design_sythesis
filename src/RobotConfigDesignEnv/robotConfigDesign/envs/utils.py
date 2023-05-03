from robotConfigDesign.envs import register
import json
import os
import sys

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


if __name__ == '__main__':
    register.PathRegister.add_path('/home/masa/learning/rl/undergraduate/cjy/robot_design_sythesis/test/defense/config.json')

    print(register.PathRegister.get_paths())

    config = getConfig()
    print(config['ROBOT_POS'])
    print(config['EX'])