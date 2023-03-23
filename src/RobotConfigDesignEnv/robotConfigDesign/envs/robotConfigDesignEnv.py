import os
import gym
from gym.utils import seeding
import numpy as np
import pybullet as p2
from pybullet_utils import bullet_client as bc

from urdfGenerator.Arrangement import Arrangement
from urdfGenerator.ModuleConfig import getModuleTypeList, generateModule
from urdfGenerator.Enums import ModuleType


CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))  # 获取执行文件所在目录的绝对路径
PARENT_DIR = os.path.abspath(os.path.join(CURRENT_DIR, os.pardir))  # 获取上级目录的绝对路径

ROBOT_POS = [0, 0, 0]
ROBOT_PATH = PARENT_DIR + '/res'
ROBOT_NAME = "rebot"
MODULETYPE_LIST = getModuleTypeList()
MODULETYPE_NUM = len(MODULETYPE_LIST)
MAX_MODULECNT = 20

LOGGING_PATH = PARENT_DIR + "/info.log"

EPSILON_P = 0.1
EPSILON_N = 0.1

W_J = 0.01
W_M = 0.01

Plane_Path = PARENT_DIR + '/res/plane.urdf'

OBS_PATH = PARENT_DIR + "/res/obstacle-25.SLDASM/urdf/obstacle-25.SLDASM.urdf"
OBS_WIDTH = 0.25
OBS_LENGTH = 0.25
OBS_HEIGHT = 0.25
OBS_PROB = 0.01
EX = [-1, 1]
EY = [-1, 1]
EZ = [0, 1]
GX = int(np.ptp(np.array(EX)) / OBS_WIDTH)
GY = int(np.ptp(np.array(EY)) / OBS_LENGTH)
GZ = int(np.ptp(np.array(EZ)) / OBS_HEIGHT)

import logging
logging.basicConfig(format='%(asctime)s-%(levelname)s: %(message)s', filename=LOGGING_PATH, level=logging.INFO)
logging.basicConfig(format='%(asctime)s-%(levelname)s: %(message)s', filename=LOGGING_PATH, level=logging.DEBUG)
logging.basicConfig(format='%(asctime)s-%(levelname)s: %(message)s', filename=LOGGING_PATH, level=logging.WARNING)
logging.basicConfig(format='%(asctime)s-%(levelname)s: %(message)s', filename=LOGGING_PATH, level=logging.ERROR)



class RobotConfigDesignEnv(gym.Env):
    metadata = {'render.modes': ['human'], 'video.frames_per_second': 50}

    def __init__(self, renders=False):
        logging.info("=======================================RobotConfigDesignEnv init============================================")
        self._A = Arrangement()
        self._robot_id = None
        self._robot_cache_id = None
        self._joint_info = list()
        self._target_pos = None
        self._target_id = None
        self._obstacle_ids = list()
        self._obstacle_matrix = None

        self._plane_id = None

        self._renders = renders
        self._physics_client_id = -1
        # self._render_height = 720
        # self._render_width = 960

        # 定义动作空间和观测空间
        # 动作空间是一个离散空间，每个动作都是一个整数，表示添加的模块类型
        self.action_space = gym.spaces.Discrete(MODULETYPE_NUM)
        # 观测空间有连续部分也有离散部分，连续部分是目标点的位置，离散部分是障碍物的位置，和机器人目前的构型A
        # self.observation_space = gym.spaces.Box(low=np.float32(-1), high=np.float32(1), shape=(3, ))
        self.observation_space = gym.spaces.Dict({
            'A': gym.spaces.MultiDiscrete([MODULETYPE_NUM for x in range(MAX_MODULECNT)]), # TODO: 解决描述问题
            'T': gym.spaces.Box(low=np.float32(-1), high=np.float32(1), shape=(3, )),
            'O': gym.spaces.MultiBinary((GX, GY, GZ)),
        })


    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        assert self.action_space.contains(action), "%r (%s) invalid"%(action, type(action))
        reward = 0
        done = False
        info = {}

        # 执行动作
        p = self._p

        # 2. 添加模块
        success = self._updateRobot(action)

        # p.stepSimulation()

        if self._A.moduleCnt > MAX_MODULECNT: # 判断模块个数是否超过最大值
            logging.info("module count exceed the max count")
            reward = -1
            done = True
        elif self._A.checkEEAttached(): # 判断是否添加末端执行器
            passEvaluation = self._IK()
            logging.info("IK result: " + str(passEvaluation))
            reward = 10 if passEvaluation else -1
            reward += -1 * W_J * self._A.jointNum + -1 * W_M * self._A.totalMass
            # reward = int(passEvaluation)
            done = True
        else: # 非终止状态
            # print("module count not exceed the max count")
            if success:
                # reward = -1 * W_J if self._A.checkJointAttached() else -1 * W_M # TODO: 调整
                # reward = -1 * W_J * self._A.jointNum + -1 * W_M * self._A.totalMass
                reward = 0 
                done = False
            else:
                reward = -1
                done = True
                # done = False

        # p.stepSimulation() # TODO: 这个函数的在IK之后调用，还是在IK之前调用

        return self._state(), reward, done, info
        # return (self._A.getModuleTypeList(MAX_MODULECNT), self._target_pos, self._obstacle_matrix), reward, done, info

    def reset(self):
        logging.info("reset simulation")
        if self._physics_client_id < 0:
            if self._renders:
                self._p = bc.BulletClient(connection_mode=p2.GUI)
                # self._p = bc.BulletClient(connection_mode=p2.GUI, options='--log-level=5')
            else:
                self._p = bc.BulletClient()
            self._physics_client_id = self._p._client

        p = self._p
        p.configureDebugVisualizer(p2.COV_ENABLE_GUI,0)
        # p.configureDebugVisualizer(p2.COV_ENABLE_WIREFRAME,1)
        # 重置仿真环境
        self._clearEnv()
        # p.resetSimulation()
        # 先去除地面
        # self._plane_id = p2.loadURDF(Plane_Path, useFixedBase=True, basePosition=[0, 0, 0], flags = p2.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)

        # 随机初始化观测值&环境&返回观测值
        self._obstacle_matrix, self._target_pos = self._random_gen_obs_target(OBS_PATH, 
                                        obs_width=OBS_WIDTH, obs_length=OBS_LENGTH, obs_height=OBS_HEIGHT,
                                        ex=EX, ey=EY, ez=EZ,
                                        obs_prob=OBS_PROB)

        return self._state()
        # return (self._A.getModuleTypeList(MAX_MODULECNT), self._target_pos, self._obstacle_matrix)

    
    def render(self, mode='human', close=False):
        # 渲染环境
        if mode == 'human':
            self._renders = True
        if mode != "rgb_array":
            return np.array([])
        # 暂不实现返回rgb_array

    def close(self):
        # 关闭环境
        if self._physics_client_id >= 0:
            self._p.disconnect()
            self._physics_client_id = -1

    
    def _random_gen_obs_target(self, obs_filepath, obs_length, obs_width, obs_height, ex, ey, ez, obs_prob):
        lx = np.ptp(np.array(ex))
        ly = np.ptp(np.array(ey))
        lz = np.ptp(np.array(ez))

        gx = int( lx / obs_length )
        gy = int( ly / obs_width )
        gz = int( lz / obs_height )

        ob_matrix = np.random.choice([0, 1], size=(gx, gy, gz), p=[1 - obs_prob, obs_prob])

        # target 是一个三维向量(x, y, z)，表示ob_matrix的一个索引，即ob_matrix[target[0]][target[1]][target[2]] = 0
        tx = np.random.randint(0, gx)
        ty = np.random.randint(0, gy)
        tz = np.random.randint(0, gz)
        ob_matrix[tx][ty][tz] = 0
        target = np.array([tx, ty, tz]) # 先不考虑姿态

        # 将target的位置转换为世界坐标系下的位置
        pos_x = (tx + ex[0] * gx/lx) / gx * lx + obs_length/2
        pos_y = (ty + ey[0] * gy/ly) / gy * ly + obs_width/2
        pos_z = (tz + ez[0] * gz/lz) / gz * lz
        # 在所在obs方盒中随机生成一个位置
        pos_x += np.random.uniform(-obs_length/2, obs_length/2)
        pos_y += np.random.uniform(-obs_width/2, obs_width/2)
        pos_z += np.random.uniform(-obs_height/2, obs_height/2)
        target = np.array([pos_x, pos_y, pos_z])

        self._target_id = p2.addUserDebugPoints([target], [[1, 1, 0]], 10)

        # 机器人基座的位置也要保持为0，需要将机器人的位置转换为ob_matrix的索引, 周围的障碍物也要保持为0
        robot_pos = [ROBOT_POS[0] - ex[0], ROBOT_POS[1] - ey[0], ROBOT_POS[2] - ez[0]]
        robot_pos = [int(robot_pos[0] / lx * gx), int(robot_pos[1] / ly * gy), int(robot_pos[2] / lz * gz)]

        for i in range(gz):
            ob_matrix[robot_pos[0]][robot_pos[1]][robot_pos[2] + i] = 0
            ob_matrix[robot_pos[0]-1][robot_pos[1]][robot_pos[2] + i] = 0
            ob_matrix[robot_pos[0]][robot_pos[1]-1][robot_pos[2] + i] = 0
            ob_matrix[robot_pos[0]-1][robot_pos[1]-1][robot_pos[2] + i] = 0

        ob_pos_list = []
        for x in range(gx):
            for y in range(gy):
                for z in range(gz):
                    if ob_matrix[x][y][z] == 1:
                        pos_x = (x + ex[0] * gx/lx) / gx * lx + obs_length/2
                        pos_y = (y + ey[0] * gy/ly) / gy * ly + obs_width/2
                        pos_z = (z + ez[0] * gz/lz) / gz * lz
                        ob_pos_list.append([pos_x, pos_y, pos_z])
        for pos in ob_pos_list:
            obs_id = p2.loadURDF(obs_filepath, useFixedBase=True, basePosition=pos, flags = p2.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
            self._obstacle_ids.append(obs_id)

        return ob_matrix, target

    def _updateRobot(self, action):
        if self._robot_id is not None:
            if self._renders:
                self._robot_cache_id = self._robot_id
            else:
                self._p.removeBody(self._robot_id)
            self._robot_id = None
            self._joint_info = list()

        success = self._A.addModule(MODULETYPE_LIST[action])
        exportPath = self._A.exportURDF(ROBOT_PATH, ROBOT_NAME)

        p2.setPhysicsEngineParameter(enableFileCaching=0, physicsClientId=self._physics_client_id)
        self._robot_id = p2.loadURDF(exportPath, useFixedBase=True, flags=p2.URDF_MERGE_FIXED_LINKS | p2.URDF_USE_SELF_COLLISION)
        p2.setPhysicsEngineParameter(enableFileCaching=1, physicsClientId=self._physics_client_id)

        joint_num = p2.getNumJoints(self._robot_id, self._physics_client_id)
        for i in range(joint_num):
            self._joint_info.append(p2.getJointInfo(self._robot_id, i, self._physics_client_id))

        if self._renders and self._robot_cache_id is not None:
            self._p.removeBody(self._robot_cache_id)
            self._robot_cache_id = None
        return success
    
    def _IK(self):
        p = self._p

        # 需要有两个关节
        # ee_link_id = self._A.moduleCnt - 1
        ee_link_id = self._A.jointNum - 1
        # for i in range(ee_link_id):
        #     logging.info(p2.getLinkState(self._robot_id, i, computeForwardKinematics=True) )
        if ee_link_id <= 0:
            return False
        # logging.info("ee_link_id: {}".format(ee_link_id))
        target_angle = p2.calculateInverseKinematics(self._robot_id, ee_link_id, self._target_pos, maxNumIterations=100)

        done = False
        success = True
        while True:
            cnt = 0
            for angle in target_angle:
                p2.setJointMotorControl2(self._robot_id, cnt, controlMode=p2.POSITION_CONTROL, targetPosition=angle)
                cnt += 1
            p.stepSimulation()
            
            # logging.info(p2.getLinkState(self._robot_id, ee_link_id, computeForwardKinematics=True) )
            linkState = p2.getLinkState(self._robot_id, ee_link_id, computeForwardKinematics=True, computeLinkVelocity=True)

            for velocity in linkState[-2]:
                if np.abs(velocity) > 1e-13:
                    done = False
                    break
                done = True
                
            contacts = p.getContactPoints(self._robot_id)
            # print("contacts: {}".format(contacts))
            for contact in contacts:
                bodyA = contact[1]
                bodyB = contact[2]
                if bodyA == self._robot_id and bodyB == self._robot_id:
                    logging.info('机器人自碰撞')
                    done = True
                    success = False
                    break
                if bodyA == self._robot_id and bodyB in self._obstacle_ids:
                    logging.info('机器人与障碍物碰撞')
                    done = True
                    success = False
                    break
                if bodyA == self._robot_id and bodyB == self._plane_id:
                    logging.info('机器人与地面碰撞')
                    done = True
                    success = False
                    break

            if done:
                break
        
        # linkState[0], linkState[1] # p_EE, n_EE
        if success:
            success = self._check_distance(np.array(linkState[0]), self._target_pos, EPSILON_P)

        return success


    def _check_distance(self, a:np.array, b:np.array, threshold):
        distance = np.math.sqrt(np.sum(np.array([(a[i] - b[i])**2 for i in range(len(a))])))
        return distance <= threshold

    def _clearEnv(self):
        p = self._p

        # 重置Debug参数
        p.removeAllUserParameters()

        if self._plane_id is not None:
            p.removeBody(self._plane_id)
            self._plane_id = None

        # 重置机器人
        if self._renders and self._robot_cache_id is not None:
            p.removeBody(self._robot_cache_id)
            self._robot_cache_id = None
        if self._robot_id is not None:
            p.removeBody(self._robot_id)
            self._robot_id = None

        self._A = Arrangement()
        self._joint_info = list()

        # 重置障碍物
        if len(self._obstacle_ids) > 0:
            for obs_id in self._obstacle_ids:
                p.removeBody(obs_id)
            self._obstacle_ids = list()

        # 重置目标
        if self._target_id is not None: 
            p.removeUserDebugItem(self._target_id) 
            self._target_id = None
    def _state(self):
        A = np.array(self._A.getModuleTypeList(MAX_MODULECNT))
        T = np.array(self._target_pos)
        O = np.array(self._obstacle_matrix)
        A = A.reshape(combined_shape(1, A.shape))
        T = T.reshape(combined_shape(1, T.shape))
        O = O.reshape(combined_shape(1, O.shape))
        # return (A, T, O)
        return {
            'A': A,
            'T': T,
            'O': O
        }


def combined_shape(length: int, shape=None):
    '''
    合并数组维度，用于生成经验数组。

    参数
    --------
    length: int
      经验数组长度
    shape: None or int or a list of int
      观测值的维度

    返回值 
    ---
    newShape: tuple
      用于初始化经验数组维度的元组

    使用方法 
    ---
    >>> combined_shape(10, 4)
    (10, 4)
    >>> combined_shape(10, (2, 4))
    (10, 2, 4)
    >>> combined_shape(10, None)
    (10, )
    >>> np.zeros(combined_shape(buf_size, obs_dim), dtype=float32)

    '''
    if shape is None:
        return (length,)
    return (length, shape) if np.isscalar(shape) else (length, *shape)


if __name__ == '__main__':
    from robotConfigDesign.envs import RobotConfigDesignEnv
    env = gym.make('RobotConfigDesign-v0')

    if env.observation_space.shape is None:
        s = (env.observation_space.sample())
        for k,v in s.items():
            print(k, v.shape)
    else:
        print(env.observation_space.shape)
    print(env.action_space.shape)
    env.action_space.shape

    state = (env.reset())
    print(state['A'].shape, state['T'].shape, state['O'].shape)

    env.close()