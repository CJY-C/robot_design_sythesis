import logging
import gym
from gym.utils import seeding
import numpy as np
import pybullet as p2
from pybullet_utils import bullet_client as bc
from robotConfigDesign.envs.utils import getConfig, getPath, HiddenOutputs

from urdfGenerator import register
register.PathRegister.add_path('/home/masa/learning/rl/undergraduate/cjy/robot_design_sythesis/src/RobotConfigDesignEnv/robotConfigDesign/res/config.json')

from urdfGenerator.Arrangement import Arrangement
from urdfGenerator.ModuleConfig import getModuleTypeList


class RobotConfigDesignEnv(gym.Env):
    metadata = {'render.modes': ['human'], 'video.frames_per_second': 50}

    def __init__(self, renders=False):
        self._config = getConfig()
        self._defaultConfig = getConfig(default=True)
        self._initConfig()
        logging.info("=======================================RobotConfigDesignEnv init============================================")
        self._A = Arrangement()
        self._MODULETYPE_LIST = getModuleTypeList()
        self._MODULETYPE_NUM = len(self._MODULETYPE_LIST)
        self._robot_id = None
        self._joint_info = list()
        self._target_pos = None
        self._target_id = None
        self._obstacle_ids = list()
        self._obstacle_matrix = None

        self._plane_id = None

        self._renders = renders
        self._physics_client_id = -1

        # 定义动作空间和观测空间
        # 动作空间是一个离散空间，每个动作都是一个整数，表示添加的模块类型
        self.action_space = gym.spaces.Discrete(self._MODULETYPE_NUM)
        # 观测空间有连续部分也有离散部分，连续部分是目标点的位置，离散部分是障碍物的位置，和机器人目前的构型A
        self.observation_space = gym.spaces.Dict({
            'A': gym.spaces.MultiDiscrete([self._MODULETYPE_NUM for x in range(self._config['MAX_MODULECNT'])]), # TODO: 解决描述问题
            'T': gym.spaces.Box(low=np.float32(-1), high=np.float32(1), shape=(3, )),
            'O': gym.spaces.MultiBinary((self._GX, self._GY, self._GZ)),
        })
    
    def _initConfig(self):

        self._visualization = self._config['VISUALIZATION'] if 'VISUALIZATION' in self._config else self._defaultConfig['VISUALIZATION']
        self._robot_cache_id = None

        self._robot_pos = self._config['ROBOT_POS'] if 'ROBOT_POS' in self._config else self._defaultConfig['ROBOT_POS']
        self._max_modulecnt = self._config['MAX_MODULECNT'] if 'MAX_MODULECNT' in self._config else self._defaultConfig['MAX_MODULECNT']
        self._epsilon_p = self._config['EPSILON_P'] if 'EPSILON_P' in self._config else self._defaultConfig['EPSILON_P']
        self._epsilon_n = self._config['EPSILON_N'] if 'EPSILON_N' in self._config else self._defaultConfig['EPSILON_N']
        self._w_j = self._config['W_J'] if 'W_J' in self._config else self._defaultConfig['W_J']
        self._w_m = self._config['W_M'] if 'W_M' in self._config else self._defaultConfig['W_M']
        self._obs_prob = self._config['OBS_PROB'] if 'OBS_PROB' in self._config else self._defaultConfig['OBS_PROB']
        self._ex = self._config['EX'] if 'EX' in self._config else self._defaultConfig['EX']
        self._ey = self._config['EY'] if 'EY' in self._config else self._defaultConfig['EY']
        self._ez = self._config['EZ'] if 'EZ' in self._config else self._defaultConfig['EZ']

        PARENT_DIR = getPath(__file__, level=1)
        self._ROBOT_PATH = self._config['ROBOT_PATH'] if 'ROBOT_PATH' in self._config else PARENT_DIR + '/res'
        self._LOGGING_PATH = self._config['LOGGING_PATH'] + "/info.log" if 'LOGGING_PATH' in self._config else PARENT_DIR + "/info.log"
        self._PLANE_PATH = self._config['PLANE_PATH'] if 'PLANE_PATH' in self._config else PARENT_DIR + '/res/plane.urdf'
        self._OBS_PATH = self._config['OBS_PATH'] if 'OBS_PATH' in self._config else PARENT_DIR + "/res/obstacle-25.SLDASM/urdf/obstacle-25.SLDASM.urdf"

        self._ROBOT_NAME = self._config['ROBOT_NAME'] if 'ROBOT_NAME' in self._config else self._defaultConfig['ROBOT_NAME']

        self._OBS_WIDTH = self._config['OBS_WIDTH'] if 'OBS_WIDTH' in self._config else 0.25
        self._OBS_LENGTH = self._config['OBS_LENGTH'] if 'OBS_LENGTH' in self._config else 0.25
        self._OBS_HEIGHT = self._config['OBS_HEIGHT'] if 'OBS_HEIGHT' in self._config else 0.25

        self._GX = int(np.ptp(np.array(self._ex)) / self._OBS_WIDTH)
        self._GY = int(np.ptp(np.array(self._ey)) / self._OBS_LENGTH)
        self._GZ = int(np.ptp(np.array(self._ez)) / self._OBS_HEIGHT)

        # init log
        logging.basicConfig(format='%(asctime)s-%(levelname)s: %(message)s', filename=self._LOGGING_PATH, level=logging.INFO)
        logging.basicConfig(format='%(asctime)s-%(levelname)s: %(message)s', filename=self._LOGGING_PATH, level=logging.DEBUG)
        logging.basicConfig(format='%(asctime)s-%(levelname)s: %(message)s', filename=self._LOGGING_PATH, level=logging.WARNING)
        logging.basicConfig(format='%(asctime)s-%(levelname)s: %(message)s', filename=self._LOGGING_PATH, level=logging.ERROR)


    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        assert self.action_space.contains(action), "%r (%s) invalid"%(action, type(action))
        reward = 0
        done = False
        info = {}

        success = self._updateRobot(action)

        if self._A.moduleCnt > self._max_modulecnt: # 判断模块个数是否超过最大值
            logging.info("module count exceed the max count")
            reward = -10
            done = True
        elif self._A.checkEEAttached(): # 判断是否添加末端执行器
            passEvaluation = self._IK()
            logging.info("IK result: " + str(passEvaluation))
            if passEvaluation:
                reward = 10 
                pos = self._target_pos
                obs = self._obstacle_matrix
                logging.info(f'pos: {pos}\nobs: {obs}')
            else:
                reward = 0
            # reward += -1 * self._config['W_J'] * self._A.jointNum + -1 * self._config['W_M] * self._A.totalMass
            # reward = int(passEvaluation)
            done = True
        else: # 非终止状态
            # print("module count not exceed the max count")
            if success:
                # reward = -1 * self._config['W_J'] * self._A.jointNum + -1 * self._config['W_M] * self._A.totalMass
                # reward = 0 
                info = { 'action_space': self._A.getAttachableSubModuleActions() }
                reward =  -1 * self._w_j if self._A.checkJointAttached() else 0
                reward += -1 * self._w_m * self._A.attachedMass# TODO: 调整
                # reward += -1 * self._config['W_J'] * self._A.jointNum + -1 * self._config['W_M] * self._A.totalMass
                done = False
            else:
                # raise("add module failed")
                info = { 'action_space': self._A.getAttachableSubModuleActions() }
                reward = -10
                # done = True
                done = False


        return self._state(), reward, done, info

    def reset(self):
        logging.info("reset simulation")
        if self._physics_client_id < 0:
            if self._renders:
                self._p = bc.BulletClient(connection_mode=p2.GUI)
            else:
                self._p = bc.BulletClient()
            self._physics_client_id = self._p._client

        p = self._p
        # p.configureDebugVisualizer(p2.COV_ENABLE_GUI,0)
        # p.configureDebugVisualizer(p2.COV_ENABLE_WIREFRAME,1)
        # 重置仿真环境
        self._clearEnv()
        p.resetSimulation()
        # 先去除地面
        # self._plane_id = p2.loadURDF(Plane_Path, useFixedBase=True, basePosition=[0, 0, 0], flags = p2.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)

        # 随机初始化观测值&环境&返回观测值
        self._obstacle_matrix, self._target_pos = self._random_gen_obs_target(self._OBS_PATH, 
                                        obs_width=self._OBS_WIDTH, obs_length=self._OBS_LENGTH, obs_height=self._OBS_HEIGHT,
                                        ex=self._ex, ey=self._ey, ez=self._ez,
                                        obs_prob=self._obs_prob)

        return self._state()
        # return (self._A.getModuleTypeList(self._config['MAX_MODULECNT']), self._target_pos, self._obstacle_matrix)

    
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
        pos_z += np.random.uniform(0, obs_height)
        target = np.array([pos_x, pos_y, pos_z])

        self._target_id = p2.addUserDebugPoints([target], [[1, 1, 0]], 10)

        # 机器人基座的位置也要保持为0，需要将机器人的位置转换为ob_matrix的索引, 周围的障碍物也要保持为0
        robot_pos = [self._config['ROBOT_POS'][0] - ex[0], self._config['ROBOT_POS'][1] - ey[0], self._config['ROBOT_POS'][2] - ez[0]]
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
        if self._visualization:
            if self._robot_id is not None:
                self._robot_cache_id = self._robot_id
        else:
            if self._robot_id is not None:
                self._p.removeBody(self._robot_id)
                self._robot_id = None
                self._joint_info = list()

        success = self._A.addModule(self._MODULETYPE_LIST[action])

        if self._visualization:
            exportPath = self._A.exportURDF(self._ROBOT_PATH, self._ROBOT_NAME)
            p2.setPhysicsEngineParameter(enableFileCaching=0, physicsClientId=self._physics_client_id)
            with HiddenOutputs(): 
                self._robot_id = p2.loadURDF(exportPath, useFixedBase=True, flags=p2.URDF_MERGE_FIXED_LINKS | p2.URDF_USE_SELF_COLLISION)
            p2.setPhysicsEngineParameter(enableFileCaching=1, physicsClientId=self._physics_client_id)
            if self._robot_cache_id is not None:
                p2.removeBody(self._robot_cache_id)
                self._robot_cache_id = None


        # joint_num = p2.getNumJoints(self._robot_id, self._physics_client_id)
        # for i in range(joint_num):
        #     self._joint_info.append(p2.getJointInfo(self._robot_id, i, self._physics_client_id))

        return success
    
    def _IK(self):
        p = self._p

        # 需要有两个关节
        logging.info(self._A.getModuleTypeList())
        ee_link_id = self._A.jointNum - 1
        # for i in range(ee_link_id+1):
        #     logging.info(p2.getLinkState(self._robot_id, i, computeForwardKinematics=True) )
        if ee_link_id < 0:
            return False
        logging.info("ee_link_id: {0}, robot_id: {1}".format(ee_link_id, self._robot_id))

        if not self._visualization:
            exportPath = self._A.exportURDF(self._ROBOT_PATH, self._ROBOT_NAME)
            p2.setPhysicsEngineParameter(enableFileCaching=0, physicsClientId=self._physics_client_id)
            with HiddenOutputs(): 
                self._robot_id = p2.loadURDF(exportPath, useFixedBase=True, flags=p2.URDF_MERGE_FIXED_LINKS | p2.URDF_USE_SELF_COLLISION)
            p2.setPhysicsEngineParameter(enableFileCaching=1, physicsClientId=self._physics_client_id)

        target_angle = p2.calculateInverseKinematics(self._robot_id, ee_link_id, self._target_pos, maxNumIterations=400)

        done = False
        success = True
        c = 0
        while True:
            cnt = 0
            for angle in target_angle:
                if self._visualization:
                    p2.setJointMotorControl2(self._robot_id, cnt, controlMode=p2.POSITION_CONTROL, targetPosition=angle, force=0.1, maxVelocity=0.01)
                    # p2.setJointMotorControl2(self._robot_id, cnt, controlMode=p2.POSITION_CONTROL, targetPosition=angle, maxVelocity=0.01)
                else:
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

            c += 1  
            if c > 1000 and not self._visualization:
                logging.info('IK计算次数过多')
                done = True
                success = False
                break
            if done:
                break
        
        # linkState[0], linkState[1] # p_EE, n_EE
        if success:
            success = self._check_distance(np.array(linkState[0]), self._target_pos, self._epsilon_p)

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
        if self._robot_cache_id is not None:
            p.removeBody(self._robot_cache_id)
            self._robot_cache_id = None
        if self._robot_id is not None:
            p.removeBody(self._robot_id)
            self._robot_id = None

        del self._A
        self._A = Arrangement()
        del self._joint_info
        self._joint_info = list()

        # 重置障碍物
        if len(self._obstacle_ids) > 0:
            for obs_id in self._obstacle_ids:
                p.removeBody(obs_id)
            del self._obstacle_ids
            self._obstacle_ids = list()

        # 重置目标
        if self._target_id is not None: 
            p.removeUserDebugItem(self._target_id) 
            self._target_id = None

    def _state(self):
        A = np.array(self._A.getModuleTypeList(self._max_modulecnt))
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
            print(v)
    else:
        print(env.observation_space.shape)
    print(env.action_space.shape)
    env.action_space.shape

    state = (env.reset())
    print(state['A'].shape, state['T'].shape, state['O'].shape)

    env.close()