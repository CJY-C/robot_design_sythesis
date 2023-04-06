from urdfGenerator import Module, UnitType, UnitOrder, ModuleType
from urdfGenerator.Arrangement import Arrangement
from urdfGenerator.ModuleConfig import getModuleTypeList
from urdfGenerator.UnitConfig import generateUnit

from odio_urdf import urdf_to_odio

from math import pi

import numpy as np

from random import randint


def genArrangement(moduleTypeList, moduleN):
    while True:
        ok = True
        a = Arrangement()
        ok = a.addModule(ModuleType.BASEL)
        for i in range(moduleN):
            ok = a.addModule(moduleTypeList[randint(0, len(moduleTypeList)-1)])
        ok = a.addModule(ModuleType.ENDEFFECTORL)
        if ok:
            break
        ok = a.addModule(ModuleType.ENDEFFECTORM)
        if ok:
            break
        ok = a.addModule(ModuleType.ENDEFFECTORS)
        if ok:
            break
    return a


def test1():
    moduleTypeList = getModuleTypeList()


    # a1 = Arrangement()
    # a1.addModule(ModuleType.BASEL)
    # a1.addModule(ModuleType.JOINTL)
    # a1.addModule(ModuleType.JOINTL)
    # a1.addModule(ModuleType.STRAIGHTLINKL)
    # a1.addModule(ModuleType.CORNERLINKL)
    # a1.addModule(ModuleType.JOINTL)
    # a1.addModule(ModuleType.STRAIGHTLINKL)
    # a1.addModule(ModuleType.JOINTL)
    # a1.addModule(ModuleType.JOINTL)
    # a1.addModule(ModuleType.JOINTL)
    # a1.addModule(ModuleType.ENDEFFECTORL)

    mj = None

    import pybullet as p

    client = p.connect(p.GUI)
    p.setPhysicsEngineParameter(enableFileCaching=0, physicsClientId=client)
    # ground = p.loadURDF("/home/masa/learning/rl/undergraduate/cjy/robot_design_sythesis/test/urdf/plane.urdf")
    p.setGravity(0, 0, -10, physicsClientId=client)

    p_id = None


    bt = p.addUserDebugParameter( "random_generate", 1, 0, 1)
    bt_num = 0
    while True:
        new_bt_num = p.readUserDebugParameter(bt)
        if new_bt_num != bt_num:
            if p_id is not None:
                p.removeBody(p_id) 
                p.removeAllUserParameters()
                bt = p.addUserDebugParameter( "random_generate", 1, 0, 1)
                new_bt_num = p.readUserDebugParameter(bt)
            bt_num = new_bt_num
            moduleN = randint(8, 18)
            a = genArrangement(moduleTypeList, moduleN)
            a.exportURDF(__file__, 'arrangement1')
            p_id = p.loadURDF( '/home/masa/learning/rl/undergraduate/cjy/robot_design_sythesis/test/urdf/arrangement1.urdf', useFixedBase=True, flags=p.URDF_MERGE_FIXED_LINKS)
            joint_num = p.getNumJoints(p_id, client)
            joint_info = list()
            motor_list = list()
            for i in range(joint_num):
                joint_info.append(p.getJointInfo(p_id, i, client))
                motor_list.append(p.addUserDebugParameter( f"motor{i}", -np.pi, np.pi, 0))
            mj = list(zip(motor_list[:], joint_info[:]))
        for motorIndex, jointIndex in mj:
            p.setJointMotorControl2( p_id, jointIndex[0], controlMode=p.POSITION_CONTROL,  targetPosition=p.readUserDebugParameter(motorIndex))
        p.stepSimulation()

    p.destroy(client)
    p.disconnect()

def test2():
    import os
    from urdfGenerator.UnitConfig import generateUnit
    from urdfGenerator.ModuleConfig import generateModule
    print("Unit class test")
    a = Arrangement()
    b = generateModule(ModuleType.BASEL)
    j = generateModule(ModuleType.CORNERLINKLM, UnitOrder.REVERSE)
    print(j.unitTypeList)
    j2 = generateModule(ModuleType.JOINTL)
    c = generateModule(ModuleType.CORNERLINKLL)

    a.addModule(ModuleType.BASEL)
    a.addModule(ModuleType.JOINTL)
    a.addModule(ModuleType.JOINTL)
    a.addModule(ModuleType.STRAIGHTLINKLM)
    a.addModule(ModuleType.JOINTM)
    a.addModule(ModuleType.CORNERLINKMM)
    a.addModule(ModuleType.STRAIGHTLINKMS)
    a.addModule(ModuleType.JOINTS)
    a.addModule(ModuleType.JOINTS)
    a.addModule(ModuleType.JOINTS)
    a.addModule(ModuleType.ENDEFFECTORS)

    # a.exportURDF(os.getcwd() + '/test/urdf', 'unit7')

    # import os
    # dir = os.path.dirname(os.path.realpath(__file__))
    # with open(dir + '/unit7.urdf', 'w') as f:
    #     f.write(a.__str__())

    # import os
    # import sys

    # # 将标准输出重定向到 Python 的空设备
    # sys.stdout = open(os.devnull, "w")

# 在此处写下您的 Python 代码，它将不会产生任何输出
    import sys

    # 将所有输出重定向到文件中
    sys.stdout = open('output.txt', 'w')
    sys.stderr = sys.stdout

    # 在此处写下您的 Python 代码
    print('Hello, world!')


    import logging
    import pybullet as p

    # 创建一个名为 "disable_all" 的日志记录器，并禁用所有日志记录
    logging.basicConfig(level=logging.CRITICAL, filename="disable_all.log")

    # 在此处写下您的 Python 代码，它将不会产生任何输出




    client = p.connect(p.GUI, options="--opengl3 --log-level=5")
    p.setPhysicsEngineParameter(enableFileCaching=0, physicsClientId=client)
    p_id = p.loadURDF( '/home/masa/learning/rl/undergraduate/cjy/robot_design_sythesis/test/urdf/unit7.urdf', useFixedBase=True, flags=p.URDF_MERGE_FIXED_LINKS | p.URDF_USE_SELF_COLLISION)
    # o_id = p.loadURDF('/home/masa/learning/rl/undergraduate/cjy/robot_design_sythesis/test/urdf/obstacle-25.SLDASM/urdf/obstacle-25.SLDASM.urdf', useFixedBase=True, basePosition=[-1, 0, 0])

    p.setGravity(0, 0, -10, physicsClientId=client)

    joint_num = p.getNumJoints(p_id, client)
    joint_info = list()
    motor_list = list()
    for i in range(joint_num):
        joint_info.append(p.getJointInfo(p_id, i, client))
        motor_list.append(p.addUserDebugParameter( f"motor{i}", -np.pi, np.pi, 0))
    
    
    # p.addUserDebugPoints([[-1, 0, 0]], [[1, 0, 0]], 10)

    # print (p.getNumJoints(p_id))
    # for i in joint_info:
    #     print(i)
    # print(p.getLinkState(p_id, 5))
    ori = p.getQuaternionFromEuler([0,0,0])
    print(ori)
    print(p.getLinkState(p_id, 5, computeForwardKinematics=True, computeLinkVelocity=True))
    # print(p.getLinkState(p_id, 5))
    target_angle = p.calculateInverseKinematics(p_id, 5, [-0.5, 0, 0], ori, maxNumIterations=1000, residualThreshold=0.0001)
    print(target_angle)

    show_once = True

    done = True
    # while True:
    #     # mj = list(zip(motor_list[:], joint_info[:]))
    #     # for motorIndex, jointIndex in mj:
    #     #     p.setJointMotorControl2( p_id, jointIndex[0], controlMode=p.POSITION_CONTROL,  targetPosition=p.readUserDebugParameter(motorIndex))
    #     cnt = 0
    #     for angle in target_angle:
    #         p.setJointMotorControl2(p_id, cnt, controlMode=p.POSITION_CONTROL, targetPosition=angle)
    #         cnt += 1

    #     # print(p.getLinkState(p_id, 4))
    #     p.stepSimulation()

    #     print(p.getLinkState(p_id, 5, computeForwardKinematics=True, computeLinkVelocity=True))
    #     linkState = p.getLinkState(p_id, 5, computeForwardKinematics=True, computeLinkVelocity=True)

    #     for i in linkState[-2]:
    #         if np.abs(i) > 1e-13:
    #             done = False
    #             break
    #         done = True
    #     if done:
    #         break
    while True:
        pass

    p.disconnect()

    print('test')

    # 关闭日志记录器，将缓冲区中的内容写入文件
    logging.shutdown()
    # # 恢复标准输出
    # sys.stdout = sys.__stdout__

# TODO: mask
def random_gen_obs_target(obs_filepath, obs_length, obs_width, obs_height, ex, ey, ez, obs_prob):
    import pybullet as p
    client = p.connect(p.GUI)
    p_id = p.loadURDF( '/home/masa/learning/rl/undergraduate/cjy/robot_design_sythesis/test/urdf/unit7.urdf', useFixedBase=True, flags=p.URDF_MERGE_FIXED_LINKS | p.URDF_USE_SELF_COLLISION)

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
    target = [tx, ty, tz] # 先不考虑姿态

    # 将target的位置转换为世界坐标系下的位置
    pos_x = (tx + ex[0] * gx/lx) / gx * lx + obs_length/2
    pos_y = (ty + ey[0] * gy/ly) / gy * ly + obs_width/2
    pos_z = (tz + ez[0] * gz/lz) / gz * lz
    # 在所在obs方盒中随机生成一个位置
    pos_x += np.random.uniform(-obs_length/2, obs_length/2)
    pos_y += np.random.uniform(-obs_width/2, obs_width/2)
    pos_z += np.random.uniform(0, obs_height)
    # pos_z += np.random.uniform(-obs_height/2, obs_height/2)
    target = [pos_x, pos_y, pos_z]

    target_id = p.addUserDebugPoints([target], [[1, 1, 0]], 10)

    # 机器人基座的位置也要保持为0，需要将机器人的位置转换为ob_matrix的索引, 周围的障碍物也要保持为0
    robot_pos = [0, 0, 0]
    robot_pos = [robot_pos[0] - ex[0], robot_pos[1] - ey[0], robot_pos[2] - ez[0]]
    robot_pos = [int(robot_pos[0] / lx * gx), int(robot_pos[1] / ly * gy), int(robot_pos[2] / lz * gz)]
    ob_matrix[robot_pos[0]][robot_pos[1]][robot_pos[2]] = 0
    ob_matrix[robot_pos[0]-1][robot_pos[1]][robot_pos[2]] = 0
    ob_matrix[robot_pos[0]][robot_pos[1]-1][robot_pos[2]] = 0
    ob_matrix[robot_pos[0]-1][robot_pos[1]-1][robot_pos[2]] = 0



    for x in range(gx):
        for y in range(gy):
            for z in range(gz):
                if ob_matrix[x][y][z] == 1:
                    pos_x = (x + ex[0] * gx/lx) / gx * lx + obs_length/2
                    pos_y = (y + ey[0] * gy/ly) / gy * ly + obs_width/2
                    pos_z = (z + ez[0] * gz/lz) / gz * lz
                    obs_id = p.loadURDF(obs_filepath, useFixedBase=True, basePosition=[pos_x, pos_y, pos_z], flags = p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)


    # grid_size = 6
    # obs_prob = 1
    # ob_matrix = np.random.choice([0, 1], size=(grid_size, grid_size, grid_size//2), p=[1 - obs_prob, obs_prob])

    # for x in range(grid_size):
    #     for y in range(grid_size):
    #         for z in range(grid_size//2):
    #             if ob_matrix[x][y][z] == 1:
    #                 pos_x = (x - grid_size/2) / grid_size * 1.5 + 0.125
    #                 pos_y = (y - grid_size/2) / grid_size * 1.5 + 0.125
    #                 pos_z = z / grid_size  * 1.5
                    # obs_id = p.loadURDF('/home/masa/learning/rl/undergraduate/cjy/robot_design_sythesis/test/urdf/obstacle-25.SLDASM/urdf/obstacle-25.SLDASM.urdf', useFixedBase=True, basePosition=[pos_x, pos_y, pos_z])
                    # obs_id = p.loadURDF(obs_filepath, useFixedBase=True, basePosition=[pos_x, pos_y, pos_z], flags = p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
    while True:
        p.stepSimulation()

    return ob_matrix, target

# 先不搞
def show_matrix(ob_matrix=None):
    import numpy as np
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

    # ob_matrix = np.random.choice([0, 1], size=(8, 8, 4), p=[1 - 0.5, 0.5])
    # 随机生成数据
    N = 1000
    x = np.random.randn(N)
    y = np.random.randn(N)
    z = np.random.randn(N)

    # 绘制3D散点图
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(x, y, z, s=5, c='blue', alpha=0.5)

    # 设置坐标轴范围
    ax.set_xlim([-5, 5])
    ax.set_ylim([-5, 5])
    ax.set_zlim([-5, 5])

    # 设置坐标轴标签
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # 显示图形
    plt.show()

def test_ob():
    import gym
    import numpy as np
    observation_space = gym.spaces.Dict({
        'A': gym.spaces.MultiDiscrete(np.array([len(getModuleTypeList()) for x in range(20)])),
        'obstacle_matrix': gym.spaces.MultiBinary((8, 8, 4)),
        'target_pos': gym.spaces.Box(low=np.float32(-1), high=np.float32(1), shape=(3, ))
    })
    print(observation_space['A'])

def ur(env):
    s, r, d, _ = env.step(0)
    print(f'r: {r}, d: {d == False}')
    # sleep(1)
    s, r, d, _ = env.step(1)
    print(f'r: {r}, d: {d == False}')
    # sleep(1)
    s, r, d, _ = env.step(1)
    print(f'r: {r}, d: {d == False}')
    # sleep(1)
    s, r, d, _ = env.step(5)
    print(f'r: {r}, d: {d == False}')
    # sleep(1)
    s, r, d, _ = env.step(8)
    print(f'r: {r}, d: {d == False}')
    # sleep(1)
    s, r, d, _ = env.step(9)
    print(f'r: {r}, d: {d == False}')
    # sleep(1)
    s, r, d, _ = env.step(12)
    print(f'r: {r}, d: {d == False}')
    # sleep(1)
    s, r, d, _ = env.step(14)
    print(f'r: {r}, d: {d == False}')
    # sleep(1)
    s, r, d, _ = env.step(14)
    print(f'r: {r}, d: {d == False}')
    # sleep(1)
    s, r, d, _ = env.step(14)
    print(f'r: {r}, d: {d == False}')
    # sleep(1)
    s, r, d, _ = env.step(15)
    print(f's: {s}, r: {r}, d: {d == False}')
    # sleep(5)
    return d

def failed(env):
    s, r, d, _ = env.step(7)
    print(f'r: {r}, d: {d == False}')
    # sleep(1)
    s, r, d, _ = env.step(11)
    print(f'r: {r}, d: {d == False}')
    # sleep(1)
    s, r, d, _ = env.step(8)
    print(f'r: {r}, d: {d == False}')
    # sleep(1)
    s, r, d, _ = env.step(9)
    print(f'r: {r}, d: {d == False}')
    # sleep(1)
    s, r, d, _ = env.step(8)
    print(f'r: {r}, d: {d == False}')
    # sleep(1)
    s, r, d, _ = env.step(11)
    print(f'r: {r}, d: {d == False}')
    # sleep(1)
    s, r, d, _ = env.step(13)
    print(f'r: {r}, d: {d == False}')
    # sleep(5)
    return d

def testEnv():
    # gym 走起
    import gym
    from robotConfigDesign.envs import RobotConfigDesignEnv
    from time import sleep

    env = gym.make('RobotConfigDesign-v0')

    env.render()
    env.reset()

    for _ in range(5):
        d = failed(env)
        if d:
            env.reset()
        d = ur(env)
        if d:
            env.reset()
    # sleep(1)
    # for _ in range(100):
    #     env.render()
    #     s, r, d, _ = env.step(env.action_space.sample())
    #     if d:
    #         env.reset()

    while True:
        pass
    # env.close()


def testLog():
    import logging
    logging.basicConfig(filename='output.log', level=logging.INFO)

    # 在此处写下您的 Python 代码

    logging.info('程序输出信息')

def testAllLog():
    import sys

    # 将所有输出重定向到文件中
    sys.stdout = open('output.txt', 'w')
    sys.stderr = sys.stdout

    # 在此处写下您的 Python 代码
    print('Hello, world!')

def classAndUrdf():
    unit1 = generateUnit(UnitType.BASEL)
    unit2 = generateUnit(UnitType.JOINTL)

    a = Arrangement()
    a.addModule(ModuleType.BASEL)
    a.addModule(ModuleType.JOINTL)
    a.addModule(ModuleType.JOINTL)
    a.addModule(ModuleType.STRAIGHTLINKLM)
    a.addModule(ModuleType.JOINTM)
    a.addModule(ModuleType.ENDEFFECTORM)

    unit3 = unit1 + unit2

    import os
    dir = os.path.dirname(os.path.realpath(__file__))
    exportPath = dir + '/'+'a.urdf'
    with open(exportPath, 'w') as f:
        f.write(str(a))
    
    # a = urdf_to_odio(str(unit1))






if __name__ == '__main__':
    # test1()
    # test2()
    obs_filepath = '/home/masa/learning/rl/undergraduate/cjy/robot_design_sythesis/src/RobotConfigDesignEnv/robotConfigDesign/res/obstacle-25.SLDASM/urdf/obstacle-25.SLDASM.urdf'
    random_gen_obs_target(obs_filepath=obs_filepath,
                    obs_width=0.25, obs_height=0.25, obs_length=0.25,
                    ex=(-1, 1), ey=(-1, 1), ez=(0, 1), 
                    obs_prob=0.005
                   )
    # show_matrix()
    # test_ob()
    # testEnv()
    # testLog()
    # testAllLog()
    # classAndUrdf()
