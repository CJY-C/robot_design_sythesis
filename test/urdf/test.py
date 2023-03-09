from urdfGenerator import Module, UnitType, UnitOrder, ModuleType
from urdfGenerator.Arrangement import Arrangement

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
    return a


if __name__ == '__main__':

    moduleTypeList = [ModuleType.BASEL, ModuleType.JOINTL, ModuleType.STRAIGHTLINKL, ModuleType.CORNERLINKL, ModuleType.ENDEFFECTORL]


    a1 = Arrangement()
    a1.addModule(ModuleType.BASEL)
    a1.addModule(ModuleType.JOINTL)
    a1.addModule(ModuleType.JOINTL)
    a1.addModule(ModuleType.STRAIGHTLINKL)
    a1.addModule(ModuleType.CORNERLINKL)
    a1.addModule(ModuleType.JOINTL)
    a1.addModule(ModuleType.STRAIGHTLINKL)
    a1.addModule(ModuleType.JOINTL)
    a1.addModule(ModuleType.JOINTL)
    a1.addModule(ModuleType.JOINTL)
    a1.addModule(ModuleType.ENDEFFECTORL)

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
