from copy import deepcopy

from urdfGenerator.Enums import UnitType, ModuleType, UnitOrder
from urdfGenerator.Module import Module

# 标准模块
BaseLModuleCnt = 0
BaseLModule = Module("BaseModule", [UnitType.BASEL, UnitType.CONNECTORL])

JointLModuleCnt = 0
JointLModule = Module("JointModule", [UnitType.JOINTL, UnitType.CONNECTORL])

StraightLinkLModuleCnt = 0
StraightLinkLModule = Module("StraightLinkModule", [UnitType.STRAIGHTLINKL])

CornerLinkLModuleCnt = 0
CornerLinkLModule = Module("CornerLinkModule", [UnitType.CONNECTORL, UnitType.CORNERLINKL, UnitType.CONNECTORL])

EndEffectorLModuleCnt = 0
EndEffectorLModule = Module("End1Module", [UnitType.ENDEFFECTORL])

EndEffectorL2ModuleCnt = 0
EndEffectorL2Module = Module("End2Module", [UnitType.ENDEFFECTORL])

def generateModule(moduleType, order=UnitOrder.NORMAL):
    global BaseLModuleCnt, JointLModuleCnt, StraightLinkLModuleCnt, CornerLinkLModuleCnt, EndEffectorLModuleCnt, EndEffectorL2ModuleCnt
    if moduleType == ModuleType.BASEL:
        module = deepcopy(BaseLModule) if order == UnitOrder.NORMAL else deepcopy(BaseLModule).reverse()
        module.set_name("BaseLModule" + str(BaseLModuleCnt))
        BaseLModuleCnt += 1
    elif moduleType == ModuleType.JOINTL:
        module = deepcopy(JointLModule) if order == UnitOrder.NORMAL else deepcopy(JointLModule).reverse()
        module.set_name("JointLModule" + str(JointLModuleCnt))
        JointLModuleCnt += 1
    elif moduleType == ModuleType.STRAIGHTLINKL:
        module = deepcopy(StraightLinkLModule) if order == UnitOrder.NORMAL else deepcopy(StraightLinkLModule).reverse()
        module.set_name("StraightLinkLModule" + str(StraightLinkLModuleCnt))
        StraightLinkLModuleCnt += 1
    elif moduleType == ModuleType.CORNERLINKL:
        module = deepcopy(CornerLinkLModule) if order == UnitOrder.NORMAL else deepcopy(CornerLinkLModule).reverse()
        module.set_name("CornerLinkLModule" + str(CornerLinkLModuleCnt))
        CornerLinkLModuleCnt += 1
    elif moduleType == ModuleType.ENDEFFECTORL:
        module = deepcopy(EndEffectorL2Module) if order == UnitOrder.NORMAL else deepcopy(EndEffectorL2Module).reverse()
        module.set_name("EndEffectorL2Module" + str(EndEffectorL2ModuleCnt))
        EndEffectorL2ModuleCnt += 1
    # elif moduleType == ModuleType.ENDEFFECTORL:
    #     module = deepcopy(EndEffectorLModule) if order == UnitOrder.NORMAL else deepcopy(EndEffectorLModule).reverse()
    #     module.set_name("EndEffectorLModule" + str(EndEffectorLModuleCnt))
    #     EndEffectorLModuleCnt += 1
    else:
        raise Exception("moduleType not found")

    return module


# 测试
if __name__ == "__main__":

    m = generateModule(ModuleType.ENDEFFECTORL)

    print(m)
    # unit7 = InputUnit + CornerLinkLUnit + InputUnit
    # unit7 = StraightLinkUnitL + (InputUnit.reverse() + JointTUnit.reverse())

    # import os
    # dir = os.path.dirname(os.path.realpath(__file__))
    # with open(dir + '/unit7.urdf', 'w') as f:
    #     f.write(unit7.__str__())