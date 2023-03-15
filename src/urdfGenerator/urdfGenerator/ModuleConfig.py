from copy import deepcopy

from urdfGenerator.Enums import UnitType, ModuleType, UnitOrder, MountingAngle
from urdfGenerator.Module import Module

# 标准模块
BaseLModuleCnt = 0
BaseLModule = Module("BLM", [UnitType.BASEL, UnitType.CONNECTORL])

BaseMModuleCnt = 0
BaseMModule = Module("BMM", [UnitType.BASEM, UnitType.CONNECTORM])

JointLModuleCnt = 0
JointLModule = Module("JLM", [UnitType.JOINTL, UnitType.CONNECTORL])
JointLModule.rotate_head(MountingAngle.ONEEIGHTY)

JointMModuleCnt = 0
JointMModule = Module("JMM", [UnitType.JOINTM, UnitType.CONNECTORM])
JointMModule.rotate_head(MountingAngle.ONEEIGHTY)

JointSModuleCnt = 0
JointSModule = Module("JSM", [UnitType.JOINTS, UnitType.CONNECTORS])
JointSModule.rotate_head(MountingAngle.ONEEIGHTY)


CornerLLModuleCnt = 0
CornerLLModule = Module("CLLM", [UnitType.CONNECTORL, UnitType.CORNERLINKLL, UnitType.CONNECTORL])
CornerLLModule.rotate_head(MountingAngle.ONEEIGHTY)

CornerLMModuleCnt = 0
CornerLMModule = Module("CLMM", [UnitType.CONNECTORM, UnitType.CORNERLINKLM, UnitType.CONNECTORL])
CornerLMModule.rotate_head(MountingAngle.ONEEIGHTY)

CornerMMModuleCnt = 0
CornerMMModule = Module("CMM", [UnitType.CONNECTORM, UnitType.CORNERLINKMM, UnitType.CONNECTORM])
CornerMMModule.rotate_head(MountingAngle.ONEEIGHTY)

CornerMSModuleCnt = 0
CornerMSModule = Module("CMSM", [UnitType.CONNECTORS, UnitType.CORNERLINKMS, UnitType.CONNECTORM])
CornerMSModule.rotate_head(MountingAngle.ONEEIGHTY)


StraightLinkLModuleCnt = 0
StraightLinkLModule = Module("SLM", [UnitType.STRAIGHTLINKLL])

StraightLinkLMModuleCnt = 0
StraightLinkLMModule = Module("SLMM", [UnitType.STRAIGHTLINKLM])
StraightLinkLMModule.rotate_head(MountingAngle.ONEEIGHTY)

StraightLinkMModuleCnt = 0
StraightLinkMModule = Module("SMM", [UnitType.STRAIGHTLINKMM])
StraightLinkMModule.rotate_head(MountingAngle.ONEEIGHTY)

StraightLinkMSModuleCnt = 0
StraightLinkMSModule = Module("SMSM", [UnitType.STRAIGHTLINKMS])

EndEffectorLModuleCnt = 0
EndEffectorLModule = Module("EELM", [UnitType.CONNECTORL, UnitType.ENDEFFECTORL])

EndEffectorMModuleCnt = 0
EndEffectorMModule = Module("EEMM", [UnitType.CONNECTORM, UnitType.ENDEFFECTORM])

EndEffectorSModuleCnt = 0
EndEffectorSModule = Module("EESM", [UnitType.CONNECTORS, UnitType.ENDEFFECTORS])


def getModuleTypeList():
    return [
        ModuleType.BASEL, ModuleType.BASEM, # 1
        ModuleType.JOINTL, ModuleType.JOINTM, ModuleType.JOINTS, # 4
        ModuleType.CORNERLINKLL, ModuleType.CORNERLINKLM, ModuleType.CORNERLINKMM, ModuleType.CORNERLINKMS, # 8
        ModuleType.STRAIGHTLINKLL, ModuleType.STRAIGHTLINKLM, ModuleType.STRAIGHTLINKMM, ModuleType.STRAIGHTLINKMS, # 12
        ModuleType.ENDEFFECTORL, ModuleType.ENDEFFECTORM, ModuleType.ENDEFFECTORS # 15
    ]

def generateModule(moduleType, order=UnitOrder.NORMAL):
    global BaseLModuleCnt, JointLModuleCnt, CornerLLModuleCnt, StraightLinkLModuleCnt, EndEffectorLModuleCnt, \
    JointMModuleCnt, CornerLMModuleCnt, StraightLinkMModuleCnt, BaseMModuleCnt, EndEffectorMModuleCnt, StraightLinkLMModuleCnt, CornerMMModuleCnt,\
    CornerMSModuleCnt, JointSModuleCnt, EndEffectorSModuleCnt, StraightLinkMSModuleCnt
    if moduleType == ModuleType.BASEL:
        module = deepcopy(BaseLModule) if order == UnitOrder.NORMAL else deepcopy(BaseLModule).reverse()
        module.set_name("BLM" + str(BaseLModuleCnt))
        BaseLModuleCnt += 1
    elif moduleType == ModuleType.BASEM:
        module = deepcopy(BaseMModule) if order == UnitOrder.NORMAL else deepcopy(BaseMModule).reverse()
        module.set_name("BMM" + str(BaseMModuleCnt))
        BaseMModuleCnt += 1
    elif moduleType == ModuleType.JOINTL:
        module = deepcopy(JointLModule) if order == UnitOrder.NORMAL else deepcopy(JointLModule).reverse()
        module.set_name("JLM" + str(JointLModuleCnt))
        JointLModuleCnt += 1
    elif moduleType == ModuleType.JOINTM:
        module = deepcopy(JointMModule) if order == UnitOrder.NORMAL else deepcopy(JointMModule).reverse()
        module.set_name("JMM" + str(JointMModuleCnt))
        JointMModuleCnt += 1
    elif moduleType == ModuleType.JOINTS:
        module = deepcopy(JointSModule) if order == UnitOrder.NORMAL else deepcopy(JointSModule).reverse()
        module.set_name("JSM" + str(JointSModuleCnt))
        JointSModuleCnt += 1
    elif moduleType == ModuleType.CORNERLINKLL:
        module = deepcopy(CornerLLModule) if order == UnitOrder.NORMAL else deepcopy(CornerLLModule).reverse()
        module.set_name("CLLM" + str(CornerLLModuleCnt))
        CornerLLModuleCnt += 1
    elif moduleType == ModuleType.CORNERLINKLM:
        module = deepcopy(CornerLMModule) if order == UnitOrder.NORMAL else deepcopy(CornerLMModule).reverse()
        module.set_name("CLMM" + str(CornerLMModuleCnt))
        CornerLMModuleCnt += 1
    elif moduleType == ModuleType.CORNERLINKMM:
        module = deepcopy(CornerMMModule) if order == UnitOrder.NORMAL else deepcopy(CornerMMModule).reverse()
        module.set_name("CMM" + str(CornerMMModuleCnt))
        CornerMMModuleCnt += 1
    elif moduleType == ModuleType.CORNERLINKMS:
        module = deepcopy(CornerMSModule) if order == UnitOrder.NORMAL else deepcopy(CornerMSModule).reverse()
        module.set_name("CMSM" + str(CornerMSModuleCnt))
        CornerMSModuleCnt += 1
    elif moduleType == ModuleType.STRAIGHTLINKLL:
        module = deepcopy(StraightLinkLModule) if order == UnitOrder.NORMAL else deepcopy(StraightLinkLModule).reverse()
        module.set_name("SLM" + str(StraightLinkLModuleCnt))
        StraightLinkLModuleCnt += 1
    elif moduleType == ModuleType.STRAIGHTLINKLM:
        module = deepcopy(StraightLinkLMModule) if order == UnitOrder.NORMAL else deepcopy(StraightLinkLMModule).reverse()
        module.set_name("SLMM" + str(StraightLinkLMModuleCnt))
        StraightLinkLMModuleCnt += 1
    elif moduleType == ModuleType.STRAIGHTLINKMM:
        module = deepcopy(StraightLinkMModule) if order == UnitOrder.NORMAL else deepcopy(StraightLinkMModule).reverse()
        module.set_name("SMM" + str(StraightLinkMModuleCnt))
        StraightLinkMModuleCnt += 1
    elif moduleType == ModuleType.STRAIGHTLINKMS:
        module = deepcopy(StraightLinkMSModule) if order == UnitOrder.NORMAL else deepcopy(StraightLinkMSModule).reverse()
        module.set_name("SMSM" + str(StraightLinkMSModuleCnt))
        StraightLinkMSModuleCnt += 1
    elif moduleType == ModuleType.ENDEFFECTORL:
        module = deepcopy(EndEffectorLModule) if order == UnitOrder.NORMAL else deepcopy(EndEffectorLModule).reverse()
        module.set_name("EELM" + str(EndEffectorLModuleCnt))
        EndEffectorLModuleCnt += 1
    elif moduleType == ModuleType.ENDEFFECTORM:
        module = deepcopy(EndEffectorMModule) if order == UnitOrder.NORMAL else deepcopy(EndEffectorMModule).reverse()
        module.set_name("EEMM" + str(EndEffectorMModuleCnt))
        EndEffectorMModuleCnt += 1
    elif moduleType == ModuleType.ENDEFFECTORS:
        module = deepcopy(EndEffectorSModule) if order == UnitOrder.NORMAL else deepcopy(EndEffectorSModule).reverse()
        module.set_name("EESM" + str(EndEffectorSModuleCnt))
        EndEffectorSModuleCnt += 1
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