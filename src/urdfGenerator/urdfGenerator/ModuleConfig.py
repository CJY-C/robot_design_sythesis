from urdfGenerator import register
register.PathRegister.add_path('/home/masa/learning/rl/undergraduate/cjy/robot_design_sythesis/src/RobotConfigDesignEnv/robotConfigDesign/res/config.json')
from copy import deepcopy

from urdfGenerator.Enums import UnitType, ModuleType, Order, MountingAngle, LinkType
from urdfGenerator.Module import Module
from urdfGenerator.utils import getConfig


# 拿到配置
config = getConfig()
modules_config = config['modules']

modules = {}
moduleTypeList = []
moduleSubTypeDict = {}


# 导入所有的模块
for module_config in modules_config:
    module_name = module_config["name"]
    module_id = module_config["id"]

    if not (module_name == "NONE"):
        module_components = list(map(lambda c: getattr(UnitType, c), module_config["components"]))
        # print(module_components)

        module = Module(module_name, module_components)
        if "mountingAngle" in module_config:
            angle = getattr(MountingAngle, module_config["mountingAngle"])
            module.rotate_head(angle)

        modules[module_name] = {
            'module': module,
            'id': module_id,
            'count': 0,
        }

    setattr(ModuleType, module_name, module_id)
    moduleTypeList.append(getattr(ModuleType, module_name))
    moduleSubTypeDict[moduleTypeList[-1]] = {
        "submodulesI": module_config["submodulesI"] if "submodulesI" in module_config else None,
        "submodulesO": module_config["submodulesO"] if "submodulesO" in module_config else None,
    }

# 将moduleSubTypeDict中的字符串转换为ModuleType
for moduleType in moduleTypeList:
    if moduleSubTypeDict[moduleType]["submodulesI"] is not None:
        moduleSubTypeDict[moduleType]["submodulesI"] = list(map(lambda s: getattr(ModuleType, s), moduleSubTypeDict[moduleType]["submodulesI"]))
    if moduleSubTypeDict[moduleType]["submodulesO"] is not None:
        moduleSubTypeDict[moduleType]["submodulesO"] = list(map(lambda s: getattr(ModuleType, s), moduleSubTypeDict[moduleType]["submodulesO"]))
# print(moduleSubTypeDict)

def getAttachableSubModuleActions(moduleType:ModuleType, linkType:LinkType) -> list:
    '''
    根据moduleType和linkType获取可连接的submodules

    Args:
        moduleType: moduleType类型，表示当前module的类型
        linkType: LinkType类型，表示当前module的连接类型

    Returns:
        一个list，表示可连接的submodules
    '''
    submodules = moduleSubTypeDict[moduleType]["submodulesI"] if linkType == LinkType.INPUT else moduleSubTypeDict[moduleType]["submodulesO"]
    mtl = getModuleTypeList()
    if submodules is not None:
        submodules = list(map(lambda s: mtl.index(s), submodules))
    else:
        submodules = []
    return submodules

def moduleType2action(moduleType:ModuleType) -> int:
    '''
    将moduleType转换为action

    Args:
        moduleType: moduleType类型，表示当前module的类型

    Returns:
        一个int，表示moduleType对应的action
    '''
    return getModuleTypeList().index(moduleType)

def generateModule(moduleType:ModuleType, order=Order.NORMAL) -> module:
    '''
    生成一个module对象

    Args:
        moduleType: moduleType类型，表示要生成的module的类型
        order: Order类型，表示要生成的module的方向

    Returns:
        一个module对象
    '''
    for name, m in modules.items():
        if m['id'] == moduleType:
            module = deepcopy(m['module']) if order == Order.NORMAL else deepcopy(m['module']).reverse()
            module.set_name(name + str(m['count']))
            m['count'] += 1
            return module


# # 标准模块
# BaseLModuleCnt = 0
# BaseLModule = Module("BLM", [UnitType.BASEL, UnitType.CONNECTORL])

# BaseMModuleCnt = 0
# BaseMModule = Module("BMM", [UnitType.BASEM, UnitType.CONNECTORM])

# JointLModuleCnt = 0
# JointLModule = Module("JLM", [UnitType.JOINTL, UnitType.CONNECTORL])
# JointLModule.rotate_head(MountingAngle.ONEEIGHTY)

# JointMModuleCnt = 0
# JointMModule = Module("JMM", [UnitType.JOINTM, UnitType.CONNECTORM])
# # JointMModule.rotate_head(MountingAngle.ONEEIGHTY)

# JointSModuleCnt = 0
# JointSModule = Module("JSM", [UnitType.JOINTS, UnitType.CONNECTORS])
# JointSModule.rotate_head(MountingAngle.ONEEIGHTY)


# CornerLLModuleCnt = 0
# CornerLLModule = Module("CLLM", [UnitType.CONNECTORL, UnitType.CORNERLINKLL, UnitType.CONNECTORL])
# CornerLLModule.rotate_head(MountingAngle.ONEEIGHTY)

# CornerLMModuleCnt = 0
# CornerLMModule = Module("CLMM", [UnitType.CONNECTORM, UnitType.CORNERLINKLM, UnitType.CONNECTORL])
# CornerLMModule.rotate_head(MountingAngle.ONEEIGHTY)

# CornerMMModuleCnt = 0
# CornerMMModule = Module("CMM", [UnitType.CONNECTORM, UnitType.CORNERLINKMM, UnitType.CONNECTORM])
# CornerMMModule.rotate_head(MountingAngle.ONEEIGHTY)

# CornerMSModuleCnt = 0
# CornerMSModule = Module("CMSM", [UnitType.CONNECTORS, UnitType.CORNERLINKMS, UnitType.CONNECTORM])
# CornerMSModule.rotate_head(MountingAngle.ONEEIGHTY)


# StraightLinkLModuleCnt = 0
# StraightLinkLModule = Module("SLM", [UnitType.STRAIGHTLINKLL])

# StraightLinkLMModuleCnt = 0
# StraightLinkLMModule = Module("SLMM", [UnitType.STRAIGHTLINKLM])
# StraightLinkLMModule.rotate_head(MountingAngle.ONEEIGHTY)

# StraightLinkMModuleCnt = 0
# StraightLinkMModule = Module("SMM", [UnitType.STRAIGHTLINKMM])
# StraightLinkMModule.rotate_head(MountingAngle.ONEEIGHTY)

# StraightLinkMSModuleCnt = 0
# StraightLinkMSModule = Module("SMSM", [UnitType.STRAIGHTLINKMS])

# EndEffectorLModuleCnt = 0
# EndEffectorLModule = Module("EELM", [UnitType.CONNECTORL, UnitType.ENDEFFECTORL])

# EndEffectorMModuleCnt = 0
# EndEffectorMModule = Module("EEMM", [UnitType.CONNECTORM, UnitType.ENDEFFECTORM])

# EndEffectorSModuleCnt = 0
# EndEffectorSModule = Module("EESM", [UnitType.CONNECTORS, UnitType.ENDEFFECTORS])


def getModuleTypeList():
    return moduleTypeList[1:]
    # return [
    #     ModuleType.BASEL, # 0
    #     ModuleType.JOINTL, ModuleType.CORNERLINKLL, ModuleType.CORNERLINKLM, ModuleType.STRAIGHTLINKLL, ModuleType.STRAIGHTLINKLM, ModuleType.ENDEFFECTORL,
    #     ModuleType.BASEM, # 7
    #     ModuleType.JOINTM, ModuleType.CORNERLINKMM, ModuleType.CORNERLINKMS, ModuleType.STRAIGHTLINKMM, ModuleType.STRAIGHTLINKMS, ModuleType.ENDEFFECTORM, 
    #     ModuleType.JOINTS, ModuleType.ENDEFFECTORS
    # ]

# def generateModule(moduleType, order=Order.NORMAL):
    # global BaseLModuleCnt, JointLModuleCnt, CornerLLModuleCnt, StraightLinkLModuleCnt, EndEffectorLModuleCnt, \
    # JointMModuleCnt, CornerLMModuleCnt, StraightLinkMModuleCnt, BaseMModuleCnt, EndEffectorMModuleCnt, StraightLinkLMModuleCnt, CornerMMModuleCnt,\
    # CornerMSModuleCnt, JointSModuleCnt, EndEffectorSModuleCnt, StraightLinkMSModuleCnt
    # if moduleType == ModuleType.BASEL:
    #     module = deepcopy(BaseLModule) if order == Order.NORMAL else deepcopy(BaseLModule).reverse()
    #     module.set_name("BLM" + str(BaseLModuleCnt))
    #     BaseLModuleCnt += 1
    # elif moduleType == ModuleType.BASEM:
    #     module = deepcopy(BaseMModule) if order == Order.NORMAL else deepcopy(BaseMModule).reverse()
    #     module.set_name("BMM" + str(BaseMModuleCnt))
    #     BaseMModuleCnt += 1
    # elif moduleType == ModuleType.JOINTL:
    #     module = deepcopy(JointLModule) if order == Order.NORMAL else deepcopy(JointLModule).reverse()
    #     module.set_name("JLM" + str(JointLModuleCnt))
    #     JointLModuleCnt += 1
    # elif moduleType == ModuleType.JOINTM:
    #     module = deepcopy(JointMModule) if order == Order.NORMAL else deepcopy(JointMModule).reverse()
    #     module.set_name("JMM" + str(JointMModuleCnt))
    #     JointMModuleCnt += 1
    # elif moduleType == ModuleType.JOINTS:
    #     module = deepcopy(JointSModule) if order == Order.NORMAL else deepcopy(JointSModule).reverse()
    #     module.set_name("JSM" + str(JointSModuleCnt))
    #     JointSModuleCnt += 1
    # elif moduleType == ModuleType.CORNERLINKLL:
    #     module = deepcopy(CornerLLModule) if order == Order.NORMAL else deepcopy(CornerLLModule).reverse()
    #     module.set_name("CLLM" + str(CornerLLModuleCnt))
    #     CornerLLModuleCnt += 1
    # elif moduleType == ModuleType.CORNERLINKLM:
    #     module = deepcopy(CornerLMModule) if order == Order.NORMAL else deepcopy(CornerLMModule).reverse()
    #     module.set_name("CLMM" + str(CornerLMModuleCnt))
    #     CornerLMModuleCnt += 1
    # elif moduleType == ModuleType.CORNERLINKMM:
    #     module = deepcopy(CornerMMModule) if order == Order.NORMAL else deepcopy(CornerMMModule).reverse()
    #     module.set_name("CMM" + str(CornerMMModuleCnt))
    #     CornerMMModuleCnt += 1
    # elif moduleType == ModuleType.CORNERLINKMS:
    #     module = deepcopy(CornerMSModule) if order == Order.NORMAL else deepcopy(CornerMSModule).reverse()
    #     module.set_name("CMSM" + str(CornerMSModuleCnt))
    #     CornerMSModuleCnt += 1
    # elif moduleType == ModuleType.STRAIGHTLINKLL:
    #     module = deepcopy(StraightLinkLModule) if order == Order.NORMAL else deepcopy(StraightLinkLModule).reverse()
    #     module.set_name("SLM" + str(StraightLinkLModuleCnt))
    #     StraightLinkLModuleCnt += 1
    # elif moduleType == ModuleType.STRAIGHTLINKLM:
    #     module = deepcopy(StraightLinkLMModule) if order == Order.NORMAL else deepcopy(StraightLinkLMModule).reverse()
    #     module.set_name("SLMM" + str(StraightLinkLMModuleCnt))
    #     StraightLinkLMModuleCnt += 1
    # elif moduleType == ModuleType.STRAIGHTLINKMM:
    #     module = deepcopy(StraightLinkMModule) if order == Order.NORMAL else deepcopy(StraightLinkMModule).reverse()
    #     module.set_name("SMM" + str(StraightLinkMModuleCnt))
    #     StraightLinkMModuleCnt += 1
    # elif moduleType == ModuleType.STRAIGHTLINKMS:
    #     module = deepcopy(StraightLinkMSModule) if order == Order.NORMAL else deepcopy(StraightLinkMSModule).reverse()
    #     module.set_name("SMSM" + str(StraightLinkMSModuleCnt))
    #     StraightLinkMSModuleCnt += 1
    # elif moduleType == ModuleType.ENDEFFECTORL:
    #     module = deepcopy(EndEffectorLModule) if order == Order.NORMAL else deepcopy(EndEffectorLModule).reverse()
    #     module.set_name("EELM" + str(EndEffectorLModuleCnt))
    #     EndEffectorLModuleCnt += 1
    # elif moduleType == ModuleType.ENDEFFECTORM:
    #     module = deepcopy(EndEffectorMModule) if order == Order.NORMAL else deepcopy(EndEffectorMModule).reverse()
    #     module.set_name("EEMM" + str(EndEffectorMModuleCnt))
    #     EndEffectorMModuleCnt += 1
    # elif moduleType == ModuleType.ENDEFFECTORS:
    #     module = deepcopy(EndEffectorSModule) if order == Order.NORMAL else deepcopy(EndEffectorSModule).reverse()
    #     module.set_name("EESM" + str(EndEffectorSModuleCnt))
    #     EndEffectorSModuleCnt += 1
    # else:
    #     raise Exception("moduleType not found")

    # return module

    


# def getAttachableSubModuleActions(moduleType, linkType):
#     subModuleActions = list()
#     if moduleType == ModuleType.NONE:
#         subModuleActions = [0, 7]
#     elif moduleType == ModuleType.BASEL:
#         subModuleActions = [1, 4, 5]
#     elif moduleType == ModuleType.BASEM:
#         subModuleActions = [8, 11, 12]
#     elif moduleType == ModuleType.JOINTL and linkType == LinkType.OUTPUT:
#         subModuleActions = [2, 3, 6]
#     elif moduleType == ModuleType.JOINTL and linkType == LinkType.INPUT:
#         subModuleActions = [1, 4, 5]
#     elif moduleType == ModuleType.JOINTM and linkType == LinkType.OUTPUT:
#         subModuleActions = [9, 10, 13]
#     elif moduleType == ModuleType.JOINTM and linkType == LinkType.INPUT:
#         subModuleActions = [8, 11, 12]
#     elif moduleType == ModuleType.JOINTS and linkType == LinkType.OUTPUT:
#         subModuleActions = [15]
#     elif moduleType == ModuleType.JOINTS and linkType == LinkType.INPUT:
#         subModuleActions = [14]
#     elif moduleType == ModuleType.CORNERLINKLL and linkType == LinkType.INPUT:
#         subModuleActions = [1, 4, 5]
#     elif moduleType == ModuleType.CORNERLINKLM:
#         subModuleActions = [8, 11, 12]
#     elif moduleType == ModuleType.CORNERLINKMM:
#         subModuleActions = [8, 11, 12]
#     elif moduleType == ModuleType.CORNERLINKMS:
#         subModuleActions = [14]
#     elif moduleType == ModuleType.STRAIGHTLINKLL:
#         subModuleActions = [1, 2, 3, 6]
#     elif moduleType == ModuleType.STRAIGHTLINKLM:
#         subModuleActions = [8, 9, 10, 13]
#     elif moduleType == ModuleType.STRAIGHTLINKMM:
#         subModuleActions = [8, 9, 10, 13]
#     elif moduleType == ModuleType.STRAIGHTLINKMS:
#         subModuleActions = [14, 15]
#     elif moduleType == ModuleType.ENDEFFECTORL or moduleType == ModuleType.ENDEFFECTORM or moduleType == ModuleType.ENDEFFECTORS:
#         subModuleActions = []
#     else:
#         raise Exception("moduleType not found")
    
#     return subModuleActions
        
        



# 测试
if __name__ == "__main__":

    b = generateModule(ModuleType.BASEL)
    m = generateModule(getModuleTypeList()[0])

    # m = getAttachableSubModuleActions(moduleType=ModuleType.JOINTL, linkType=LinkType.INPUT)

    print(moduleTypeList)
    print(b)

    for module in getModuleTypeList():
        print(module)
    
    print(moduleType2action(ModuleType.BASEM))
    print(list(map(moduleType2action, getModuleTypeList())))
    # unit7 = InputUnit + CornerLinkLUnit + InputUnit
    # unit7 = StraightLinkUnitL + (InputUnit.reverse() + JointTUnit.reverse())

    # import os
    # dir = os.path.dirname(os.path.realpath(__file__))
    # with open(dir + '/unit7.urdf', 'w') as f:
    #     f.write(unit7.__str__())