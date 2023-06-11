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


def getModuleTypeList():
    return moduleTypeList[1:]


# 测试
if __name__ == "__main__":

    b = generateModule(ModuleType.BASEL)
    m = generateModule(getModuleTypeList()[0])


    print(moduleTypeList)
    print(b)

    for module in getModuleTypeList():
        print(module)
    
    print(moduleType2action(ModuleType.BASEM))
    print(list(map(moduleType2action, getModuleTypeList())))