from copy import deepcopy
from odio_urdf import *
from urdfGenerator.Enums import Order, UnitType
from urdfGenerator.Unit import Unit
from urdfGenerator.utils import parseURDF, getConfig


# 拿到配置
config = getConfig()
res_path = config['res_path']
units_config = config['units']

units = {}
unitTypeList = []

# 导入所有的单元
for unit_config in units_config:
    unit_name = unit_config["name"]
    unit_id = unit_config["id"]
    unit_filepath = res_path + unit_config["filepath"]

    unit_link_list, unit_link_type_list, unit_joint_list, unit_interface_size_list = parseURDF(unit_filepath)

    unit = Unit(unit_name, unit_link_list, unit_link_type_list, unit_joint_list, unit_interface_size_list)
    if "mass" in unit_config:
        unit.mass = unit_config["mass"]
    units[unit_name] = {
        'unit': unit,
        'id': unit_id,
        'count': 0,
    }
    setattr(UnitType, unit_name, unit_id)
    unitTypeList.append(getattr(UnitType, unit_name))

def generateUnit(unitType:UnitType, order=Order.NORMAL) -> Unit:
    '''
    生成一个Unit对象

    Args:
        unitType: UnitType类型，表示要生成的Unit的类型
        order: Order类型，表示要生成的Unit的方向

    Returns:
        一个Unit对象
    '''
    for name, u in units.items():
        if u['id'] == unitType:
            unit = deepcopy(u['unit']) if order == Order.NORMAL else deepcopy(u['unit']).reverse()
            unit.set_name(name + str(u['count']))
            u['count'] += 1
            return unit


# 测试
if __name__ == "__main__":
    from urdfGenerator import register
    # TODO: 测试，暂时将资源路径写死
    register.PathRegister.add_path('/home/masa/learning/rl/undergraduate/cjy/robot_design_sythesis/src/RobotConfigDesignEnv/robotConfigDesign/res/config.json')

    bl = generateUnit(UnitType.BASEL)
    bm = generateUnit(UnitType.BASEM)
    print(bm.name)
    