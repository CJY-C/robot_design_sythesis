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


# # 获取执行文件所在目录的绝对路径
# CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))  
# # 获取上级目录的绝对路径
# PARENT_DIR = os.path.abspath(os.path.join(CURRENT_DIR, os.pardir))  
# # 获取上上级目录的绝对路径
# GRANDPA_DIR = os.path.abspath(os.path.join(PARENT_DIR, os.pardir))

# RES_PATH = GRANDPA_DIR + '/RobotConfigDesignEnv/robotConfigDesign/res/'
# print(RES_PATH)


# # basic units

# baseLUnitCnt = 0
# baseLUnitFilepath = RES_PATH + "base-l.SLDASM/urdf/base-l.SLDASM.urdf"
# baseLLinkList, baseLLinkTypeList, baseLJointList, baseLInterfaceSizeList = parseURDF(baseLUnitFilepath)
# BaseLUnit = Unit("BaseL_U", baseLLinkList, baseLLinkTypeList, baseLJointList, baseLInterfaceSizeList)
# BaseLUnit.mass = 1

# baseMUnitCnt = 0
# baseMUnitFilepath = RES_PATH + "base-m.SLDASM/urdf/base-m.SLDASM.urdf"
# baseMLinkList, baseMLinkTypeList, baseMJointList, baseMInterfaceSizeList = parseURDF(baseMUnitFilepath)
# BaseMUnit = Unit("BaseM_U", baseMLinkList, baseMLinkTypeList, baseMJointList, baseMInterfaceSizeList)
# BaseMUnit.mass = 0.5

# inputLUnitCnt = 0
# inputLUnitFilepath = RES_PATH + "input_connector_l.SLDASM/urdf/input_connector_l.SLDASM.urdf"
# inputLLinkList, inputLLinkTypeList, inputLJointList, inputLInterfaceSizeList = parseURDF(inputLUnitFilepath)
# InputLUnit = Unit("InputL_U", inputLLinkList, inputLLinkTypeList, inputLJointList, inputLInterfaceSizeList)
# InputLUnit.mass = 0

# inputMUnitCnt = 0
# inputMUnitFilepath = RES_PATH + "input_connector_m.SLDASM/urdf/input_connector_m.SLDASM.urdf"
# inputMLinkList, inputMLinkTypeList, inputMJointList, inputMInterfaceSizeList = parseURDF(inputMUnitFilepath)
# InputMUnit = Unit("InputM_U", inputMLinkList, inputMLinkTypeList, inputMJointList, inputMInterfaceSizeList)
# InputMUnit.mass = 0

# inputSUnitCnt = 0
# inputSUnitFilepath = RES_PATH + "input_connector_s.SLDASM/urdf/input_connector_s.SLDASM.urdf"
# inputSLinkList, inputSLinkTypeList, inputSJointList, inputSInterfaceSizeList = parseURDF(inputSUnitFilepath)
# InputSUnit = Unit("InputS_U", inputSLinkList, inputSLinkTypeList, inputSJointList, inputSInterfaceSizeList)
# InputSUnit.mass = 0

# jointTLUnitCnt = 0
# jointTLUnitFilepath = RES_PATH + "jointT-l.SLDASM/urdf/jointT-l.SLDASM.urdf"
# jointTLLinkList, jointTLLinkTypeList, jointTLJointList, jointTLInterfaceSizeList = parseURDF(jointTLUnitFilepath)
# JointTLUnit = Unit("JointTL_U", jointTLLinkList, jointTLLinkTypeList, jointTLJointList, jointTLInterfaceSizeList)

# jointMUnitCnt = 0
# jointMUnitFilepath = RES_PATH + "jointT-m.SLDASM/urdf/jointT-m.SLDASM.urdf"
# jointMLinkList, jointMLinkTypeList, jointMJointList, jointMInterfaceSizeList = parseURDF(jointMUnitFilepath)
# JointMUnit = Unit("JointM_U", jointMLinkList, jointMLinkTypeList, jointMJointList, jointMInterfaceSizeList)

# jointSUnitCnt = 0
# jointSUnitFilepath = RES_PATH + "jointT-s.SLDASM/urdf/jointT-s.SLDASM.urdf"
# jointSLinkList, jointSLinkTypeList, jointSJointList, jointSInterfaceSizeList = parseURDF(jointSUnitFilepath)
# JointSUnit = Unit("JointS_U", jointSLinkList, jointSLinkTypeList, jointSJointList, jointSInterfaceSizeList)

# cornerLLUnitCnt = 0
# cornerLLUnitFilepath = RES_PATH + "corner_link-l.SLDASM/urdf/corner_link-l.SLDASM.urdf"
# cornerLLLinkList, cornerLLLinkTypeList, cornerLLJointList, cornerLLInterfaceSizeList = parseURDF(cornerLLUnitFilepath)
# CornerLLUnit = Unit("CornerLL_U", cornerLLLinkList, cornerLLLinkTypeList, cornerLLJointList, cornerLLInterfaceSizeList)

# cornerLMUnitCnt = 0
# cornerLMUnitFilepath = RES_PATH + "corner_link-lm.SLDASM/urdf/corner_link-lm.SLDASM.urdf"
# cornerLMLinkList, cornerLMLinkTypeList, cornerLMJointList, cornerLMInterfaceSizeList = parseURDF(cornerLMUnitFilepath)
# CornerLMUnit = Unit("CornerLM_U", cornerLMLinkList, cornerLMLinkTypeList, cornerLMJointList, cornerLMInterfaceSizeList)

# cornerMMUnitCnt = 0
# cornerMMUnitFilepath = RES_PATH + "corner_link-mm.SLDASM/urdf/corner_link-mm.SLDASM.urdf"
# cornerMMLinkList, cornerMMLinkTypeList, cornerMMJointList, cornerMMInterfaceSizeList = parseURDF(cornerMMUnitFilepath)
# CornerMMUnit = Unit("CornerMM_U", cornerMMLinkList, cornerMMLinkTypeList, cornerMMJointList, cornerMMInterfaceSizeList)

# cornerMSUnitCnt = 0
# cornerMSUnitFilepath = RES_PATH + "corner_link-ms.SLDASM/urdf/corner_link-ms.SLDASM.urdf"
# cornerMSLinkList, cornerMSLinkTypeList, cornerMSJointList, cornerMSInterfaceSizeList = parseURDF(cornerMSUnitFilepath)
# CornerMSUnit = Unit("CornerMS_U", cornerMSLinkList, cornerMSLinkTypeList, cornerMSJointList, cornerMSInterfaceSizeList)

# straightLUnitCnt = 0
# straightLUnitFilepath = RES_PATH + "straight_link-l.SLDASM/urdf/straight_link-l.SLDASM.urdf"
# straightLLinkList, straightLLinkTypeList, straightLJointList, straightLInterfaceSizeList = parseURDF(straightLUnitFilepath)
# StraightLUnit = Unit("StraightL_U", straightLLinkList, straightLLinkTypeList, straightLJointList, straightLInterfaceSizeList)

# straightLMUnitCnt = 0
# straightLMUnitFilepath = RES_PATH + "straight_link-lm.SLDASM/urdf/straight_link-lm.SLDASM.urdf"
# straightLMLinkList, straightLMLinkTypeList, straightLMJointList, straightLMInterfaceSizeList = parseURDF(straightLMUnitFilepath)
# StraightLMUnit = Unit("StraightLM_U", straightLMLinkList, straightLMLinkTypeList, straightLMJointList, straightLMInterfaceSizeList)

# straightMUnitCnt = 0
# straightMUnitFilepath = RES_PATH + "straight_link-m.SLDASM/urdf/straight_link-m.SLDASM.urdf"
# straightMLinkList, straightMLinkTypeList, straightMJointList, straightMInterfaceSizeList = parseURDF(straightMUnitFilepath)
# StraightMUnit = Unit("StraightM_U", straightMLinkList, straightMLinkTypeList, straightMJointList, straightMInterfaceSizeList)

# straightMSUnitCnt = 0
# straightMSUnitFilepath = RES_PATH + "straight_link-ms.SLDASM/urdf/straight_link-ms.SLDASM.urdf"
# straightMSLinkList, straightMSLinkTypeList, straightMSJointList, straightMSInterfaceSizeList = parseURDF(straightMSUnitFilepath)
# StraightMSUnit = Unit("StraightMS_U", straightMSLinkList, straightMSLinkTypeList, straightMSJointList, straightMSInterfaceSizeList)


# endeffectorLUnitCnt = 0
# endeffectorLUnitFilepath = RES_PATH + "end_effector_v_l.SLDASM/urdf/end_effector_v_l.SLDASM.urdf"
# endeffectorLLinkList, endeffectorLLinkTypeList, endeffectorLJointList, endeffectorLInterfaceSizeList = parseURDF(endeffectorLUnitFilepath)
# EndEffectorLUnit = Unit("EndeffectorL_U", endeffectorLLinkList, endeffectorLLinkTypeList, endeffectorLJointList, endeffectorLInterfaceSizeList)

# endeffectorMUnitCnt = 0
# endeffectorMUnitFilepath = RES_PATH + "end_effector_v_m.SLDASM/urdf/end_effector_v_m.SLDASM.urdf"
# endeffectorMLinkList, endeffectorMLinkTypeList, endeffectorMJointList, endeffectorMInterfaceSizeList = parseURDF(endeffectorMUnitFilepath)
# EndEffectorMUnit = Unit("EndeffectorM_U", endeffectorMLinkList, endeffectorMLinkTypeList, endeffectorMJointList, endeffectorMInterfaceSizeList)

# endeffectorSUnitCnt = 0
# endeffectorSUnitFilepath = RES_PATH + "end_effector_v_s.SLDASM/urdf/end_effector_v_s.SLDASM.urdf"
# endeffectorSLinkList, endeffectorSLinkTypeList, endeffectorSJointList, endeffectorSInterfaceSizeList = parseURDF(endeffectorSUnitFilepath)
# EndEffectorSUnit = Unit("EndeffectorS_U", endeffectorSLinkList, endeffectorSLinkTypeList, endeffectorSJointList, endeffectorSInterfaceSizeList)

# generate unit
# def generateUnit(unitType, order=Order.NORMAL):
    # Test
    # print(register.PathRegister.get_paths())
    # global baseLUnitCnt, inputLUnitCnt, jointTLUnitCnt, cornerLLUnitCnt, straightLUnitCnt, endeffectorLUnitCnt, \
    # cornerLMUnitCnt, jointMUnitCnt, inputMUnitCnt, straightMUnitCnt, baseMUnitCnt, endeffectorMUnitCnt, straightLMUnitCnt, cornerMMUnitCnt,\
    # jointSUnitCnt, cornerMSUnitCnt, inputSUnitCnt, endeffectorSUnitCnt, straightMSUnitCnt

    # if unitType == UnitType.BASEL:
    #     # unit = deepcopy(BaseLUnit) if order == Order.NORMAL else deepcopy(BaseLUnit).reverse()
    #     unit = deepcopy(units['BASEL']['unit']) if order == Order.NORMAL else deepcopy(units['BaseL']['unit']).reverse()
    #     unit.set_name("BLU" + str(units['BASEL']['count']))
    #     units['BASEL']['count'] += 1
    # elif unitType == UnitType.BASEM:
    #     unit = deepcopy(BaseMUnit) if order == Order.NORMAL else deepcopy(BaseMUnit).reverse()
    #     unit.set_name("BMU" + str(baseMUnitCnt))
    #     baseMUnitCnt += 1
    # elif unitType == UnitType.CONNECTORL:
    #     unit = deepcopy(InputLUnit) if order == Order.NORMAL else deepcopy(InputLUnit).reverse()
    #     unit.set_name("ILU" + str(inputLUnitCnt))
    #     inputLUnitCnt += 1
    # elif unitType == UnitType.CONNECTORM:
    #     unit = deepcopy(InputMUnit) if order == Order.NORMAL else deepcopy(InputMUnit).reverse()
    #     unit.set_name("IMU" + str(inputMUnitCnt))
    #     inputMUnitCnt += 1
    # elif unitType == UnitType.CONNECTORS:
    #     unit = deepcopy(InputSUnit) if order == Order.NORMAL else deepcopy(InputSUnit).reverse()
    #     unit.set_name("ISU" + str(inputSUnitCnt))
    #     inputSUnitCnt += 1
    # elif unitType == UnitType.JOINTL:
    #     unit = deepcopy(JointTLUnit) if order == Order.NORMAL else deepcopy(JointTLUnit).reverse()
    #     unit.set_name("JTLU" + str(jointTLUnitCnt))
    #     jointTLUnitCnt += 1
    # elif unitType == UnitType.JOINTM:
    #     unit = deepcopy(JointMUnit) if order == Order.NORMAL else deepcopy(JointMUnit).reverse()
    #     unit.set_name("JTMU" + str(jointMUnitCnt))
    #     jointMUnitCnt += 1
    # elif unitType == UnitType.JOINTS:
    #     unit = deepcopy(JointSUnit) if order == Order.NORMAL else deepcopy(JointSUnit).reverse()
    #     unit.set_name("JSU" + str(jointSUnitCnt))
    #     jointSUnitCnt += 1
    # elif unitType == UnitType.CORNERLINKLL:
    #     unit = deepcopy(CornerLLUnit) if order == Order.NORMAL else deepcopy(CornerLLUnit).reverse()
    #     unit.set_name("CLLU" + str(cornerLLUnitCnt))
    #     cornerLLUnitCnt += 1
    # elif unitType == UnitType.CORNERLINKLM:
    #     unit = deepcopy(CornerLMUnit) if order == Order.NORMAL else deepcopy(CornerLMUnit).reverse()
    #     unit.set_name("CLMU" + str(cornerLMUnitCnt))
    #     cornerLMUnitCnt += 1
    # elif unitType == UnitType.CORNERLINKMM:
    #     unit = deepcopy(CornerMMUnit) if order == Order.NORMAL else deepcopy(CornerMMUnit).reverse()
    #     unit.set_name("CMMU" + str(cornerMMUnitCnt))
    #     cornerMMUnitCnt += 1
    # elif unitType == UnitType.CORNERLINKMS:
    #     unit = deepcopy(CornerMSUnit) if order == Order.NORMAL else deepcopy(CornerMSUnit).reverse()
    #     unit.set_name("CMSU" + str(cornerMSUnitCnt))
    #     cornerMSUnitCnt += 1
    # elif unitType == UnitType.STRAIGHTLINKLL:
    #     unit = deepcopy(StraightLUnit) if order == Order.NORMAL else deepcopy(StraightLUnit).reverse()
    #     unit.set_name("SLU" + str(straightLUnitCnt))
    #     straightLUnitCnt += 1
    # elif unitType == UnitType.STRAIGHTLINKLM:
    #     unit = deepcopy(StraightLMUnit) if order == Order.NORMAL else deepcopy(StraightLMUnit).reverse()
    #     unit.set_name("SLMU" + str(straightLMUnitCnt))
    #     straightLMUnitCnt += 1
    # elif unitType == UnitType.STRAIGHTLINKMM:
    #     unit = deepcopy(StraightMUnit) if order == Order.NORMAL else deepcopy(StraightMUnit).reverse()
    #     unit.set_name("SMU" + str(straightMUnitCnt))
    #     straightMUnitCnt += 1
    # elif unitType == UnitType.STRAIGHTLINKMS:
    #     unit = deepcopy(StraightMSUnit) if order == Order.NORMAL else deepcopy(StraightMSUnit).reverse()
    #     unit.set_name("SMSU" + str(straightMSUnitCnt))
    #     straightMSUnitCnt += 1
    # elif unitType == UnitType.ENDEFFECTORL:
    #     unit = deepcopy(EndEffectorLUnit) if order == Order.NORMAL else deepcopy(EndEffectorLUnit).reverse()
    #     unit.set_name("ELU" + str(endeffectorLUnitCnt))
    #     endeffectorLUnitCnt += 1
    # elif unitType == UnitType.ENDEFFECTORM:
    #     unit = deepcopy(EndEffectorMUnit) if order == Order.NORMAL else deepcopy(EndEffectorMUnit).reverse()
    #     unit.set_name("EMU" + str(endeffectorMUnitCnt))
    #     endeffectorMUnitCnt += 1
    # elif unitType == UnitType.ENDEFFECTORS:
    #     unit = deepcopy(EndEffectorSUnit) if order == Order.NORMAL else deepcopy(EndEffectorSUnit).reverse()
    #     unit.set_name("ESU" + str(endeffectorSUnitCnt))
    #     endeffectorSUnitCnt += 1
    # else:
    #     raise Exception("unitType not found")

    # unit.set_name_prefix(modulePrefix)
    return unit


# 测试
if __name__ == "__main__":
    from urdfGenerator import register
    # TODO: 为了测试，暂时将资源路径写死
    register.PathRegister.add_path('/home/masa/learning/rl/undergraduate/cjy/robot_design_sythesis/src/RobotConfigDesignEnv/robotConfigDesign/res/config.json')

    bl = generateUnit(UnitType.BASEL)
    bm = generateUnit(UnitType.BASEM)
    print(bm.name)
    
    # moduletype1 = list([UnitType.JOINTL, UnitType.CONNECTORL])
    # moduletype1 = list([UnitType.CONNECTORLL, UnitType.CORNERLINKL, UnitType.CONNECTORL, ])
    # # moduletype1 = list([UnitType.BASEL, UnitType.STRAIGHTLINKL, UnitType.CORNERLINKL, UnitType.JOINTL, UnitType.ENDEFFECTORL])
    # unitList = list()

    # for unitType in moduletype1:
    #     unit = generateUnit(unitType)
    #     # unit = generateUnit(unitType, Order.REVERSE)
    #     unitList.append(unit)

    # # unit7 = reduce(lambda x, y: x + y, unitList)
    # unit7 = reduce(lambda x, y: x + y, list(reversed(unitList)))

    # print(unit7.linkTypeDeque)
    # unit7 = InputUnit + CornerLinkLUnit + InputUnit
    # unit7 = StraightLinkUnitL + (InputUnit.reverse() + JointTUnit.reverse())

    # import os
    # dir = os.path.dirname(os.path.realpath(__file__))
    # with open(dir + '/unit7.urdf', 'w') as f:
    #     f.write(unit7.__str__())