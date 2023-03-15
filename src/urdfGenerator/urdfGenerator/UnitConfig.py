
from collections import deque
from odio_urdf import *
from urdfGenerator.Enums import LinkType, UnitOrder, UnitType, InterfaceSize

from copy import deepcopy

from urdfGenerator.LinkPlus import Link
from urdfGenerator.JointPlus import Joint
from urdfGenerator.Unit import Unit

from functools import reduce


def list_to_dict(lst):
    if len(lst) == 0:
        return None
    if len(lst) == 1:
        return lst[0]
    if len(lst) % 2 != 0:
        raise ValueError('List length must be even')
    result = {}
    for i in range(0, len(lst), 2):
        key = lst[i]
        value = lst[i + 1]
        if isinstance(value, list):
            value = list_to_dict(value)
        if key in result:
            result[f'{key}{len(result) + 1}'] = value
        else:
            result[key] = value
    return result


def parseURDF(filepath):
    with open(filepath, "r") as f:
        urdf_string = f.read()
    urdf = urdf_to_odio(urdf_string)

    import pyparsing as pp

    # Define punctuation literals
    LPAR, RPAR, EQ = map(pp.Suppress, '()=')
    # 定义括号
    LPAR, RPAR = pp.Suppress("("), pp.Suppress(")")

    # Define key and value literals
    key = pp.Word(pp.alphas)
    value = pp.quotedString | pp.Word(pp.printables, excludeChars=',)')

    # Define key-value pairs
    pair = key + EQ + value

    # Define nested expression
    expr = pp.Forward()
    args = pp.Group(pp.delimitedList(pair | expr))
    expr << key + LPAR + args + pp.Optional(pp.Suppress(',')) + RPAR

    # Define top-level expression
    top_expr = pp.delimitedList(expr) + pp.StringEnd()

    # Parse input string
    result = top_expr.parseString(urdf).asList()

    linkList = list()
    linkTypeList = list()
    jointList = list()
    interfaceSizeList = list()
    for key, value in list_to_dict(result).items():
        for key_v, value_v in value.items():
            # print(f'{key_v}: {value_v}')
            if 'Link' in key_v:
                link = eval(f'\
                Link(\
                    Inertial(\
                        Origin(xyz={value_v["Inertial"]["Origin"]["xyz"] }, rpy={value_v["Inertial"]["Origin"]["rpy"]}), \
                        Mass(value={value_v["Inertial"]["Mass"]["value"]}),\
                        Inertia(ixx={value_v["Inertial"]["Inertia"]["ixx"]}, ixy={value_v["Inertial"]["Inertia"]["ixy"]}, ixz={value_v["Inertial"]["Inertia"]["ixz"]}, iyy={value_v["Inertial"]["Inertia"]["iyy"]}, iyz={value_v["Inertial"]["Inertia"]["iyz"]}, izz={value_v["Inertial"]["Inertia"]["izz"]}),\
                    ),\
                    Visual(\
                        Origin(xyz={value_v["Visual"]["Origin"]["xyz"]}, rpy={value_v["Visual"]["Origin"]["rpy"]}), \
                        Geometry(Mesh(filename={value_v["Visual"]["Geometry"]["Mesh"]["filename"]})), \
                        Material(Color(rgba={value_v["Visual"]["Material"]["Color"]["rgba"]}), \
                        name={value_v["Visual"]["Material"]["name"]})\
                    ),\
                    Collision(\
                        Origin(xyz={value_v["Collision"]["Origin"]["xyz"]}, rpy={value_v["Collision"]["Origin"]["rpy"]}), \
                        Geometry(Mesh(filename={value_v["Collision"]["Geometry"]["Mesh"]["filename"]}))\
                    ), \
                    name={value_v["name"]},\
                )\
                ')
                linkList.append(link)

                if 'body' in value_v['name']:
                    linkTypeList.append(LinkType.BODY)
                elif 'in' in value_v['name']:
                    linkTypeList.append(LinkType.INPUT)
                elif 'out' in value_v['name']:
                    linkTypeList.append(LinkType.OUTPUT)
                else:
                    raise Exception('link type error')


            elif 'Joint' in key_v:
                # print(f"{key_v}: {value_v['type']}")
                if 'fixed' in value_v['type']:
                    joint = eval(f'\
                    Joint(\
                        Origin(xyz={value_v["Origin"]["xyz"]}, rpy={value_v["Origin"]["rpy"]}), \
                        Parent(link={value_v["Parent"]["link"]}), \
                        Child(link={value_v["Child"]["link"]}), \
                        Axis(xyz={value_v["Axis"]["xyz"]}), \
                        name={value_v["name"]},\
                        type={value_v["type"]},\
                    )\
                    ')
                elif 'revolute' in value_v['type']:
                    joint = eval(f'\
                    Joint(\
                        Origin(xyz={value_v["Origin"]["xyz"]}, rpy={value_v["Origin"]["rpy"]}), \
                        Parent(link={value_v["Parent"]["link"]}), \
                        Child(link={value_v["Child"]["link"]}), \
                        Axis(xyz={value_v["Axis"]["xyz"]}), \
                        Limit(upper={value_v["Limit"]["upper"]}, lower={value_v["Limit"]["lower"]}, effort={value_v["Limit"]["effort"]}, velocity={value_v["Limit"]["velocity"]}), \
                        name={value_v["name"]},\
                        type={value_v["type"]},\
                    )\
                    ')
                else:
                    raise Exception('joint type error')
                jointList.append(joint)
                
                if 'large' in value_v['name']:
                    interfaceSizeList.append(InterfaceSize.LARGE)
                elif 'medium' in value_v['name']:
                    interfaceSizeList.append(InterfaceSize.MEDIUM)
                elif 'small' in value_v['name']:
                    interfaceSizeList.append(InterfaceSize.SMALL)
                else:
                    raise Exception('interface size error')


    return linkList, linkTypeList, jointList, interfaceSizeList

# basic units

baseLUnitCnt = 0
baseLUnitFilepath = "/home/masa/learning/rl/undergraduate/cjy/robot_design_sythesis/test/urdf/base-l.SLDASM/urdf/base-l.SLDASM.urdf"
baseLLinkList, baseLLinkTypeList, baseLJointList, baseLInterfaceSizeList = parseURDF(baseLUnitFilepath)
BaseLUnit = Unit("BaseL_U", baseLLinkList, baseLLinkTypeList, baseLJointList, baseLInterfaceSizeList)

baseMUnitCnt = 0
baseMUnitFilepath = "/home/masa/learning/rl/undergraduate/cjy/robot_design_sythesis/test/urdf/base-m.SLDASM/urdf/base-m.SLDASM.urdf"
baseMLinkList, baseMLinkTypeList, baseMJointList, baseMInterfaceSizeList = parseURDF(baseMUnitFilepath)
BaseMUnit = Unit("BaseM_U", baseMLinkList, baseMLinkTypeList, baseMJointList, baseMInterfaceSizeList)

inputLUnitCnt = 0
inputLUnitFilepath = "/home/masa/learning/rl/undergraduate/cjy/robot_design_sythesis/test/urdf/input_connector_l.SLDASM/urdf/input_connector_l.SLDASM.urdf"
inputLLinkList, inputLLinkTypeList, inputLJointList, inputLInterfaceSizeList = parseURDF(inputLUnitFilepath)
InputLUnit = Unit("InputL_U", inputLLinkList, inputLLinkTypeList, inputLJointList, inputLInterfaceSizeList)

inputMUnitCnt = 0
inputMUnitFilepath = "/home/masa/learning/rl/undergraduate/cjy/robot_design_sythesis/test/urdf/input_connector_m.SLDASM/urdf/input_connector_m.SLDASM.urdf"
inputMLinkList, inputMLinkTypeList, inputMJointList, inputMInterfaceSizeList = parseURDF(inputMUnitFilepath)
InputMUnit = Unit("InputM_U", inputMLinkList, inputMLinkTypeList, inputMJointList, inputMInterfaceSizeList)

inputSUnitCnt = 0
inputSUnitFilepath = "/home/masa/learning/rl/undergraduate/cjy/robot_design_sythesis/test/urdf/input_connector_s.SLDASM/urdf/input_connector_s.SLDASM.urdf"
inputSLinkList, inputSLinkTypeList, inputSJointList, inputSInterfaceSizeList = parseURDF(inputSUnitFilepath)
InputSUnit = Unit("InputS_U", inputSLinkList, inputSLinkTypeList, inputSJointList, inputSInterfaceSizeList)

jointTLUnitCnt = 0
jointTLUnitFilepath = "/home/masa/learning/rl/undergraduate/cjy/robot_design_sythesis/test/urdf/jointT-l.SLDASM/urdf/jointT-l.SLDASM.urdf"
jointTLLinkList, jointTLLinkTypeList, jointTLJointList, jointTLInterfaceSizeList = parseURDF(jointTLUnitFilepath)
JointTLUnit = Unit("JointTL_U", jointTLLinkList, jointTLLinkTypeList, jointTLJointList, jointTLInterfaceSizeList)

jointMUnitCnt = 0
jointMUnitFilepath = "/home/masa/learning/rl/undergraduate/cjy/robot_design_sythesis/test/urdf/jointT-m.SLDASM/urdf/jointT-m.SLDASM.urdf"
jointMLinkList, jointMLinkTypeList, jointMJointList, jointMInterfaceSizeList = parseURDF(jointMUnitFilepath)
JointMUnit = Unit("JointM_U", jointMLinkList, jointMLinkTypeList, jointMJointList, jointMInterfaceSizeList)

jointSUnitCnt = 0
jointSUnitFilepath = "/home/masa/learning/rl/undergraduate/cjy/robot_design_sythesis/test/urdf/jointT-s.SLDASM/urdf/jointT-s.SLDASM.urdf"
jointSLinkList, jointSLinkTypeList, jointSJointList, jointSInterfaceSizeList = parseURDF(jointSUnitFilepath)
JointSUnit = Unit("JointS_U", jointSLinkList, jointSLinkTypeList, jointSJointList, jointSInterfaceSizeList)

cornerLLUnitCnt = 0
cornerLLUnitFilepath = "/home/masa/learning/rl/undergraduate/cjy/robot_design_sythesis/test/urdf/corner_link-l.SLDASM/urdf/corner_link-l.SLDASM.urdf"
cornerLLLinkList, cornerLLLinkTypeList, cornerLLJointList, cornerLLInterfaceSizeList = parseURDF(cornerLLUnitFilepath)
CornerLLUnit = Unit("CornerLL_U", cornerLLLinkList, cornerLLLinkTypeList, cornerLLJointList, cornerLLInterfaceSizeList)

cornerLMUnitCnt = 0
cornerLMUnitFilepath = "/home/masa/learning/rl/undergraduate/cjy/robot_design_sythesis/test/urdf/corner_link-lm.SLDASM/urdf/corner_link-lm.SLDASM.urdf"
cornerLMLinkList, cornerLMLinkTypeList, cornerLMJointList, cornerLMInterfaceSizeList = parseURDF(cornerLMUnitFilepath)
CornerLMUnit = Unit("CornerLM_U", cornerLMLinkList, cornerLMLinkTypeList, cornerLMJointList, cornerLMInterfaceSizeList)

cornerMMUnitCnt = 0
cornerMMUnitFilepath = "/home/masa/learning/rl/undergraduate/cjy/robot_design_sythesis/test/urdf/corner_link-mm.SLDASM/urdf/corner_link-mm.SLDASM.urdf"
cornerMMLinkList, cornerMMLinkTypeList, cornerMMJointList, cornerMMInterfaceSizeList = parseURDF(cornerMMUnitFilepath)
CornerMMUnit = Unit("CornerMM_U", cornerMMLinkList, cornerMMLinkTypeList, cornerMMJointList, cornerMMInterfaceSizeList)

cornerMSUnitCnt = 0
cornerMSUnitFilepath = "/home/masa/learning/rl/undergraduate/cjy/robot_design_sythesis/test/urdf/corner_link-ms.SLDASM/urdf/corner_link-ms.SLDASM.urdf"
cornerMSLinkList, cornerMSLinkTypeList, cornerMSJointList, cornerMSInterfaceSizeList = parseURDF(cornerMSUnitFilepath)
CornerMSUnit = Unit("CornerMS_U", cornerMSLinkList, cornerMSLinkTypeList, cornerMSJointList, cornerMSInterfaceSizeList)

straightLUnitCnt = 0
straightLUnitFilepath = "/home/masa/learning/rl/undergraduate/cjy/robot_design_sythesis/test/urdf/straight_link-l.SLDASM/urdf/straight_link-l.SLDASM.urdf"
straightLLinkList, straightLLinkTypeList, straightLJointList, straightLInterfaceSizeList = parseURDF(straightLUnitFilepath)
StraightLUnit = Unit("StraightL_U", straightLLinkList, straightLLinkTypeList, straightLJointList, straightLInterfaceSizeList)

straightLMUnitCnt = 0
straightLMUnitFilepath = "/home/masa/learning/rl/undergraduate/cjy/robot_design_sythesis/test/urdf/straight_link-lm.SLDASM/urdf/straight_link-lm.SLDASM.urdf"
straightLMLinkList, straightLMLinkTypeList, straightLMJointList, straightLMInterfaceSizeList = parseURDF(straightLMUnitFilepath)
StraightLMUnit = Unit("StraightLM_U", straightLMLinkList, straightLMLinkTypeList, straightLMJointList, straightLMInterfaceSizeList)

straightMUnitCnt = 0
straightMUnitFilepath = "/home/masa/learning/rl/undergraduate/cjy/robot_design_sythesis/test/urdf/straight_link-m.SLDASM/urdf/straight_link-m.SLDASM.urdf"
straightMLinkList, straightMLinkTypeList, straightMJointList, straightMInterfaceSizeList = parseURDF(straightMUnitFilepath)
StraightMUnit = Unit("StraightM_U", straightMLinkList, straightMLinkTypeList, straightMJointList, straightMInterfaceSizeList)

straightMSUnitCnt = 0
straightMSUnitFilepath = "/home/masa/learning/rl/undergraduate/cjy/robot_design_sythesis/test/urdf/straight_link-ms.SLDASM/urdf/straight_link-ms.SLDASM.urdf"
straightMSLinkList, straightMSLinkTypeList, straightMSJointList, straightMSInterfaceSizeList = parseURDF(straightMSUnitFilepath)
StraightMSUnit = Unit("StraightMS_U", straightMSLinkList, straightMSLinkTypeList, straightMSJointList, straightMSInterfaceSizeList)


endeffectorLUnitCnt = 0
endeffectorLUnitFilepath = "/home/masa/learning/rl/undergraduate/cjy/robot_design_sythesis/test/urdf/end_effector_l.SLDASM/urdf/end_effector_l.SLDASM.urdf"
endeffectorLLinkList, endeffectorLLinkTypeList, endeffectorLJointList, endeffectorLInterfaceSizeList = parseURDF(endeffectorLUnitFilepath)
EndEffectorLUnit = Unit("EndeffectorL_U", endeffectorLLinkList, endeffectorLLinkTypeList, endeffectorLJointList, endeffectorLInterfaceSizeList)

endeffectorMUnitCnt = 0
endeffectorMUnitFilepath = "/home/masa/learning/rl/undergraduate/cjy/robot_design_sythesis/test/urdf/end_effector_m.SLDASM/urdf/end_effector_m.SLDASM.urdf"
endeffectorMLinkList, endeffectorMLinkTypeList, endeffectorMJointList, endeffectorMInterfaceSizeList = parseURDF(endeffectorMUnitFilepath)
EndEffectorMUnit = Unit("EndeffectorM_U", endeffectorMLinkList, endeffectorMLinkTypeList, endeffectorMJointList, endeffectorMInterfaceSizeList)

endeffectorSUnitCnt = 0
endeffectorSUnitFilepath = "/home/masa/learning/rl/undergraduate/cjy/robot_design_sythesis/test/urdf/end_effector_s.SLDASM/urdf/end_effector_s.SLDASM.urdf"
endeffectorSLinkList, endeffectorSLinkTypeList, endeffectorSJointList, endeffectorSInterfaceSizeList = parseURDF(endeffectorSUnitFilepath)
EndEffectorSUnit = Unit("EndeffectorS_U", endeffectorSLinkList, endeffectorSLinkTypeList, endeffectorSJointList, endeffectorSInterfaceSizeList)

# generate unit
def generateUnit(unitType, order=UnitOrder.NORMAL):
    global baseLUnitCnt, inputLUnitCnt, jointTLUnitCnt, cornerLLUnitCnt, straightLUnitCnt, endeffectorLUnitCnt, \
    cornerLMUnitCnt, jointMUnitCnt, inputMUnitCnt, straightMUnitCnt, baseMUnitCnt, endeffectorMUnitCnt, straightLMUnitCnt, cornerMMUnitCnt,\
    jointSUnitCnt, cornerMSUnitCnt, inputSUnitCnt, endeffectorSUnitCnt, straightMSUnitCnt

    if unitType == UnitType.BASEL:
        unit = deepcopy(BaseLUnit) if order == UnitOrder.NORMAL else deepcopy(BaseLUnit).reverse()
        unit.set_name("BLU" + str(baseLUnitCnt))
        baseLUnitCnt += 1
    elif unitType == UnitType.BASEM:
        unit = deepcopy(BaseMUnit) if order == UnitOrder.NORMAL else deepcopy(BaseMUnit).reverse()
        unit.set_name("BMU" + str(baseMUnitCnt))
        baseMUnitCnt += 1
    elif unitType == UnitType.CONNECTORL:
        unit = deepcopy(InputLUnit) if order == UnitOrder.NORMAL else deepcopy(InputLUnit).reverse()
        unit.set_name("ILU" + str(inputLUnitCnt))
        inputLUnitCnt += 1
    elif unitType == UnitType.CONNECTORM:
        unit = deepcopy(InputMUnit) if order == UnitOrder.NORMAL else deepcopy(InputMUnit).reverse()
        unit.set_name("IMU" + str(inputMUnitCnt))
        inputMUnitCnt += 1
    elif unitType == UnitType.CONNECTORS:
        unit = deepcopy(InputSUnit) if order == UnitOrder.NORMAL else deepcopy(InputSUnit).reverse()
        unit.set_name("ISU" + str(inputSUnitCnt))
        inputSUnitCnt += 1
    elif unitType == UnitType.JOINTL:
        unit = deepcopy(JointTLUnit) if order == UnitOrder.NORMAL else deepcopy(JointTLUnit).reverse()
        unit.set_name("JTLU" + str(jointTLUnitCnt))
        jointTLUnitCnt += 1
    elif unitType == UnitType.JOINTM:
        unit = deepcopy(JointMUnit) if order == UnitOrder.NORMAL else deepcopy(JointMUnit).reverse()
        unit.set_name("JTMU" + str(jointMUnitCnt))
        jointMUnitCnt += 1
    elif unitType == UnitType.JOINTS:
        unit = deepcopy(JointSUnit) if order == UnitOrder.NORMAL else deepcopy(JointSUnit).reverse()
        unit.set_name("JSU" + str(jointSUnitCnt))
        jointSUnitCnt += 1
    elif unitType == UnitType.CORNERLINKLL:
        unit = deepcopy(CornerLLUnit) if order == UnitOrder.NORMAL else deepcopy(CornerLLUnit).reverse()
        unit.set_name("CLLU" + str(cornerLLUnitCnt))
        cornerLLUnitCnt += 1
    elif unitType == UnitType.CORNERLINKLM:
        unit = deepcopy(CornerLMUnit) if order == UnitOrder.NORMAL else deepcopy(CornerLMUnit).reverse()
        unit.set_name("CLMU" + str(cornerLMUnitCnt))
        cornerLMUnitCnt += 1
    elif unitType == UnitType.CORNERLINKMM:
        unit = deepcopy(CornerMMUnit) if order == UnitOrder.NORMAL else deepcopy(CornerMMUnit).reverse()
        unit.set_name("CMMU" + str(cornerMMUnitCnt))
        cornerMMUnitCnt += 1
    elif unitType == UnitType.CORNERLINKMS:
        unit = deepcopy(CornerMSUnit) if order == UnitOrder.NORMAL else deepcopy(CornerMSUnit).reverse()
        unit.set_name("CMSU" + str(cornerMSUnitCnt))
        cornerMSUnitCnt += 1
    elif unitType == UnitType.STRAIGHTLINKLL:
        unit = deepcopy(StraightLUnit) if order == UnitOrder.NORMAL else deepcopy(StraightLUnit).reverse()
        unit.set_name("SLU" + str(straightLUnitCnt))
        straightLUnitCnt += 1
    elif unitType == UnitType.STRAIGHTLINKLM:
        unit = deepcopy(StraightLMUnit) if order == UnitOrder.NORMAL else deepcopy(StraightLMUnit).reverse()
        unit.set_name("SLMU" + str(straightLMUnitCnt))
        straightLMUnitCnt += 1
    elif unitType == UnitType.STRAIGHTLINKMM:
        unit = deepcopy(StraightMUnit) if order == UnitOrder.NORMAL else deepcopy(StraightMUnit).reverse()
        unit.set_name("SMU" + str(straightMUnitCnt))
        straightMUnitCnt += 1
    elif unitType == UnitType.STRAIGHTLINKMS:
        unit = deepcopy(StraightMSUnit) if order == UnitOrder.NORMAL else deepcopy(StraightMSUnit).reverse()
        unit.set_name("SMSU" + str(straightMSUnitCnt))
        straightMSUnitCnt += 1
    elif unitType == UnitType.ENDEFFECTORL:
        unit = deepcopy(EndEffectorLUnit) if order == UnitOrder.NORMAL else deepcopy(EndEffectorLUnit).reverse()
        unit.set_name("ELU" + str(endeffectorLUnitCnt))
        endeffectorLUnitCnt += 1
    elif unitType == UnitType.ENDEFFECTORM:
        unit = deepcopy(EndEffectorMUnit) if order == UnitOrder.NORMAL else deepcopy(EndEffectorMUnit).reverse()
        unit.set_name("EMU" + str(endeffectorMUnitCnt))
        endeffectorMUnitCnt += 1
    elif unitType == UnitType.ENDEFFECTORS:
        unit = deepcopy(EndEffectorSUnit) if order == UnitOrder.NORMAL else deepcopy(EndEffectorSUnit).reverse()
        unit.set_name("ESU" + str(endeffectorSUnitCnt))
        endeffectorSUnitCnt += 1
    else:
        raise Exception("unitType not found")

    # unit.set_name_prefix(modulePrefix)
    return unit


# 测试
if __name__ == "__main__":

    # moduletype1 = list([UnitType.JOINTL, UnitType.CONNECTORL])
    moduletype1 = list([UnitType.CONNECTORL, UnitType.CORNERLINKL, UnitType.CONNECTORL, ])
    # moduletype1 = list([UnitType.BASEL, UnitType.STRAIGHTLINKL, UnitType.CORNERLINKL, UnitType.JOINTL, UnitType.ENDEFFECTORL])
    unitList = list()

    for unitType in moduletype1:
        unit = generateUnit(unitType)
        # unit = generateUnit(unitType, UnitOrder.REVERSE)
        unitList.append(unit)

    # unit7 = reduce(lambda x, y: x + y, unitList)
    unit7 = reduce(lambda x, y: x + y, list(reversed(unitList)))

    print(unit7.linkTypeDeque)
    # unit7 = InputUnit + CornerLinkLUnit + InputUnit
    # unit7 = StraightLinkUnitL + (InputUnit.reverse() + JointTUnit.reverse())

    # import os
    # dir = os.path.dirname(os.path.realpath(__file__))
    # with open(dir + '/unit7.urdf', 'w') as f:
    #     f.write(unit7.__str__())