
from collections import deque
from odio_urdf import *
from urdfGenerator.Enums import LinkType, UnitOrder, UnitType, ModuleType

from copy import deepcopy

from urdfGenerator.LinkPlus import Link
from urdfGenerator.JointPlus import Joint
from urdfGenerator.Unit import Unit

import numpy as np
import modern_robotics as mr

from functools import reduce

from math import pi

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
    for key, value in list_to_dict(result).items():
        for key_v, value_v in value.items():
            # print(f'{key_v}: {value_v}')
            if 'Link' in key_v:
                link = eval(f'\
                Link(\
                    Inertial(\
                        Origin(xyz={ value_v["Inertial"]["Origin"]["xyz"] }, rpy={value_v["Inertial"]["Origin"]["rpy"]}), \
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
            elif 'Joint' in key_v:
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
                jointList.append(joint)

    # linkTypeList = [
    #     LinkType.OUTPUT,
    #     LinkType.BODY,
    #     LinkType.INPUT
    # ]

    return linkList, jointList

# basic units

baseUnitCnt = 0
baseUnitFilepath = "/home/masa/learning/rl/undergraduate/robot_design_sythesis/Tests/urdf_generator/OTesst/base-l.SLDASM/urdf/base-l.SLDASM.urdf"
baseLinkTpyeList = [
    LinkType.BODY,
    LinkType.INPUT,
]
baseLinkList, baseJointList = parseURDF(baseUnitFilepath)
BaseUnit = Unit("baseUnit", baseLinkList, baseLinkTpyeList, baseJointList)

inputUnitCnt = 0
inputUnitFilepath = "/home/masa/learning/rl/undergraduate/robot_design_sythesis/Tests/urdf_generator/OTesst/input_connector_l.SLDASM/urdf/input_connector_l.SLDASM.urdf"
inputLinkTpyeList = [
    LinkType.OUTPUT,
    LinkType.BODY,
    LinkType.INPUT,
]
inputLinkList, inputJointList = parseURDF(inputUnitFilepath)
InputUnit = Unit("inputUnit", inputLinkList, inputLinkTpyeList, inputJointList)

jointUnitCnt = 0
jointTUnitFilepath = "/home/masa/learning/rl/undergraduate/robot_design_sythesis/Tests/urdf_generator/OTesst/joint-T-l.SLDASM/urdf/joint-T-l.SLDASM.urdf"
jointTLinkTpyeList = [
    LinkType.OUTPUT,
    LinkType.BODY,
    LinkType.INPUT
]
jointTLinkList, jointTJointList = parseURDF(jointTUnitFilepath)
JointTUnit = Unit("jointTUnit", jointTLinkList, jointTLinkTpyeList, jointTJointList)
JointTUnit.rotate(pi/2)

straightLinkUnitCnt = 0
straightLinkUnitLFilepath = "/home/masa/learning/rl/undergraduate/robot_design_sythesis/Tests/urdf_generator/OTesst/straight_link.SLDASM/urdf/straight_link.SLDASM.urdf"
straightLinkUnitLLinkTpyeList = [
    LinkType.OUTPUT,
    LinkType.BODY,
    LinkType.OUTPUT
]
straightLinkUnitLLinkList, straightLinkUnitLJointList = parseURDF(straightLinkUnitLFilepath)
StraightLinkUnitL = Unit("straightLinkUnitL", straightLinkUnitLLinkList, straightLinkUnitLLinkTpyeList, straightLinkUnitLJointList)
# StraightLinkUnitL.rotate(pi/2)

cornerLinkUnitCnt = 0
cornerLinkLUnitFilepath = "/home/masa/learning/rl/undergraduate/robot_design_sythesis/Tests/urdf_generator/OTesst/corner_link.SLDASM/urdf/corner_link.SLDASM.urdf"
cornerLinkLUnitLinkTpyeList = [
    LinkType.INPUT,
    LinkType.BODY,
    LinkType.INPUT
]
cornerLinkLUnitLinkList, cornerLinkLUnitJointList = parseURDF(cornerLinkLUnitFilepath)
CornerLinkLUnit = Unit("cornerLinkLUnit", cornerLinkLUnitLinkList, cornerLinkLUnitLinkTpyeList, cornerLinkLUnitJointList)
CornerLinkLUnit.rotate(pi)

endEffectorUnitCnt = 0
endEffectorLinkUnitFilepath = "/home/masa/learning/rl/undergraduate/robot_design_sythesis/Tests/urdf_generator/OTesst/end_effector_1.SLDASM/urdf/end_effector_1.SLDASM.urdf"
endEffectorLinkUnitLinkTpyeList = [
    LinkType.INPUT,
    LinkType.BODY,
]
endEffectorLinkUnitLinkList, endEffectorLinkUnitJointList = parseURDF(endEffectorLinkUnitFilepath)
EndEffectorLinkUnit = Unit("endEffectorLinkUnit", endEffectorLinkUnitLinkList, endEffectorLinkUnitLinkTpyeList, endEffectorLinkUnitJointList)

endEffector2UnitCnt = 0
endEffector2LinkUnitFilepath = "/home/masa/learning/rl/undergraduate/robot_design_sythesis/Tests/urdf_generator/OTesst/end_effector_2.SLDASM/urdf/end_effector_2.SLDASM.urdf"
endEffector2LinkUnitLinkTpyeList = [
    LinkType.INPUT,
    LinkType.BODY,
]
endEffector2LinkUnitLinkList, endEffector2LinkUnitJointList = parseURDF(endEffector2LinkUnitFilepath)
EndEffector2LinkUnit = Unit("endEffector2LinkUnit", endEffector2LinkUnitLinkList, endEffector2LinkUnitLinkTpyeList, endEffector2LinkUnitJointList)

# generate unit
def generateUnit(unitType, order=UnitOrder.NORMAL):
    global baseUnitCnt, inputUnitCnt, jointUnitCnt, straightLinkUnitCnt, cornerLinkUnitCnt, endEffectorUnitCnt, endEffector2UnitCnt
    if unitType == UnitType.BASEL:
        unit = deepcopy(BaseUnit) if order == UnitOrder.NORMAL else deepcopy(BaseUnit).reverse()
        unit.set_name("baseUnit" + str(baseUnitCnt))
        baseUnitCnt += 1
    elif unitType == UnitType.CONNECTORL:
        unit = deepcopy(InputUnit) if order == UnitOrder.NORMAL else deepcopy(InputUnit).reverse()
        unit.set_name("inputUnit" + str(inputUnitCnt))
        inputUnitCnt += 1
    elif unitType == UnitType.JOINTL:
        unit = deepcopy(JointTUnit) if order == UnitOrder.NORMAL else deepcopy(JointTUnit).reverse()
        unit.set_name("jointTUnit" + str(jointUnitCnt))
        jointUnitCnt += 1
    elif unitType == UnitType.STRAIGHTLINKL:
        unit = deepcopy(StraightLinkUnitL) if order == UnitOrder.NORMAL else deepcopy(StraightLinkUnitL).reverse()
        unit.set_name("straightLinkUnitL" + str(straightLinkUnitCnt))
        straightLinkUnitCnt += 1
    elif unitType == UnitType.CORNERLINKL:
        unit = deepcopy(CornerLinkLUnit) if order == UnitOrder.NORMAL else deepcopy(CornerLinkLUnit).reverse()
        unit.set_name("cornerLinkLUnit" + str(cornerLinkUnitCnt))
        cornerLinkUnitCnt += 1
    elif unitType == UnitType.ENDEFFECTORL:
        unit = deepcopy(EndEffector2LinkUnit) if order == UnitOrder.NORMAL else deepcopy(EndEffector2LinkUnit).reverse()
        unit.set_name("endEffectorLink2Unit" + str(endEffector2UnitCnt))
        endEffector2UnitCnt += 1
    # elif unitType == UnitType.ENDEFFECTORL:
    #     unit = deepcopy(EndEffectorLinkUnit) if order == UnitOrder.NORMAL else deepcopy(EndEffectorLinkUnit).reverse()
    #     unit.set_name("endEffectorLinkUnit" + str(endEffectorUnitCnt))
    #     endEffectorUnitCnt += 1
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