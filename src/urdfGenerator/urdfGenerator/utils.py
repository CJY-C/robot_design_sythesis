from odio_urdf import *
from urdfGenerator.Enums import InterfaceSize, LinkType
from urdfGenerator.LinkPlus import Link
from urdfGenerator.JointPlus import Joint
from urdfGenerator import register
import json

def getConfig():
    config = None
    if register.PathRegister.get_paths() is []:
        raise ValueError('No path registered')
    with open(register.PathRegister.get_paths()[0], 'r') as f:
        config = json.load(f)
    return config


def list_to_dict(lst):
    '''
    将列表转换为字典

    Args: lst

    Returns: result
    '''
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
    '''
    解析URDF文件

    Args: filepath

    Returns: linkList, linkTypeList, jointList, interfaceSizeList
    '''
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