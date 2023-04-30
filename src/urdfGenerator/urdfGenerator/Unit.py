from collections import deque
from odio_urdf import *
from urdfGenerator.Enums import LinkType, Order, UnitType

from copy import deepcopy

from urdfGenerator.LinkPlus import Link
from urdfGenerator.JointPlus import Joint

import numpy as np
import modern_robotics as mr

from functools import reduce


# 模块基本单元类
class Unit:
    def __init__(self, name, linkList, linkTypeList, jointList, interfaceSize, order=Order.NORMAL) -> None:
        self.linkDeque = deque(linkList)
        self.linkTypeDeque = deque(linkTypeList)

        self.jointDeque = deque(jointList)
        self.interfaceSizeDeque = deque(interfaceSize)

        self.prefix = ""
        self.set_name(name=name)

        self.order = order


    def _merge_units(self, unit1, unit2, flag=True):
        '''
        合并两个模块单元

        Args:
            unit1: Unit类型，表示当前模块单元
            unit2: Unit类型，表示另一个模块单元
            flag: bool类型，表示是否为正向连接
            
        Returns:
            合并后新的模块单元
        '''
        linkDeque = deepcopy(unit1.linkDeque)
        linkTypeDeque = deepcopy(unit1.linkTypeDeque)
        jointDeque = deepcopy(unit1.jointDeque)
        interfaceSizeDeque = deepcopy(unit1.interfaceSizeDeque)

        otherLinkDeque = deepcopy(unit2.linkDeque)
        otherLinkTypeDeque = deepcopy(unit2.linkTypeDeque)
        otherJointDeque = deepcopy(unit2.jointDeque)
        otherInterfaceSizeDeque = deepcopy(unit2.interfaceSizeDeque)

        if flag:
            linkDeque.pop(); linkTypeDeque.pop()
            interfaceSizeDeque.pop()
            inputLinkName = otherLinkDeque[0].name
            jointDeque[-1].set_child(inputLinkName)
        else:
            otherLinkDeque.popleft(); otherLinkTypeDeque.popleft(); otherInterfaceSizeDeque.popleft()
            inputLinkName = linkDeque[-1].name
            otherJointDeque[0].set_parent(inputLinkName)

        return Unit(self.name,
                    list(linkDeque + otherLinkDeque),
                    list(linkTypeDeque + otherLinkTypeDeque),
                    list(jointDeque + otherJointDeque),
                    list(interfaceSizeDeque + otherInterfaceSizeDeque))

    def __add__(self, other: 'Unit') -> 'Unit':
        if self.check_attachable(other):
            if self.check_head_type(LinkType.OUTPUT):
                return self._merge_units(self, other)
            elif other.check_tail_type(LinkType.OUTPUT):
                return self._merge_units(self, other, flag=False)
        elif self.check_attachable(other.reverse()):
            otherR = other.reverse()
            if self.check_head_type(LinkType.OUTPUT):
                return self._merge_units(self, otherR)
            elif otherR.check_tail_type(LinkType.OUTPUT):
                return self._merge_units(self, otherR, flag=False)
        elif self.reverse().check_attachable(other):
            selfR = self.reverse()
            if selfR.check_head_type(LinkType.OUTPUT):
                return self._merge_units(selfR, other)
            elif other.check_tail_type(LinkType.OUTPUT):
                return self._merge_units(selfR, other, flag=False)
        return None

    # def __add__(self, other: 'Unit') -> 'Unit':
    #     # 深拷贝防止修改原始数据
    #     if self.check_attachable(other):
    #         if self.check_head_type(LinkType.OUTPUT):
    #             linkDeque = deepcopy(self.linkDeque); otherLinkDeque = deepcopy(other.linkDeque)
    #             linkTypeDeque = deepcopy(self.linkTypeDeque); otherLinkTypeDeque = deepcopy(other.linkTypeDeque)
    #             jointDeque = deepcopy(self.jointDeque); otherJointDeque = deepcopy(other.jointDeque)
    #             interfaceSizeDeque = deepcopy(self.interfaceSizeDeque); otherInterfaceSizeDeque = deepcopy(other.interfaceSizeDeque)

    #             linkDeque.pop(); linkTypeDeque.pop(); interfaceSizeDeque.pop()
    #             inputLinkName = otherLinkDeque[0].name
    #             jointDeque[-1].set_child(inputLinkName)

    #             return Unit(self.name, 
    #                         list(linkDeque + otherLinkDeque), 
    #                         list(linkTypeDeque + otherLinkTypeDeque), 
    #                         list(jointDeque + otherJointDeque),
    #                         list(interfaceSizeDeque + otherInterfaceSizeDeque))
    #         elif other.check_tail_type(LinkType.OUTPUT):
    #             linkDeque = deepcopy(self.linkDeque); otherLinkDeque = deepcopy(other.linkDeque)
    #             linkTypeDeque = deepcopy(self.linkTypeDeque); otherLinkTypeDeque = deepcopy(other.linkTypeDeque)
    #             jointDeque = deepcopy(self.jointDeque); otherJointDeque = deepcopy(other.jointDeque)
    #             interfaceSizeDeque = deepcopy(self.interfaceSizeDeque); otherInterfaceSizeDeque = deepcopy(other.interfaceSizeDeque)

    #             otherLinkDeque.popleft(); otherLinkTypeDeque.popleft(); otherInterfaceSizeDeque.popleft()
    #             inputLinkName = linkDeque[-1].name
    #             otherJointDeque[0].set_parent(inputLinkName)

    #             return Unit(self.name, 
    #                         list(linkDeque + otherLinkDeque), 
    #                         list(linkTypeDeque + otherLinkTypeDeque), 
    #                         list(jointDeque + otherJointDeque),
    #                         list(interfaceSizeDeque + otherInterfaceSizeDeque))
    #     elif self.check_attachable(other.reverse()):
    #         otherR = other.reverse()
    #         if self.check_head_type(LinkType.OUTPUT):
    #             linkDeque = deepcopy(self.linkDeque); otherLinkDeque = deepcopy(otherR.linkDeque)
    #             linkTypeDeque = deepcopy(self.linkTypeDeque); otherLinkTypeDeque = deepcopy(otherR.linkTypeDeque)
    #             jointDeque = deepcopy(self.jointDeque); otherJointDeque = deepcopy(otherR.jointDeque)
    #             interfaceSizeDeque = deepcopy(self.interfaceSizeDeque); otherInterfaceSizeDeque = deepcopy(otherR.interfaceSizeDeque)

    #             linkDeque.pop(); linkTypeDeque.pop(); interfaceSizeDeque.pop()
    #             inputLinkName = otherLinkDeque[0].name
    #             jointDeque[-1].set_child(inputLinkName)

    #             return Unit(self.name, 
    #                         list(linkDeque + otherLinkDeque), 
    #                         list(linkTypeDeque + otherLinkTypeDeque), 
    #                         list(jointDeque + otherJointDeque),
    #                         list(interfaceSizeDeque + otherInterfaceSizeDeque))
    #         elif otherR.check_tail_type(LinkType.OUTPUT):
    #             linkDeque = deepcopy(self.linkDeque); otherLinkDeque = deepcopy(otherR.linkDeque)
    #             linkTypeDeque = deepcopy(self.linkTypeDeque); otherLinkTypeDeque = deepcopy(otherR.linkTypeDeque)
    #             jointDeque = deepcopy(self.jointDeque); otherJointDeque = deepcopy(otherR.jointDeque)
    #             interfaceSizeDeque = deepcopy(self.interfaceSizeDeque); otherInterfaceSizeDeque = deepcopy(otherR.interfaceSizeDeque)

    #             otherLinkDeque.popleft(); otherLinkTypeDeque.popleft(); otherInterfaceSizeDeque.popleft()
    #             inputLinkName = linkDeque[-1].name
    #             otherJointDeque[0].set_parent(inputLinkName)

    #             return Unit(self.name, 
    #                         list(linkDeque + otherLinkDeque), 
    #                         list(linkTypeDeque + otherLinkTypeDeque), 
    #                         list(jointDeque + otherJointDeque),
    #                         list(interfaceSizeDeque + otherInterfaceSizeDeque))
    #     elif self.reverse().check_attachable(other):
    #         selfR = self.reverse()
    #         if selfR.check_head_type(LinkType.OUTPUT):
    #             linkDeque = deepcopy(selfR.linkDeque); otherLinkDeque = deepcopy(other.linkDeque)
    #             linkTypeDeque = deepcopy(selfR.linkTypeDeque); otherLinkTypeDeque = deepcopy(other.linkTypeDeque)
    #             jointDeque = deepcopy(selfR.jointDeque); otherJointDeque = deepcopy(other.jointDeque)
    #             interfaceSizeDeque = deepcopy(selfR.interfaceSizeDeque); otherInterfaceSizeDeque = deepcopy(other.interfaceSizeDeque)

    #             linkDeque.pop(); linkTypeDeque.pop(); interfaceSizeDeque.pop()
    #             inputLinkName = otherLinkDeque[0].name
    #             jointDeque[-1].set_child(inputLinkName)

    #             return Unit(self.name, 
    #                         list(linkDeque + otherLinkDeque), 
    #                         list(linkTypeDeque + otherLinkTypeDeque), 
    #                         list(jointDeque + otherJointDeque),
    #                         list(interfaceSizeDeque + otherInterfaceSizeDeque))
    #         elif other.check_tail_type(LinkType.OUTPUT):
    #             linkDeque = deepcopy(selfR.linkDeque); otherLinkDeque = deepcopy(other.linkDeque)
    #             linkTypeDeque = deepcopy(selfR.linkTypeDeque); otherLinkTypeDeque = deepcopy(other.linkTypeDeque)
    #             jointDeque = deepcopy(selfR.jointDeque); otherJointDeque = deepcopy(other.jointDeque)
    #             interfaceSizeDeque = deepcopy(selfR.interfaceSizeDeque); otherInterfaceSizeDeque = deepcopy(other.interfaceSizeDeque)

    #             otherLinkDeque.popleft(); otherLinkTypeDeque.popleft(); otherInterfaceSizeDeque.popleft()
    #             inputLinkName = linkDeque[-1].name
    #             otherJointDeque[0].set_parent(inputLinkName)

    #             return Unit(self.name, 
    #                         list(linkDeque + otherLinkDeque), 
    #                         list(linkTypeDeque + otherLinkTypeDeque), 
    #                         list(jointDeque + otherJointDeque),
    #                         list(interfaceSizeDeque + otherInterfaceSizeDeque))
    #     return None

    def __str__(self):
        # 深拷贝防止修改原始数据
        linkDeque = deepcopy(self.linkDeque)
        jointDeque = deepcopy(self.jointDeque)
        return Robot(*linkDeque, *jointDeque).__str__()

    def set_name(self, name):
        self.name = name
        for link in self.linkDeque:
            link.set_name_prefix(name)
        for joint in self.jointDeque:
            joint.set_name_prefix(name)

    def set_name_prefix(self, prefix):
        if self.prefix == "":
            self.name = prefix + '_' + self.name
        else:
            self.name = self.name.replace(self.name[:self.name.find('_')], prefix, 1)
        self.prefix = prefix
        self.set_name(self.name)


    def reverse(self):
        linkDeque = deepcopy(self.linkDeque)
        linkTypeDeque = deepcopy(self.linkTypeDeque)
        jointDeque = deepcopy(self.jointDeque)
        interfaceSizeDeque = deepcopy(self.interfaceSizeDeque)
        if self.order == Order.REVERSE:
            self.order = Order.NORMAL
        else:
            self.order = Order.REVERSE
        for joint in jointDeque:
            joint.reverse_joint()
        return Unit(self.name + "_reverse", list(reversed(linkDeque)), list(reversed(linkTypeDeque)), list(reversed(jointDeque)), list(reversed(interfaceSizeDeque)))

    def check_attachable(self, other: 'Unit'):
        '''
        检查两个单元是否可以连接:
        1. 两个单元的接口尺寸相同
        2. 保证两个单元的连接类型一个是OUTPUT，一个是INPUT
        '''
        if self.check_head_type(LinkType.OUTPUT) and other.check_tail_type(LinkType.INPUT) \
            and self.interfaceSizeDeque[-1] == other.interfaceSizeDeque[0]:
                return True
        elif self.check_head_type(LinkType.INPUT) and other.check_tail_type(LinkType.OUTPUT) \
            and self.interfaceSizeDeque[-1] == other.interfaceSizeDeque[0]:
                return True
        return False

    def check_head_type(self, linkType):
        return self.linkTypeDeque[-1] == linkType

    def check_tail_type(self, linkType):
        return self.linkTypeDeque[0] == linkType

    def rotate(self, angle):
        self.jointDeque[0].rotate(angle)
    
    @property
    def mass(self):
        mass = 0
        zp = zip(self.linkDeque, self.linkTypeDeque)
        for link, linkType in zp:
            if linkType == LinkType.BODY:
                mass = link.mass
        return float(mass)
    
    @mass.setter
    def mass(self, mass):
        zp = zip(self.linkDeque, self.linkTypeDeque)
        for link, linkType in zp:
            if linkType == LinkType.BODY:
                link.mass = mass


if __name__ == '__main__':
    from urdfGenerator.UnitConfig import generateUnit
    print("Unit class test")
    b =generateUnit(UnitType.JOINTL)
    c = generateUnit(UnitType.CONNECTORL)
    # print(b.mass)
    print(b.linkTypeDeque)
    print(c.linkTypeDeque)
    print(b+c)
    # ju = generateUnit(UnitType.JOINTL)

    # ju.rotate(0.5)
    # print(ju)

    # print(ju.check_head_type(LinkType.OUTPUT))
    # print(ju.check_tail_type(LinkType.OUTPUT))
