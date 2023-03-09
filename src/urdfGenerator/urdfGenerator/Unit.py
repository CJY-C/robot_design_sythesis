from collections import deque
from odio_urdf import *
from urdfGenerator.Enums import LinkType, UnitOrder, UnitType

from copy import deepcopy

from urdfGenerator.LinkPlus import Link
from urdfGenerator.JointPlus import Joint

import numpy as np
import modern_robotics as mr

from functools import reduce


# 模块基本单元类
class Unit:
    def __init__(self, name, linkList, linkTypeList, jointList, order=UnitOrder.NORMAL) -> None:
        self.linkDeque = deque(linkList)
        self.linkTypeDeque = deque(linkTypeList)
        self.jointDeque = deque(jointList)
        self.set_name(name=name)
        self.order = order
        self.prefix = ""

    def __add__(self, other):
        # 深拷贝防止修改原始数据
        if self.linkTypeDeque[-1] == LinkType.OUTPUT and other.linkTypeDeque[0] == LinkType.INPUT:
            linkDeque = deepcopy(self.linkDeque); otherLinkDeque = deepcopy(other.linkDeque)
            linkTypeDeque = deepcopy(self.linkTypeDeque); otherLinkTypeDeque = deepcopy(other.linkTypeDeque)
            jointDeque = deepcopy(self.jointDeque); otherJointDeque = deepcopy(other.jointDeque)

            linkDeque.pop(); linkTypeDeque.pop()
            inputLinkName = otherLinkDeque[0].name
            jointDeque[-1].set_child(inputLinkName)

            return Unit(self.name, list(linkDeque + otherLinkDeque), list(linkTypeDeque + otherLinkTypeDeque), list(jointDeque + otherJointDeque))
            # return Unit(self.name + '+' + other.name, list(linkDeque + otherLinkDeque), list(linkTypeDeque + otherLinkTypeDeque), list(jointDeque + otherJointDeque))
        elif other.linkTypeDeque[0] == LinkType.OUTPUT and self.linkTypeDeque[-1] == LinkType.INPUT:
            linkDeque = deepcopy(self.linkDeque); otherLinkDeque = deepcopy(other.linkDeque)
            linkTypeDeque = deepcopy(self.linkTypeDeque); otherLinkTypeDeque = deepcopy(other.linkTypeDeque)
            jointDeque = deepcopy(self.jointDeque); otherJointDeque = deepcopy(other.jointDeque)

            otherLinkDeque.popleft(); otherLinkTypeDeque.popleft()
            inputLinkName = linkDeque[-1].name
            otherJointDeque[0].set_parent(inputLinkName)

            return Unit(self.name, list(linkDeque + otherLinkDeque), list(linkTypeDeque + otherLinkTypeDeque), list(jointDeque + otherJointDeque))
            # return Unit(self.name + '+' + other.name, list(linkDeque + otherLinkDeque), list(linkTypeDeque + otherLinkTypeDeque), list(jointDeque + otherJointDeque))
        elif (self.linkTypeDeque[-1] == LinkType.OUTPUT and other.linkTypeDeque[-1] == LinkType.INPUT) or\
            (self.linkTypeDeque[-1] == LinkType.INPUT and other.linkTypeDeque[-1] == LinkType.OUTPUT):
            otherR = other.reverse()
            linkDeque = deepcopy(self.linkDeque); otherLinkDeque = deepcopy(otherR.linkDeque)
            linkTypeDeque = deepcopy(self.linkTypeDeque); otherLinkTypeDeque = deepcopy(otherR.linkTypeDeque)
            jointDeque = deepcopy(self.jointDeque); otherJointDeque = deepcopy(otherR.jointDeque)

            linkDeque.pop(); linkTypeDeque.pop()
            inputLinkName = otherLinkDeque[0].name
            jointDeque[-1].set_child(inputLinkName)

            return Unit(self.name, list(linkDeque + otherLinkDeque), list(linkTypeDeque + otherLinkTypeDeque), list(jointDeque + otherJointDeque))
            # return Unit(self.name + '+' + other.name, list(linkDeque + otherLinkDeque), list(linkTypeDeque + otherLinkTypeDeque), list(jointDeque + otherJointDeque))
        elif (other.linkTypeDeque[0] == LinkType.OUTPUT and self.linkTypeDeque[0] == LinkType.INPUT) or \
            (other.linkTypeDeque[0] == LinkType.INPUT and self.linkTypeDeque[0] == LinkType.OUTPUT):
            selfR = self.reverse()
            linkDeque = deepcopy(selfR.linkDeque); otherLinkDeque = deepcopy(other.linkDeque)
            linkTypeDeque = deepcopy(selfR.linkTypeDeque); otherLinkTypeDeque = deepcopy(other.linkTypeDeque)
            jointDeque = deepcopy(selfR.jointDeque); otherJointDeque = deepcopy(other.jointDeque)

            otherLinkDeque.popleft(); otherLinkTypeDeque.popleft()
            inputLinkName = linkDeque[-1].name
            otherJointDeque[0].set_parent(inputLinkName)

            return Unit(self.name, list(linkDeque + otherLinkDeque), list(linkTypeDeque + otherLinkTypeDeque), list(jointDeque + otherJointDeque))
            # return Unit(self.name + '+' + other.name, list(linkDeque + otherLinkDeque), list(linkTypeDeque + otherLinkTypeDeque), list(jointDeque + otherJointDeque))
        return None

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
        if self.order == UnitOrder.REVERSE:
            self.order = UnitOrder.NORMAL
        else:
            self.order = UnitOrder.REVERSE
        for joint in jointDeque:
            joint.reverse_joint()
        return Unit(self.name + "_reverse", list(reversed(linkDeque)), list(reversed(linkTypeDeque)), list(reversed(jointDeque)))

    def check_attachable(self, other: 'Unit'):
        if self.check_head_type(LinkType.OUTPUT) and other.check_tail_type(LinkType.INPUT):
            return True
        elif self.check_head_type(LinkType.INPUT) and other.check_tail_type(LinkType.OUTPUT):
            return True
        return False

    def check_head_type(self, linkType):
        return self.linkTypeDeque[-1] == linkType

    def check_tail_type(self, linkType):
        return self.linkTypeDeque[0] == linkType

    def rotate(self, angle):
        self.jointDeque[0].rotate(angle)


if __name__ == '__main__':
    from urdfGenerator.UnitConfig import generateUnit
    print("Unit class test")
    ju = generateUnit(UnitType.JOINTL)

    ju.rotate(0.5)
    print(ju)

    print(ju.check_head_type(LinkType.OUTPUT))
    print(ju.check_tail_type(LinkType.OUTPUT))
