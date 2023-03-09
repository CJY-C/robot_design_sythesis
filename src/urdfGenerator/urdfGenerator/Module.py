from urdfGenerator.UnitConfig import generateUnit
from urdfGenerator.Enums import UnitType, UnitOrder, LinkType, reverse_order

from copy import deepcopy

from urdfGenerator.LinkPlus import Link
from urdfGenerator.JointPlus import Joint

from functools import reduce

class Module:
    def __init__(self, name, unitTypeList, unitOrder=None) -> None:
        self.unitTypeList = unitTypeList
        self.unitList = []
        self.unitOrderList = [UnitOrder.NORMAL] * len(unitTypeList) if unitOrder is None else unitOrder
        self.generateUnitList()
        # self.module = reduce(lambda x, y: x + y, self.unitList)
        self.set_name(name)

    def __add__(self, other) -> 'Module':
        unitTypeList = deepcopy(self.unitTypeList)
        otherUnitTypeList = deepcopy(other.unitTypeList)
        unitOrderList = deepcopy(self.unitOrderList)
        otherUnitOrderList = deepcopy(other.unitOrderList)
        return Module(self.name, unitTypeList + otherUnitTypeList, unitOrderList + otherUnitOrderList)
        # return Module(self.name + "+" + other.name, unitTypeList + otherUnitTypeList, unitOrderList + otherUnitOrderList)

    def __str__(self) -> str:
        # return str(self.module)
        return str(reduce(lambda x, y: x + y, self.unitList))

    def set_name(self, name):
        self.name = name
        # self.module.set_name_prefix(name)
        for unit in self.unitList:
            unit.set_name_prefix(name)

    def set_name_prefix(self, prefix):
        self.set_name(prefix + '_' + self.name)
    
    def check_attachable(self, other):
        # if self.unitList[-1].check_attachable(other.unitList[0]):
        if self.module.check_attachable(other.module):
            return True
        return False

    def reverse(self):
        unitOrderList = deepcopy(self.unitOrderList)
        unitOrderList = list(map(reverse_order,list(reversed(unitOrderList))))
        unitTypeList = deepcopy(list(reversed(self.unitTypeList)))
        return Module(self.name + "_reverse", unitTypeList, unitOrderList)

    def rotate(self, angle):
        self.unitList[0].rotate(angle)

    def generateUnitList(self):
        zipTO = list(zip(self.unitTypeList, self.unitOrderList))

        for unitType, unitOrder in zipTO:
            self.unitList.append(generateUnit(unitType, unitOrder))
        self.module = reduce(lambda x, y: x + y, self.unitList)
        # return [generateUnit(unitType) for unitType in self.unitTypeList]


if __name__ == '__main__':
    module1 = Module("module1", [UnitType.BASEL, UnitType.CONNECTORL])
    module2 = Module("module2", [UnitType.JOINTL, UnitType.CONNECTORL])
    module3 = Module("module3", [UnitType.JOINTL, UnitType.CONNECTORL])
    module4 = Module("module4", [UnitType.STRAIGHTLINKL])
    module5 = Module("module5", [UnitType.CONNECTORL, UnitType.CORNERLINKL, UnitType.CONNECTORL])
    module6 = Module("module6", [UnitType.JOINTL, UnitType.CONNECTORL])
    module7 = Module("module7", [UnitType.STRAIGHTLINKL])
    module8 = Module("module8", [UnitType.JOINTL, UnitType.CONNECTORL])
    module9 = Module("module9", [UnitType.JOINTL, UnitType.CONNECTORL])
    module10 = Module("module10", [UnitType.JOINTL, UnitType.CONNECTORL])
    module11 = Module("module11", [UnitType.ENDEFFECTORL])

    print(module1)

    # module3 = module8.reverse()

    # print(module4.unitList[-1].linkTypeDeque)
    # print(module5.unitList[0].linkTypeDeque)
    # print(module5.unitList[0].reverse().linkTypeDeque)
    m5r = module5.reverse()
    m6r = module6.reverse()
    m8r = module8.reverse()
    m9r = module9.reverse()
    m10r = module10.reverse()

    print(module1.check_attachable(module2))
    print(module2.check_attachable(module3))
    print(module3.check_attachable(module4))
    print(module4.check_attachable(m5r))
    print(m5r.check_attachable(module6))
    print(module6.check_attachable(module7))
    print(module7.check_attachable(m8r))
    print(m8r.check_attachable(m9r))
    print(m9r.check_attachable(m10r))
    print(m10r.check_attachable(module11))

    module3 = module11

    import os
    dir = os.path.dirname(os.path.realpath(__file__))
    with open(dir + '/unit7.urdf', 'w') as f:
        f.write(module3.__str__())