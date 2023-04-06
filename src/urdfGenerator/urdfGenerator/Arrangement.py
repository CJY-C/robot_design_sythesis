from urdfGenerator.ModuleConfig import generateModule, getAttachableSubModuleActions, getModuleTypeList

from urdfGenerator.Enums import ModuleType, UnitOrder, LinkType

from functools import reduce

from copy import deepcopy

import numpy as np

class Arrangement:
    def __init__(self) -> None:
        self.moduleList = []
        self.moduleTypeList = []
        self.moduleCnt = 0
        self.arrangement = None

    def addModule(self, moduleType):
        if self.moduleCnt == 0:
            self.moduleList.append(generateModule(moduleType))
        else: 
            if self.moduleList[-1].check_attachable(generateModule(moduleType)):
                self.moduleList.append(generateModule(moduleType))
            elif self.moduleList[-1].check_attachable(generateModule(moduleType, UnitOrder.REVERSE)):
                self.moduleList.append(generateModule(moduleType, UnitOrder.REVERSE))
            else:
                # print("Cannot attach module")
                return False
        self.moduleTypeList.append(moduleType)
        self.moduleCnt += 1

        if self.arrangement is not None:
            del self.arrangement
        self.arrangement = reduce(lambda x, y: x + y, self.moduleList)

        return True

    def checkEEAttached(self):
        if self.moduleTypeList[-1] == ModuleType.ENDEFFECTORL or \
           self.moduleTypeList[-1] == ModuleType.ENDEFFECTORM or \
           self.moduleTypeList[-1] == ModuleType.ENDEFFECTORS:
            return True
        else:
            return False

    def checkJointAttached(self):
        if self.moduleTypeList[-1] == ModuleType.JOINTL or \
           self.moduleTypeList[-1] == ModuleType.JOINTM or \
           self.moduleTypeList[-1] == ModuleType.JOINTS:
            return True
        else:
            return False
    
    @property
    def jointNum(self):
        return len(list(filter(lambda x: x == ModuleType.JOINTL or x == ModuleType.JOINTM or x == ModuleType.JOINTS, self.moduleTypeList)))
        # return len(list(filter(lambda x: x == ModuleType.JOINTL or x == ModuleType.JOINTM or x == ModuleType.JOINTS, self.moduleTypeList)))

    @property
    def totalMass(self):
        return reduce(lambda x, y: x + y, list(map(lambda x: x.mass, self.moduleList)))
    
    @property
    def attachedMass(self):
        return self.moduleList[-1].mass

    def __str__(self) -> str:
        return str(reduce(lambda x, y: x + y, self.moduleList))

    def exportURDF(self, filepath, filename):
        import os
        # dir = os.path.dirname(os.path.realpath(filepath))
        exportPath = filepath + '/'+filename+'.urdf'
        with open(exportPath, 'w') as f:
            f.write(str(self))
        return exportPath
    
    def getModuleTypeList(self, maxModuleCnt = None):
        # 深拷贝一份self.moduleTypeList
        moduleTypeList = deepcopy(self.moduleTypeList)
        if maxModuleCnt == None:
            return moduleTypeList
        else:
            if maxModuleCnt > len(self.moduleTypeList):
                # 多出来的长度用NONE填充
                moduleTypeList = moduleTypeList + [ModuleType.NONE] * (maxModuleCnt - len(self.moduleTypeList))
            else:
                moduleTypeList = moduleTypeList[:maxModuleCnt]
    
        # 将moduleTypeList中的枚举值转换为int
        test = getModuleTypeList()
        moduleTypeList = list(map(lambda x: test.index(x) if test.count(x) else -1, moduleTypeList))
        # moduleTypeList = list(map(lambda x: x.value, moduleTypeList))
        return moduleTypeList
    
    def getAttachableSubModuleActions(self):
        moduleType = self.moduleTypeList[-1]
        linkType = self.arrangement.get_head_link_type()
        return np.array(getAttachableSubModuleActions(moduleType, linkType))


if __name__ == '__main__':

    a = Arrangement()

    a.addModule(ModuleType.BASEL) # BASEL对应 323000
    a.addModule(ModuleType.JOINTL) # JOINTL对应 323001
    a.addModule(ModuleType.JOINTL)

    # 打印模块id
    print(a.moduleTypeList)
    

































    # print(a.getModuleTypeList(10))
    # print(a.jointNum)
    # print(a.totalMass)
    # action_space = a.getAttachableSubModuleActions()
    # print(action_space)
    # print(action_space.shape)
    # print( np.choose(np.random.randint(len(action_space)), action_space) )