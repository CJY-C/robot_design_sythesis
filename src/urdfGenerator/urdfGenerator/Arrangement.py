from urdfGenerator.ModuleConfig import generateModule

from urdfGenerator.Enums import ModuleType, UnitOrder

from functools import reduce

from copy import deepcopy

class Arrangement:
    def __init__(self) -> None:
        self.moduleList = []
        self.moduleTypeList = []
        self.moduleCnt = 0

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
    
    @property
    def totalMass(self):
        return reduce(lambda x, y: x + y, list(map(lambda x: x.mass, self.moduleList)))

    def __str__(self) -> str:
        return str(reduce(lambda x, y: x + y, self.moduleList))

    def exportURDF(self, filepath, filename):
        import os
        # dir = os.path.dirname(os.path.realpath(filepath))
        exportPath = filepath + '/'+filename+'.urdf'
        with open(exportPath, 'w') as f:
            f.write(str(self))
        return exportPath
    
    def getModuleTypeList(self, maxModuleCnt = 0):
        if maxModuleCnt == 0:
            return self.moduleTypeList
        else:
            if maxModuleCnt > len(self.moduleTypeList):
                # 深拷贝一份self.moduleTypeList，多出来的长度用NONE填充
                moduleTypeList = deepcopy(self.moduleTypeList)
                moduleTypeList = moduleTypeList + [ModuleType.NONE] * (maxModuleCnt - len(self.moduleTypeList))
                # 将moduleTypeList中的枚举值转换为int
                moduleTypeList = list(map(lambda x: x.value, moduleTypeList))
                return moduleTypeList
            else:
                return self.moduleTypeList[:maxModuleCnt]
    
    # @property
    # def moduleCnt(self):
    #     return self.moduleCnt


if __name__ == '__main__':

    a = Arrangement()

    a.addModule(ModuleType.BASEL)
    a.addModule(ModuleType.JOINTL)
    a.addModule(ModuleType.JOINTL)

    print(a.getModuleTypeList(10))
    print(a.moduleTypeList)
    print(a.jointNum)
    print(a.totalMass)