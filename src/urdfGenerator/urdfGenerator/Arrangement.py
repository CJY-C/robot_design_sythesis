from urdfGenerator.ModuleConfig import generateModule

from urdfGenerator.Enums import ModuleType, UnitOrder

from functools import reduce

class Arrangement:
    def __init__(self) -> None:
        self.moduleList = []
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
        self.moduleCnt += 1
        return True

    def __str__(self) -> str:
        return str(reduce(lambda x, y: x + y, self.moduleList))

    def exportURDF(self, filepath, filename):
        import os
        dir = os.path.dirname(os.path.realpath(filepath))
        with open(dir + '/'+filename+'.urdf', 'w') as f:
            f.write(str(self))


if __name__ == '__main__':

    a = Arrangement()

    a.addModule(ModuleType.BASEL)
    a.addModule(ModuleType.JOINTL)

    print(a)