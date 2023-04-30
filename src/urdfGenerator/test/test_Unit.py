'''
Unit类的单元测试
1. 导入UnitConfig，并测试所有可以生成的Unit
'''
import unittest
from urdfGenerator.UnitConfig import generateUnit, unitTypeList

from urdfGenerator.Unit import Unit

class TestUnit(unittest.TestCase):
    def test_unit(self):
        for unitType in unitTypeList:
            unit = generateUnit(unitType)
            # self.assertEqual(unit['id'], unitType)
            self.assertIsInstance(unit, Unit)
            # 保证每个unit的linkTypeDeque非空
            self.assertTrue(len(unit.linkTypeDeque) > 0)


if __name__ == '__main__':
    unittest.main()