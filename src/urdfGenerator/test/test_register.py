# 测试用户注册资源路径的单例模式

import unittest
from urdfGenerator.register import PathRegister

class TestRegister(unittest.TestCase):

    def test_register(self):
        register = PathRegister()
        register.add_path('小老板')
        self.assertEqual(register.get_paths()[0], '小老板')

if __name__ == '__main__':
    unittest.main()
