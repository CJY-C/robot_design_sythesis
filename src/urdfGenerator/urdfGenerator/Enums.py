from enum import Enum


# 连杆类型枚举类
class LinkType(Enum):
    BODY = 0   # 实体连杆
    INPUT = 1  # 虚拟输入连杆，用于连接输入端（定位）
    OUTPUT = 2 # 虚拟输出连杆

# 基本单元顺序
class UnitOrder(Enum):
    NORMAL = 0 # 正向
    REVERSE = 1 # 反向

def reverse_order(order):
    if order == UnitOrder.NORMAL:
        return UnitOrder.REVERSE
    else:
        return UnitOrder.NORMAL

# 基本单元类型
class UnitType(Enum):
    BASEL = 10 # 基座-大
    BASEM = 11 # 基座-中
    BASES = 12 # 基座-小
    CONNECTORL = 20 # 输入端-大
    CONNECTORM = 21 # 输入端-中
    CONNECTORS = 22 # 输入端-小
    JOINTL = 30 # 关节-大
    JOINTM = 31 # 关节-中
    JOINTS = 32 # 关节-小
    STRAIGHTLINKL = 40 # 连杆-大
    STRAIGHTLINKM = 41 # 连杆-中
    STRAIGHTLINKS = 42 # 连杆-小
    CORNERLINKL = 50 # 弯头-大
    CORNERLINKM = 51 # 弯头-中
    CORNERLINKS = 52 # 弯头-小
    ENDEFFECTORL = 60 # 输出端-大
    ENDEFFECTORM = 61 # 输出端-中
    ENDEFFECTORS = 62 # 输出端-小

class ModuleType(Enum):
    BASEL = 0 # 基座-大
    BASEM = 1 # 基座-中
    BASES = 2 # 基座-小
    CONNECTORL = 10 # 输入端-大
    CONNECTORM = 11 # 输入端-中
    CONNECTORS = 12 # 输入端-小
    JOINTL = 20 # 关节-大
    JOINTM = 21 # 关节-中
    JOINTS = 22 # 关节-小
    STRAIGHTLINKL = 30 # 连杆-大
    STRAIGHTLINKM = 31 # 连杆-中
    STRAIGHTLINKS = 32 # 连杆-小
    CORNERLINKL = 40 # 弯头-大
    CORNERLINKM = 41 # 弯头-中
    CORNERLINKS = 42 # 弯头-小
    ENDEFFECTORL = 50 # 输出端-大
    ENDEFFECTORM = 51 # 输出端-中
    ENDEFFECTORS = 52 # 输出端-小



if __name__ == '__main__':
    a = [UnitOrder.NORMAL] * 3

    b = list(map(reverse_order,list(reversed(a))))

    print(a)
    print(b)