from enum import Enum


# 连杆类型枚举类
class LinkType(Enum):
    BODY = 0   # 实体连杆
    INPUT = 1  # 虚拟输入连杆，用于连接输入端（定位）
    OUTPUT = 2 # 虚拟输出连杆

# 接口大小
class InterfaceSize(Enum):
    LARGE = 0
    MEDIUM = 1
    SMALL = 2

# 安装角度
class MountingAngle(Enum):
    ZERO = 0
    NINETY = 1
    ONEEIGHTY = 2
    TWOSEVENTY = 3

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
    STRAIGHTLINKLL = 40 # 连杆-大大
    STRAIGHTLINKLM = 41 # 连杆-大中
    STRAIGHTLINKMM = 42 # 连杆-中中
    STRAIGHTLINKMS = 43 # 连杆-中小
    STRAIGHTLINKSS = 44 # 连杆-小小
    CORNERLINKLL = 50 # 弯头-大大
    CORNERLINKLM = 51 # 弯头-大中
    CORNERLINKMM = 52 # 弯头-中中
    CORNERLINKMS = 53 # 弯头-中小
    CORNERLINKSS = 54 # 弯头-小小
    ENDEFFECTORL = 60 # 输出端-大
    ENDEFFECTORM = 61 # 输出端-中
    ENDEFFECTORS = 62 # 输出端-小

class ModuleType(Enum):
    NONE = 0  # 无
    # BASEL = 10 # 基座-大
    BASEL = 3230000 # 基座-大
    BASEM = 3230001 # 基座-中
    # BASES = 3230002 # 基座-小
    # CONNECTORL = 21 # 输入端-大
    CONNECTORL = 3230011 # 输入端-大
    CONNECTORM = 3230012 # 输入端-中
    CONNECTORS = 3230013 # 输入端-小
    # JOINTL = 30 # 关节-大
    JOINTL = 3230021 # 关节-大
    JOINTM = 3230022 # 关节-中
    JOINTS = 3230023 # 关节-小
    # STRAIGHTLINKLL = 40 # 连杆-大大
    STRAIGHTLINKLL = 3230031 # 连杆-大大
    STRAIGHTLINKLM = 3230032 # 连杆-大中
    STRAIGHTLINKMM = 3230033 # 连杆-中中
    STRAIGHTLINKMS = 3230034 # 连杆-中小
    STRAIGHTLINKSS = 3230035 # 连杆-小小
    # CORNERLINKLL = 50 # 弯头-大大
    CORNERLINKLL = 3230041 # 弯头-大大
    CORNERLINKLM = 3230042 # 弯头-大中
    CORNERLINKMM = 3230043 # 弯头-中中
    CORNERLINKMS = 3230044 # 弯头-中小
    CORNERLINKSS = 3230045 # 弯头-小小
    # ENDEFFECTORL = 60 # 输出端-大
    ENDEFFECTORL = 3230051 # 输出端-大
    ENDEFFECTORM = 3230052 # 输出端-中
    ENDEFFECTORS = 3230053 # 输出端-小



if __name__ == '__main__':
    a = [UnitOrder.NORMAL] * 3

    b = list(map(reverse_order,list(reversed(a))))

    print(a)
    print(b)