from odio_urdf import *
import xml.etree.ElementTree as ET

import numpy as np
import modern_robotics as mr


def add_property(func):
    def wrapper(self,*args, **kwargs):
        self.origin = Origin(xyz="0 0 0", rpy="0 0 0")
        self.parent = Parent(link="")
        self.child = Child(link="")
        self.axis = Axis(xyz="0 0 0")
        self.prefix = ""

        Args = list(args)
        cnt = 0
        for element in Args:
            if ET.fromstring(element.__str__()).tag == 'origin':
                xyz = ET.fromstring(element.__str__()).attrib['xyz']
                rpy = ET.fromstring(element.__str__()).attrib['rpy']
                self.origin.xyz = xyz
                self.origin.rpy = rpy
                Args[cnt] = self.origin
            elif ET.fromstring(element.__str__()).tag == 'parent':
                parent = ET.fromstring(element.__str__()).attrib['link']
                self.parent.link = parent
                Args[cnt] = self.parent
            elif ET.fromstring(element.__str__()).tag == 'child':
                child = ET.fromstring(element.__str__()).attrib['link']
                self.child.link = child
                Args[cnt] = self.child
            elif ET.fromstring(element.__str__()).tag == 'axis':
                xyz = ET.fromstring(element.__str__()).attrib['xyz']
                self.axis.xyz = xyz
                Args[cnt] = self.axis
            cnt+=1
        return func(self, *Args, **kwargs)
    return wrapper


class Joint(Joint):
    required_elements = ['Parent','Child']
    allowed_elements = ['Origin','Inertial','Visual','Collision','Axis','Calibration','Dynamics','Limit','Mimic','Safety_controller']
    required_attributes = ['name','type']

    @add_property
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def set_parent(self, link):
        self.parent.link = link

    def set_child(self, link):
        self.child.link = link

    def swap_parent_child(self):
        self.parent.link, self.child.link = self.child.link, self.parent.link

    def reverse_joint(self):
        # 交换parent和child
        self.swap_parent_child()
        # 计算每个joint的齐次矩阵
        xyz1 = np.array(self.origin.xyz.split(" ")).astype(np.float)
        rpy1 = np.array(self.origin.rpy.split(" ")).astype(np.float)
        T1 = mr.RpToTrans(mr.MatrixExp3(mr.VecToso3(rpy1)), xyz1)
        # 计算每个joint的T的逆
        T1_inv = mr.TransInv(T1)
        # 将T的逆转换为xyz和rpy，将齐次矩阵转换为旋转矩阵，再使用so3ToVec函数，将旋转矩阵转换为向量
        xyz1_inv = mr.TransToRp(T1_inv)[1]
        rpy1_inv = mr.so3ToVec(mr.MatrixLog3(mr.TransToRp(T1_inv)[0]))
        # 将xyz和rpy转换为字符串
        xyz1_inv_str = " ".join([str(x) for x in xyz1_inv])
        rpy1_inv_str = " ".join([str(x) for x in rpy1_inv])
        # 将改变每个joint的origin的xyz和rpy
        self.origin.xyz = xyz1_inv_str
        self.origin.rpy = rpy1_inv_str
    
    def rotate(self, angle):
        xyz1 = np.array(self.origin.xyz.split(" ")).astype(np.float64)
        rpy1 = np.array(self.origin.rpy.split(" ")).astype(np.float64)
        T1 = mr.RpToTrans(mr.MatrixExp3(mr.VecToso3(rpy1)), xyz1)

        r_xyz = np.array([0, 0, 0]).astype(np.float)
        r_rpy = np.array([0, 0, angle]).astype(np.float)
        T_r = mr.RpToTrans(mr.MatrixExp3(mr.VecToso3(r_rpy)), r_xyz)

        # T1 = T1 @ T_r
        T1 = np.matmul(T1, T_r)

        xyz1 = mr.TransToRp(T1)[1]
        rpy1 = mr.so3ToVec(mr.MatrixLog3(mr.TransToRp(T1)[0]))

        xyz1_str = " ".join([str(x) for x in xyz1])
        rpy1_str = " ".join([str(x) for x in rpy1])

        # 将改变每个joint的origin的xyz和rpy
    
        self.origin.xyz = xyz1_str
        self.origin.rpy = rpy1_str
        


    def set_origin(self, xyz, rpy):
        self.origin.xyz = xyz
        self.origin.rpy = rpy

    def set_axis(self, xyz):
        self.axis.xyz = xyz
    
    def set_name_prefix(self, prefix):
        if self.prefix == "":
            self.name = prefix + '_' + self.name
            self.set_parent(prefix + '_' + self.parent.link)
            self.set_child(prefix + '_' + self.child.link)
        else:
            self.name = self.name.replace(self.name[:self.name.find('_')], prefix, 1)
            self.parent.link = self.parent.link.replace(self.parent.link[:self.parent.link.find('_')], prefix, 1)
            self.child.link = self.child.link.replace(self.child.link[:self.child.link.find('_')], prefix, 1)
        self.prefix = prefix

if __name__ == '__main__':
    joint = Joint(
        Origin(xyz="0 0 0", rpy="0 0 0"),
        Parent(link="pp"),
        Child(link="cc"),
        Axis(xyz="0 0 0"),
        name="123"
        )

    print(joint)
    joint.set_parent("parent_link") 
    joint.set_child("child_link")
    joint.set_axis("0 0 1")
    joint.set_origin("0 0 1", "0 1 0")
    joint.set_name_prefix("prefix")
    joint.set_name_prefix("prfx")
    joint.rotate(0.1)
    print(joint)
