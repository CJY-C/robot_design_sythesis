from odio_urdf import *
import xml.etree.ElementTree as ET

import numpy as np
import modern_robotics as mr

def add_property(func):
    def wrapper(self,*args, **kwargs):
        self.origin = Origin(xyz="0 0 0", rpy="0 0 0")
        self.Mass = Mass(value="0")
        self.inertia = Inertia(ixx="0", ixy="0", ixz="0", iyy="0", iyz="0", izz="0")
        self.inertial = Inertial(
            self.origin,
            self.Mass,
            self.inertia
        )
        self.prefix = ""

        Args = list(args)
        cnt = 0
        for element in Args:
            if ET.fromstring(element.__str__()).tag == 'inertial':
                if ET.fromstring(element.__str__()).find('origin') != None:
                    xyz = ET.fromstring(element.__str__()).find('origin').attrib['xyz']
                    rpy = ET.fromstring(element.__str__()).find('origin').attrib['rpy']
                    self.origin.xyz = xyz
                    self.origin.rpy = rpy
                if ET.fromstring(element.__str__()).find('mass') != None:
                    value = ET.fromstring(element.__str__()).find('mass').attrib['value']
                    self.mass = value
                if ET.fromstring(element.__str__()).find('inertia') != None:
                    ixx = ET.fromstring(element.__str__()).find('inertia').attrib['ixx']
                    ixy = ET.fromstring(element.__str__()).find('inertia').attrib['ixy']
                    ixz = ET.fromstring(element.__str__()).find('inertia').attrib['ixz']
                    iyy = ET.fromstring(element.__str__()).find('inertia').attrib['iyy']
                    iyz = ET.fromstring(element.__str__()).find('inertia').attrib['iyz']
                    izz = ET.fromstring(element.__str__()).find('inertia').attrib['izz']
                    self.inertia.ixx = ixx
                    self.inertia.ixy = ixy
                    self.inertia.ixz = ixz
                    self.inertia.iyy = iyy
                    self.inertia.iyz = iyz
                    self.inertia.izz = izz
                Args[cnt] = self.inertial
            cnt+=1
        return func(self, *Args, **kwargs)
    return wrapper

class Link(Link):
    allowed_elements = ['Inertial','Visual','Collision','Self_collision_checking', 'Contact']

    @add_property
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def set_name_prefix(self, prefix):
        if self.prefix == "":
            self.name = prefix + '_' + self.name
        else:
            self.name = self.name.replace(self.name[:self.name.find('_')], prefix, 1)
        self.prefix = prefix
    
    @property
    def mass(self):
        return self.Mass.value
    
    @mass.setter
    def mass(self, mass):
        self.Mass.value = mass
    # def get_mass(self):
    #     return self.mass.value


if __name__ == '__main__':
    link1 = Link(
        Inertial(
        Origin(xyz="0 0 0", rpy="0 0 0"),
        Mass(value=1),
        Inertia(ixx="0", ixy="0", ixz="0", iyy="0", iyz="0", izz="0")
        ),
        name="link1"
        )

    link1.set_name_prefix("prefix")
    link1.set_name_prefix("prx")
    link1.set_name_prefix("px")

    link1.mass = '2'

    print(link1)
    print(type(link1.mass))