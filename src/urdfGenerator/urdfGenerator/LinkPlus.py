from odio_urdf import *
import xml.etree.ElementTree as ET

import numpy as np
import modern_robotics as mr


class Link(Link):
    allowed_elements = ['Inertial','Visual','Collision','Self_collision_checking', 'Contact']

    def __init__(self, *args, **kwargs):
        self.prefix = ""
        super().__init__(*args, **kwargs)

    def set_name_prefix(self, prefix):
        if self.prefix == "":
            self.name = prefix + '_' + self.name
        else:
            self.name = self.name.replace(self.name[:self.name.find('_')], prefix, 1)
        self.prefix = prefix


if __name__ == '__main__':
    link1 = Link(name="link1")

    link1.set_name_prefix("prefix")
    link1.set_name_prefix("prx")
    link1.set_name_prefix("px")

    print(link1)