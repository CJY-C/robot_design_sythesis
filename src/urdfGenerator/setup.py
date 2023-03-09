# from os.path import join, dirname, realpath
from setuptools import setup
# import sys

# assert sys.version_info.major == 3 and sys.version_info.minor >= 6, \
#     "The Spinning Up repo is designed to work with Python 3.6 and greater." \
#     + "Please install it before proceeding."

# with open(join("spinup", "version.py")) as version_file:
#     exec(version_file.read())

setup(
    name='urdfGenerator',
    py_modules=['urdf_generator'],
    version='0.1',#'0.1',
    install_requires=[
        'odio_urdf',
    ],
    description="可以实现urdf文件的拼接重组功能",
    author="Masa Chen",
)