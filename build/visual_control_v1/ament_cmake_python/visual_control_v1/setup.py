from setuptools import find_packages
from setuptools import setup

setup(
    name='visual_control_v1',
    version='1.0.0',
    packages=find_packages(
        include=('visual_control_v1', 'visual_control_v1.*')),
)
