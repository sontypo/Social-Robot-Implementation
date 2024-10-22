import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'force_control_exp'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share/' + package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share/' + package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Saun Nguyen',
    maintainer_email='hongsonnguyen.haui@gmail.com',
    description='Manage to implement the social network - multi robot controller for Scout model.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'force_control_exp_node = force_control_exp.dynamic_updater:main',
            'social_force_pub = force_control_exp.social_force_calulator:main',
            'scan_distances = force_control_exp.getScan:main',
            'get_csv = force_control_exp.get_data:main'
        ],
    },
)
