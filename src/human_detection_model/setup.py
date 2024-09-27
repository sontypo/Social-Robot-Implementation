import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'human_detection_model'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share/' + package_name, 'weights'), glob('weights/*.pt')),
        (os.path.join('share/' + package_name, 'config'), glob('config/*')),
        (os.path.join('share/' + package_name, 'rviz2'), glob('rviz2/*.rviz')),
        (os.path.join('share/' + package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Saun Nguyen',
    maintainer_email='hongsonnguyen.haui@gmail.com',
    description='Human detection using YOLO v8',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'human_detection_node = human_detection_model.human_detection_node:main'
        ],
    },
)
