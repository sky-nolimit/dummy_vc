import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'dummy_vc_calibration'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dummy VC',
    maintainer_email='dummy-vc@localhost',
    description='阶段 0 硬件标定节点（预览-确认-执行两段式）',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stage0_calibrate = dummy_vc_calibration.stage0_calibrate:main',
        ],
    },
)
