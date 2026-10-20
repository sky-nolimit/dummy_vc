import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'dummy_vc_grasp'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 一条命令起"看 + 抓"：pick_place_demo.launch.py
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dummy VC',
    maintainer_email='dummy-vc@example.com',
    description='抓取规划与任务状态机：检测目标 -> MoveIt 路点 -> 抓取/放置',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pick_place_node = dummy_vc_grasp.pick_place_node:main',
        ],
    },
)
