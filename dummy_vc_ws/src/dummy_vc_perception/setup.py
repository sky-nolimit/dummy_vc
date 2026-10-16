import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'dummy_vc_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 视觉节点落地时把这三行放开（目录现在还不存在，先不声明，
        # 免得 colcon 装出一堆空目录）：
        # (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # (os.path.join('share', package_name, 'config'), glob('config/*')),
        # (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dummy VC',
    maintainer_email='dummy-vc@localhost',
    description='视觉感知：像素 -> base_link 下的抓取目标',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        # 节点还没写。落地时在这里加一行：
        # 'vision_node = dummy_vc_perception.vision_node:main',
        'console_scripts': [],
    },
)
