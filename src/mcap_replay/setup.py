from setuptools import setup
import os
from glob import glob

package_name = 'mcap_replay'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 添加 launch 文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='MCAP data replay for BEVNet Nav2 system',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'mcap_player = mcap_replay.mcap_player:main',
            'kitti_converter = mcap_replay.kitti_converter:main',
            'data_synchronizer = mcap_replay.data_synchronizer:main',
        ],
    },
)