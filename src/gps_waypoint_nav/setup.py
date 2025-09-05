from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'gps_waypoint_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch文件
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # 配置文件
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        # 数据文件夹
        (os.path.join('share', package_name, 'data'),
            glob('data/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='GPS waypoint navigation for orchard robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'extract_gps_from_mcap = gps_waypoint_nav.gps_extractor:main',
            'gps_path_publisher = gps_waypoint_nav.gps_path_publisher:main',
        ],
    },
)