from setuptools import setup

package_name = 'bevnet_nav2_core'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='BEVNet integration with Nav2',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'bevnet_inference_node = bevnet_nav2_core.bevnet_inference_node:main',
            'costmap_fusion = bevnet_nav2_core.costmap_fusion:main',
            'path_visualizer = bevnet_nav2_core.path_visualizer:main',
        ],
    },
)