import os

from setuptools import setup

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 与 rviz 配置（供 ros2 launch 查找）
        ('share/' + package_name + '/launch',
            ['launch/' + f for f in os.listdir('launch')
             if f.endswith('.launch.py')]),
        ('share/' + package_name + '/rviz',
            ['rviz/' + f for f in os.listdir('rviz')
             if f.endswith('.rviz')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@example.com',
    description='启动编排：系统 launch 入口',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
