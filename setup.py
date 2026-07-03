import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'aiderminal'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    py_modules=['app'],  # 顶层 app.py
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='miuseik',
    maintainer_email='miuseik@tencent.com',
    description='Aider 遥操作机器人共享 Python 库',
    license='MIT',
    entry_points={
        'console_scripts': [
            'terminal_node = aiderminal.nodes.terminal_node:main',
        ],
    },
)
