import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'aiderminal'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'numpy<2',
        'requests',
        'websockets',
        'pyyaml',
        'scipy',
        'opencv-python',
        'pybullet',
        'pyserial',
        'trimesh',
        'aiortc>=1.7.0',
        'av>=11.0.0',
        'pin-pink',
        'qpsolvers[quadprog]',
        'meshcat_shapes',
        'loop_rate_limiters',
    ],
    zip_safe=True,
    maintainer='miuseik',
    maintainer_email='miuseik@tencent.com',
    description='Aider 遥操作机器人共享 Python 库',
    license='MIT',
    entry_points={
        'console_scripts': [
            'terminal_node = aiderminal.nodes.terminal_node:main',
            'main_cli = aiderminal.app:main_cli',
        ],
    },
)
