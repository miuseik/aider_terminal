from setuptools import find_packages, setup

package_name = 'src'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
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
            'main_cli = src.app:main_cli',
        ],
    },
)
