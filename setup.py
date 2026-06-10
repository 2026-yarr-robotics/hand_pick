from setuptools import find_packages, setup
from glob import glob

package_name = 'hand_pick'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ssu',
    maintainer_email='ssu@todo.todo',
    description='Hand-eye view 기반 정밀 pick 모듈 (upright_cup_pose_node + pick_node).',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pick_node = hand_pick.pick_node:main',
            'upright_cup_pose_node = hand_pick.upright_cup_pose_node:main',
        ],
    },
)
