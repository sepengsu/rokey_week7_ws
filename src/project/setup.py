from setuptools import find_packages, setup
from glob import glob

package_name = 'project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('ui/*.ui')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jaenote',
    maintainer_email='na06219@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui = project.gui:main',
            'conveyor = project.conveyor_node:main',
            'world_cam = project.world_cam:main',
        ],
    },
)
