from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtlebot_gui_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('*.sdf')),
        (os.path.join('share', package_name), ['make_maze.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mythos',
    maintainer_email='mythos@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
          'turtlebot_gui_qt = turtlebot_gui_control.turtlebot_gui_qt:main',
          'auto_control = turtlebot_gui_control.auto_control:main',
          'manual_control = turtlebot_gui_control.manual_control:main',
        ],
    },
)
