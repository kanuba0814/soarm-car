from glob import glob
import os
from setuptools import setup
setup(name='soarm_room_navigation', version='0.1.0', packages=['soarm_room_navigation'], data_files=[('share/ament_index/resource_index/packages',['resource/soarm_room_navigation']),('share/soarm_room_navigation',['package.xml']),(os.path.join('share/soarm_room_navigation','launch'),glob('soarm_room_navigation/launch/*.py')),(os.path.join('share/soarm_room_navigation','config'),glob('soarm_room_navigation/config/*.rviz'))], install_requires=['setuptools'], zip_safe=True, tests_require=['pytest'], entry_points={'console_scripts':['room_navigation_node = soarm_room_navigation.ros_node:main','initial_map_pose = soarm_room_navigation.initial_map_pose:main']})
