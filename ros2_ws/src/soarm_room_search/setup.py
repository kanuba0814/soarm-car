from setuptools import setup
from glob import glob
import os
setup(name='soarm_room_search',version='0.1.0',packages=['soarm_room_search'],data_files=[('share/ament_index/resource_index/packages',['resource/soarm_room_search']),('share/soarm_room_search',['package.xml']),(os.path.join('share/soarm_room_search','launch'),glob('soarm_room_search/launch/*.py')),(os.path.join('share/soarm_room_search','examples'),glob('examples/*'))],install_requires=['setuptools'],zip_safe=True,tests_require=['pytest'],entry_points={'console_scripts':['room_search_gui = soarm_room_search.http_gui:main','room_search_node = soarm_room_search.ros_node:main']})
