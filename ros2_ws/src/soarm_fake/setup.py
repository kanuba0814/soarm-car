from setuptools import setup
from glob import glob
import os
setup(name='soarm_fake', version='0.1.0', packages=['soarm_fake'], data_files=[('share/ament_index/resource_index/packages', ['resource/soarm_fake']), ('share/soarm_fake', ['package.xml']), (os.path.join('share', 'soarm_fake', 'launch'), glob('soarm_fake/launch/*.launch.py')), (os.path.join('share', 'soarm_fake', 'urdf'), glob('soarm_fake/urdf/*')), (os.path.join('share', 'soarm_fake', 'rviz'), glob('soarm_fake/rviz/*'))], install_requires=['setuptools', 'PyYAML'], zip_safe=True, maintainer='soarm-car', maintainer_email='dev@example.invalid', license='Apache-2.0', entry_points={'console_scripts': ['fake_mission_server = soarm_fake.fake_mission_server:main', 'state_visualizer = soarm_fake.state_visualizer:main']})
