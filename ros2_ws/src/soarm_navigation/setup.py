from glob import glob
import os

from setuptools import setup


setup(
    name='soarm_navigation',
    version='0.1.0',
    packages=['soarm_navigation'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/soarm_navigation']),
        ('share/soarm_navigation', ['package.xml']),
        (os.path.join('share', 'soarm_navigation', 'launch'), glob('soarm_navigation/launch/*.launch.py')),
        (os.path.join('share', 'soarm_navigation', 'config'), glob('config/*')),
        (os.path.join('share', 'soarm_navigation', 'behavior_trees'), glob('behavior_trees/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
)
