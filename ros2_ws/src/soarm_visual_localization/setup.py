from glob import glob
from setuptools import setup
import os

package_name = 'soarm_visual_localization'
setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    description='Pure RGB-D guarded odometry, mapping and localization nodes',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'rgbd_pointcloud = soarm_visual_localization.nodes:pointcloud_main',
            'visual_mapper = soarm_visual_localization.nodes:mapping_main',
            'visual_localizer = soarm_visual_localization.nodes:localization_main',
        ],
    },
)
