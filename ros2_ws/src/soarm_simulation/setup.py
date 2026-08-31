from glob import glob
from setuptools import setup
import os
setup(name='soarm_simulation', version='0.1.0', packages=['soarm_simulation'], data_files=[
    ('share/ament_index/resource_index/packages', ['resource/soarm_simulation']),
    ('share/soarm_simulation', ['package.xml', 'README.md']),
    (os.path.join('share', 'soarm_simulation', 'launch'), glob('soarm_simulation/launch/*.launch.py')),
    (os.path.join('share', 'soarm_simulation', 'worlds'), glob('worlds/*')),
    (os.path.join('share', 'soarm_simulation', 'models', 'kuruma'), glob('models/kuruma/*')),
    (os.path.join('share', 'soarm_simulation', 'models', 'kuruma_ros2_control'), glob('models/kuruma_ros2_control/*')),
    (os.path.join('share', 'soarm_simulation', 'models', 'navigation_room'), glob('models/navigation_room/*')),
    (os.path.join('share', 'soarm_simulation', 'models', 'house_floorplan'), glob('models/house_floorplan/*')),
    (os.path.join('share', 'soarm_simulation', 'models', 'house_furniture'), glob('models/house_furniture/*')),
    (os.path.join('share', 'soarm_simulation', 'maps'), glob('maps/*')),
    (os.path.join('share', 'soarm_simulation', 'config'), glob('config/*'))],
    install_requires=['setuptools'], zip_safe=True,
    entry_points={'console_scripts': [
        'sim_adapter = soarm_simulation.sim_adapter:main',
        'odom_path = soarm_simulation.odom_path:main',
    ]})
