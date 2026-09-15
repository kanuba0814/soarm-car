from glob import glob
from setuptools import setup

package_name = 'soarm_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', glob('soarm_teleop/launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'keyboard_teleop = soarm_teleop.keyboard_teleop:main',
            'stm32_chassis_serial = soarm_teleop.stm32_chassis_serial:main',
        ],
    },
)
