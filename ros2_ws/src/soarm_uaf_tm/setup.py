from setuptools import setup
setup(name='soarm_uaf_tm', version='0.1.0', packages=['soarm_uaf_tm'], data_files=[('share/ament_index/resource_index/packages', ['resource/soarm_uaf_tm']), ('share/soarm_uaf_tm', ['package.xml'])], install_requires=['setuptools'], zip_safe=True, entry_points={'console_scripts': ['uaf_tm_node = soarm_uaf_tm.node:main']})
